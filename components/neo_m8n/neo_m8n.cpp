#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "TinyGPS++.h"

#include "neo_m8n_task_common.h"
#include "neo_m8n.h"

// #define GPS_DEBUG

static const char TAG[] = "neo_m8n";
static QueueHandle_t g_gps_data_queue_handle = NULL;
static QueueHandle_t g_gps_cmd_queue_handle = NULL;
static neo_m8n_state_e g_gps_state = NEO_M8N_STATE_STOPPED;
static gps_coords_t g_last_coord;

static EventGroupHandle_t g_gps_event_group = NULL; 
static EventGroupHandle_t g_gps_error_group = NULL; 

static TinyGPSPlus g_neo_m8n;

/* Getters*/
esp_err_t neo_m8n_get_data_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_gps_data_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "g_gps_data_queue NULL when retriving");

    *handle = g_gps_data_queue_handle;

    return ESP_OK;
}

esp_err_t neo_m8n_get_cmd_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_gps_cmd_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "g_gps_cmd_queue NULL when retriving");

    *handle = g_gps_cmd_queue_handle;

    return ESP_OK;
}

esp_err_t neo_m8n_get_event_group(EventGroupHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_gps_event_group != NULL, ESP_ERR_INVALID_STATE, TAG, "g_gps_event_group NULL when retriving");

    *handle = g_gps_event_group;

    return ESP_OK;
}

esp_err_t neo_m8n_get_error_group(EventGroupHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_gps_error_group != NULL, ESP_ERR_INVALID_STATE, TAG, "g_gps_error_group NULL when retriving");

    *handle = g_gps_error_group;

    return ESP_OK;
}

esp_err_t neo_m8n_get_last_coord(gps_coords_t *coord)
{
    if(g_gps_state != NEO_M8N_STATE_STARTED)
    {
        ESP_LOGE(TAG, "Error: GPS has not started receiving data!");
        return ESP_FAIL;
    }

    *coord = g_last_coord;
    return ESP_OK;
}

/* Queue & Event Groups */
inline esp_err_t neo_m8n_send2queue(gps_coords_t &coord)
{
    return xQueueSend(g_gps_data_queue_handle, &coord, portMAX_DELAY) == pdPASS ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

inline esp_err_t neo_m8n_set_state(neo_m8n_state_e state)
{
    g_gps_state = state;
    xEventGroupSetBits(g_gps_event_group, state);

    return ESP_OK;
}

/* Event handlers */
esp_err_t neo_m8n_start_event_handler(void)
{
    if (g_gps_state == NEO_M8N_STATE_STARTED)
    {
        ESP_LOGW(TAG, "Warning: Neo M8N is already started");
        return ESP_OK;
    }

    g_gps_state = NEO_M8N_STATE_STARTED;
    ESP_LOGI(TAG, "Process started successfully");

    return ESP_OK;
}

esp_err_t neo_m8n_stop_event_handler(void)
{
    if (g_gps_state == NEO_M8N_STATE_STOPPED)
    {
        ESP_LOGW(TAG, "Warning: Neo M8N is already stopped");
        return ESP_OK;
    }

    neo_m8n_set_state(NEO_M8N_STATE_STOPPED);
    ESP_LOGI(TAG, "Process stopped successfully");

    return ESP_OK;
}

void neo_m8n_event_loop(void)
{
    neo_m8n_cmd_t cmd;
    if (xQueueReceive(g_gps_cmd_queue_handle, &cmd, pdMS_TO_TICKS(10)))
    {
        switch (cmd.cmd)
        {
        case NEO_M8N_CMD_STOP:
            ESP_LOGI(TAG, "Event Received: Stop");
            if (neo_m8n_stop_event_handler() != ESP_OK)
            {
                ESP_LOGE(TAG, "Error: Failed to process stop event");
            }
            break;
        case NEO_M8N_CMD_START:
            ESP_LOGI(TAG, "Event Received: Stop");
            if (neo_m8n_start_event_handler() != ESP_OK)
            {
                ESP_LOGE(TAG, "Error: Failed to process start event");
            }
            break;
        default:
            ESP_LOGE(TAG, "Error: Unknown event received");
            break;
        }
    }
}

/* Data acquisition */
void neo_m8n_data_loop(void)
{
    static uint8_t data[NEO_M8N_UART_BUFF_SIZE];
    memset(data, 0, NEO_M8N_UART_BUFF_SIZE);

    // Read data from UART
    int len = uart_read_bytes(NEO_M8N_UART_NUM, data, NEO_M8N_UART_BUFF_SIZE, pdMS_TO_TICKS(100));

    if (len > 0)
    {
        for (int i = 0; i < len; i++)
        {
            g_neo_m8n.encode(data[i]); 
        }

        if (g_neo_m8n.location.isUpdated())
        {
            // if(g_gps_state != NEO_M8N_STATE_STARTED)
            // {
            //     neo_m8n_set_state(NEO_M8N_STATE_STARTED);
            // }
            neo_m8n_set_state(NEO_M8N_STATE_STARTED);

            gps_coords_t coord = {
                .lat = g_neo_m8n.location.lat(),
                .lon = g_neo_m8n.location.lng(),
                .alt = g_neo_m8n.altitude.meters(),
                .hdop = (float)g_neo_m8n.hdop.hdop()};
            g_last_coord = coord;
            ESP_ERROR_CHECK(neo_m8n_send2queue(coord));

#ifdef GPS_DEBUG
            printf("/*%.7f,%.7f,%.7f,%.2f*/\n",
                   coord.lat, coord.lon, coord.alt, coord.hdop);
#endif
        };

    }
}

/* Main task */
static void neo_m8n_task(void *pvParamenter)
{
    ESP_LOGI(TAG, "Iniatilizing task");

    // UART configuration
    const uart_port_t uart_num = NEO_M8N_UART_NUM;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // ESP_ERROR_CHECK(uart_set_pin(uart_num, NEO_M8N_UART_TX, NEO_M8N_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, NEO_M8N_UART_TX, NEO_M8N_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(uart_num, NEO_M8N_UART_BUFF_SIZE * 2, 0, 0, NULL, 0));

    g_gps_data_queue_handle = xQueueCreate(4, sizeof(gps_coords_t));
    g_gps_cmd_queue_handle = xQueueCreate(5, sizeof(neo_m8n_cmd_t));

    for (;;)
    {
        if (g_gps_state == NEO_M8N_STATE_STARTED)
        {
            neo_m8n_data_loop();
        }

        neo_m8n_event_loop();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void neo_m8n_task_start(void)
{
    ESP_LOGI(TAG, "Staring task!");

    g_gps_event_group = xEventGroupCreate();
    g_gps_error_group = xEventGroupCreate();

    xTaskCreatePinnedToCore(&neo_m8n_task, "neo_m8n_task",
                            NEO_M8N_STACK_SIZE, NULL, NEO_M8N_TASK_PRIORITY, NULL, NEO_M8N_CORE_ID);
}