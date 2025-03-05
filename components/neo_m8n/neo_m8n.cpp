#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "TinyGPS++.h"

#include "neo_m8n_task_common.h"
#include "neo_m8n.h"

static const char TAG[] = "neo_m8n";
static QueueHandle_t g_gps_queue_handle = NULL;

static TinyGPSPlus g_neo_m8n;

esp_err_t neo_m8n_send2queue(gps_coords_t &coord)
{
    ESP_RETURN_ON_FALSE(g_gps_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is not initialized!");

    return xQueueSend(g_gps_queue_handle, &coord, portMAX_DELAY) == pdPASS ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

QueueHandle_t neo_m8n_get_queue(void)
{
    return g_gps_queue_handle;
}

static void neo_m8n_task(void *pvParamenter)
{
    ESP_LOGI(TAG, "Iniatilizing task");

    // UART configuration
    const uart_port_t uart_num = UART_NUM_1;
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

    //ESP_ERROR_CHECK(uart_set_pin(uart_num, NEO_M8N_UART_TX, NEO_M8N_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, NEO_M8N_UART_TX, NEO_M8N_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(uart_num, NEO_M8N_UART_BUFF_SIZE * 2, 0, 0, NULL, 0));

    g_gps_queue_handle = xQueueCreate(4, sizeof(gps_coords_t));

    uint8_t data[NEO_M8N_UART_BUFF_SIZE];

    for (int i = 0; i < NEO_M8N_UART_BUFF_SIZE; i++)
    {
        data[i] = 0;
    }

    for (;;)
    {
        // Read data from UART
        int len = uart_read_bytes(uart_num, data, NEO_M8N_UART_BUFF_SIZE, pdMS_TO_TICKS(100));

        //printf("Data received: %d\n", len);

        if (len > 0)
        {
            //ESP_LOGI(TAG, "Data received: %d", len);
            for (int i = 0; i < len; i++)
            {
                g_neo_m8n.encode(data[i]); // Feed data into TinyGPS++
            }
            printf("%s\n", data);
            // Check if location is updated
            if (g_neo_m8n.location.isUpdated())
            {

                gps_coords_t coord = {
                    .lat = g_neo_m8n.location.lat(),
                    .lon = g_neo_m8n.location.lng(),
                    .alt = g_neo_m8n.altitude.meters(),
                    .hdop = g_neo_m8n.hdop.hdop()};
                ESP_ERROR_CHECK(neo_m8n_send2queue(coord));

#if true 
                printf("Lat: %.7f, Lng: %.7f, Alt: %.2f meters, HDOP: %.2f",
                        coord.lat, coord.lon, coord.alt, coord.hdop);
#endif
            };

            for (int i = 0; i < NEO_M8N_UART_BUFF_SIZE; i++)
            {
                data[i] = 0;
            }
        }
        
        //uart_flush(uart_num);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}

void neo_m8n_task_start(void)
{
    ESP_LOGI(TAG, "Staring task!");

    xTaskCreatePinnedToCore(&neo_m8n_task, "neo_m8n_task",
                            NEO_M8N_STACK_SIZE, NULL, NEO_M8N_TASK_PRIORITY, NULL, NEO_M8N_CORE_ID);
}