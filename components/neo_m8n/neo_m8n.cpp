#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_check.h"

#include "TinyGPS++.h"

#include "neo_m8n_task_common.h"
#include "neo_m8n.h"

#define DEG2RAD(x) (x * M_PI / 180.0f)

typedef struct
{
    float x;
    float y;
    float z;
    float n;
    float e;
    float d;
} gps_coords_t;

static const char TAG[] = "neo_m8n";
static QueueHandle_t g_uart_queue_handle;
static QueueHandle_t g_gps_queue_handle = NULL;

static gps_coords_t g_gps_origin;
static bool g_origin_set = false;

float g_mat_ecef2ned[3][3];
static TinyGPSPlus g_neo_m8n;

void neo_m8n_geo2ecef(float lat, float lon, float alt, gps_coords_t &coord)
{
    float N0 = ECEF_A / sqrtf(1 - ECEF_E_2 * powf(sinf(lat), 2));
    coord.x = (N0 + alt) * cosf(lat) * cosf(lon);
    coord.y = (N0 + alt) * cosf(lat) * sinf(lon);
    coord.z = (N0 * (1 - ECEF_E_2) + alt) * sinf(lat);
}

esp_err_t neo_m8n_geo2ned(float lat, float lon, float alt, gps_coords_t &coord)
{
    ESP_RETURN_ON_FALSE(g_origin_set, ESP_ERR_INVALID_STATE, TAG, "Origin is not set yet!");
    neo_m8n_geo2ecef(lat, lon, alt, coord);

    float x_diff = coord.x - g_gps_origin.x;
    float y_diff = coord.y - g_gps_origin.y;
    float z_diff = coord.z - g_gps_origin.z;

    coord.n = g_mat_ecef2ned[0][0] * x_diff + g_mat_ecef2ned[0][1] * y_diff + g_mat_ecef2ned[0][2] * z_diff;
    coord.e = g_mat_ecef2ned[1][0] * x_diff + g_mat_ecef2ned[1][1] * y_diff + g_mat_ecef2ned[1][2] * z_diff;
    coord.d = g_mat_ecef2ned[2][0] * x_diff + g_mat_ecef2ned[2][1] * y_diff + g_mat_ecef2ned[2][2] * z_diff;

    ESP_RETURN_ON_FALSE(xQueueSend(g_gps_queue_handle, &coord, portMAX_DELAY) == pdPASS, ESP_ERR_INVALID_RESPONSE, TAG, "Error sending to queue");

    return ESP_OK;
}

void neo_m8n_set_origin(float lat, float lon, float alt)
{
    if (g_origin_set)
        ESP_LOGW(TAG, "Origin will be overwritten!");

    neo_m8n_geo2ecef(lat, lon, alt, g_gps_origin);

    g_mat_ecef2ned[0][0] = -sinf(lat) * cosf(lon);
    g_mat_ecef2ned[0][1] = -sinf(lat) * sinf(lat);
    g_mat_ecef2ned[0][2] = cosf(lat);
    g_mat_ecef2ned[1][0] = -sinf(lon);
    g_mat_ecef2ned[1][1] = cosf(lon);
    g_mat_ecef2ned[1][2] = 0.0f;
    g_mat_ecef2ned[2][0] = -cosf(lat) * cosf(lon);
    g_mat_ecef2ned[2][1] = -cosf(lat) * sinf(lon);
    g_mat_ecef2ned[2][2] = -sinf(lat);

    g_origin_set = true;
}


QueueHandle_t neo_m8n_get_queue(void)
{
    return g_gps_queue_handle;
}

static void neo_m8n_task(void *pvParamenter)
{
    ESP_LOGI(TAG, "Iniatilizing task");

    // UART configuration
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(uart_num, NEO_M8N_UART_TX, NEO_M8N_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(uart_num, NEO_M8N_UART_BUFF_SIZE, NEO_M8N_UART_BUFF_SIZE, 10, &g_uart_queue_handle, 0));

    g_gps_origin = (gps_coords_t){
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
        .n = 0.0f,
        .e = 0.0f,
        .d = 0.0f,
    };

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            g_mat_ecef2ned[i][j] = 0.0f;
        }
    }

    g_gps_queue_handle = xQueueCreate(4, sizeof(gps_coords_t));

    uint8_t data[NEO_M8N_UART_BUFF_SIZE];

    for (;;)
    {
        // Read data from UART
        int len = uart_read_bytes(uart_num, data, NEO_M8N_UART_BUFF_SIZE, pdMS_TO_TICKS(100));

        if (len > 0)
        {
            for (int i = 0; i < len; i++)
            {
                g_neo_m8n.encode(data[i]); // Feed data into TinyGPS++
            }
            // Check if location is updated
            if (g_neo_m8n.location.isUpdated())
            {
                float lat = DEG2RAD(g_neo_m8n.location.lat());
                float lon = DEG2RAD(g_neo_m8n.location.lng());
                float alt = g_neo_m8n.altitude.meters();

                if (!g_origin_set)
                {
                    // Origin is set with the first lecture, however it can be modified later if necessary
                    neo_m8n_set_origin(lat, lon, alt);
                }
                else
                {
                    gps_coords_t coords;
                    ESP_ERROR_CHECK(neo_m8n_geo2ned(lat, lon, alt, coords));
                }

                printf("Lat: %.7f, Lng: %.7f, Alt: %.2f meters, HDOP: %.2f",
                       g_neo_m8n.location.lat(), g_neo_m8n.location.lng(), g_neo_m8n.altitude.meters(), g_neo_m8n.hdop.hdop());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void neo_m8n_task_start(void)
{
    ESP_LOGI(TAG, "Staring task!");

    xTaskCreatePinnedToCore(&neo_m8n_task, "neo_m8n_task",
                            NEO_M8N_STACK_SIZE, NULL, NEO_M8N_TASK_PRIORITY, NULL, NEO_M8N_CORE_ID);
}