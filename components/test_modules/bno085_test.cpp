extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
}

#include "Arduino.h"

#include "bno085_test.h"
#include "bno085.h"
#include "bno085_task_common.h"

const char *TAG = "BNO085_TEST";
static QueueHandle_t g_imu_cmd_handle = NULL;
static QueueHandle_t g_imu_data_handle = NULL;
static bool g_started_flag = false;

void test_start(void)
{
    geodesic_point_t origin = {
        .lat = INIT_LAT,
        .lon = INIT_LON,
        .alt = INIT_ALT,
    };
    imu_cmd_t cmd_origin = {
        .cmd = IMU_CMD_SET_ORIGIN,
        .origin = &origin,
    };

    if(xQueueSend(g_imu_cmd_handle, &cmd_origin, pdMS_TO_TICKS(200)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending origin cmd");
    }
    
    imu_cmd_t cmd_start = {
        .cmd = IMU_CMD_START,
        .origin = NULL,
    };
    if(xQueueSend(g_imu_cmd_handle, &cmd_start, pdMS_TO_TICKS(200)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending start cmd");
    }

    g_started_flag = true;
}

void test_get_imu_data(void)
{
    static state_vector_t state_vector;

    if (g_started_flag)
    {
        if (xQueueReceive(g_imu_data_handle, &state_vector, pdMS_TO_TICKS(100)) == pdTRUE)
        {
#if true
            printf("Lat: %f, Lon: %f, Alt: %f\r\n", state_vector.lat.integral, state_vector.lon.integral, state_vector.alt.integral);
// printf("/*%f,%f,%f*/\r\n", state_vector.lat.integral, state_vector.lon.integral, state_vector.alt.integral);
#endif
            // continue;
        }
    }
}

static void test_bno085_task(void *pvParameters)
{
    /* Get queue handlers*/
    while (imu_get_cmd_queue_handle(&g_imu_cmd_handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "Cannot get cmd handle. Retrying...");
    }
    while (imu_get_data_queue_handle(&g_imu_data_handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "Cannot get data handle. Retrying...");
    }

    ESP_LOGI(TAG, "Waiting for starting key!");
    for (;;)
    {
        if (Serial.available())
        {
            char c = Serial.read();

            if(c == 's')
            {
                test_start();
            }
        }

        test_get_imu_data();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void test_bno085_task_start(void)
{
    ESP_LOGI(TAG, "Starting BNO085 test task");
    xTaskCreatePinnedToCore(test_bno085_task, "bno085_task", 4098, NULL, 2, NULL, 0);
}