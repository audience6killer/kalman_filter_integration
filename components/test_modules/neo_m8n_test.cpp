#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "neo_m8n_test.h"
#include "neo_m8n.h"

const char *TAG = "NEO_M8N_TEST";

static void test_neo_m8n_task(void *pvParameters)
{
    neo_m8n_task_start();

    QueueHandle_t gps_queue;
    QueueHandle_t gps_cmd_queue;

    neo_m8n_get_data_queue(&gps_queue);
    neo_m8n_get_cmd_queue(&gps_cmd_queue);

    gps_coords_t state_vector;

    neo_m8n_cmd_t cmd = {
        .cmd = NEO_M8N_CMD_START,
    };

    xQueueSend(gps_cmd_queue, &cmd, pdMS_TO_TICKS(100));

    for (;;)
    {
        if (xQueueReceive(gps_queue, &state_vector, portMAX_DELAY) == pdTRUE)
        {
#if true
            // printf("Lat: %f, Lon: %f, Alt: %f\r\n", state_vector.lat.integral, state_vector.lon.integral, state_vector.alt.integral);
            printf("/*%.4f,%.4f,%.4f,%.4f*/\r\n", state_vector.lat, state_vector.lon, state_vector.alt, state_vector.hdop);
#endif
            // continue;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void test_neo_m8n_task_start(void)
{
    ESP_LOGI(TAG, "Starting NEO_M8N test task");
    xTaskCreatePinnedToCore(test_neo_m8n_task, "neo_m8n_task", 4098, NULL, 2, NULL, 0);
}