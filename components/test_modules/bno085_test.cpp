#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "bno085_test.h"
#include "bno085.h"

const char *TAG = "BNO085_TEST";

static void test_bno085_task(void *pvParameters)
{
    bno085_start_task();

    QueueHandle_t queue; 

    if(bno085_get_queue_handle(&queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting queue handle");
        vTaskDelete(NULL);
    }
    
    state_vector_t state_vector;
    
    for (;;)
    {
        if(bno085_get_state() == BNO085_OK)
        {
            if(xQueueReceive(queue, &state_vector, portMAX_DELAY) == pdTRUE)
            {
                printf("Lat: %f, Lon: %f, Alt: %f\r\n", state_vector.lat.integral, state_vector.lon.integral, state_vector.alt.integral); 
                continue;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void test_bno085_task_start(void)
{
    ESP_LOGI(TAG, "Starting BNO085 test task");
    xTaskCreatePinnedToCore(test_bno085_task, "bno085_task", 4098, NULL, 2, NULL, 0);
}