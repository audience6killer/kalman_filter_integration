
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
}

#include "ekf.h"
#include "bno085.h"
#include "kalman_filter.h"
#include "kalman_test.h"

#define KALMAN_TEST_STACK_SIZE 1024 * 7
#define KALMAN_TEST_CORE_ID 0
#define KALMAN_TEST_TASK_PRIORITY 15

static const char *TAG = "kalman_test";

static Nav_EKF *kalman_filter = NULL;
static QueueHandle_t g_imu_cmd_queue_handle = NULL;
static QueueHandle_t g_imu_data_queue_handle = NULL;

static void test_kalman_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing kalman_test task");

    // Create EKF
    kalman_filter = new Nav_EKF();

    // Initialize queues
    while(imu_get_data_queue_handle(&g_imu_data_queue_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get imu_data_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while(imu_get_cmd_queue_handle(&g_imu_cmd_queue_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get imu_cmd_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Start imu
    geodesic_point_t origin = {
        .lat = INIT_LAT,
        .lon = INIT_LON,
        .alt = INIT_ALT,
    };
    imu_cmd_t cmd_origin = {
        .cmd = IMU_CMD_SET_ORIGIN,
        .origin = &origin,
    };

    if(xQueueSend(g_imu_cmd_queue_handle, &cmd_origin, pdMS_TO_TICKS(200)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending origin cmd");
    }
    
    imu_cmd_t cmd_start = {
        .cmd = IMU_CMD_START,
        .origin = NULL,
    };
    if(xQueueSend(g_imu_cmd_queue_handle, &cmd_start, pdMS_TO_TICKS(200)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending start cmd");
    }

    state_vector_t c_state;

    static uint32_t last_time = esp_timer_get_time();
    for (;;)
    {
        if(xQueueReceive(g_imu_data_queue_handle, &c_state, pdMS_TO_TICKS(100)))
        {
            uint32_t c_time = esp_timer_get_time();
            uint32_t time_diff = c_time - last_time;
            time_diff *= 1e-6;
            last_time = c_time;

            dspm::Mat state(9, 1);
            state *= 0.0;

            state(0, 0) = c_state.lat.integral;
            state(1, 0) = c_state.lon.integral;
            state(2, 0) = c_state.alt.integral;
            state(3, 0) = c_state.vn.integral;
            state(4, 0) = c_state.ve.integral;
            state(5, 0) = c_state.vd.integral;
            state(6, 0) = c_state.fn;
            state(7, 0) = c_state.fe;
            state(8, 0) = c_state.fd;

            //kalman_filter->UpdateNominalSystem(state);
            kalman_filter->UpdateNominalSystem(c_state.lat.integral, c_state.lon.integral, c_state.alt.integral, c_state.vn.integral, c_state.ve.integral, c_state.vd.integral, c_state.fn, c_state.fe, c_state.fd);

            kalman_filter->Process(0, (float)time_diff);

            kalman_filter->PrintXState();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void test_kalman_task_start(void)
{
    ESP_LOGI(TAG, "Starting kalman_test task");

    xTaskCreatePinnedToCore(test_kalman_task, "kalman_test", KALMAN_TEST_STACK_SIZE, NULL, KALMAN_TEST_TASK_PRIORITY, NULL, KALMAN_TEST_CORE_ID);
}