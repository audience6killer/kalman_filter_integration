
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "common_types.h"
}

#include "ekf.h"
#include "neo_m8n.h"
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
static QueueHandle_t g_gps_cmd_queue_handle = NULL;
static QueueHandle_t g_gps_data_queue_handle = NULL;

static EventGroupHandle_t g_gps_event_group = NULL;

    static void
    test_kalman_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing kalman_test task");

    // Create EKF
    kalman_filter = new Nav_EKF();

    // Initialize queues
    while (imu_get_data_queue_handle(&g_imu_data_queue_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get imu_data_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (imu_get_cmd_queue_handle(&g_imu_cmd_queue_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get imu_cmd_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (neo_m8n_get_data_queue(&g_gps_data_queue_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get gps_data_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (neo_m8n_get_cmd_queue(&g_gps_cmd_queue_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get gps_cmd_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (neo_m8n_get_event_group(&g_gps_event_group) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get gps_event_group. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Start GPS and get origin */
    neo_m8n_cmd_t cmd_gps_start = {
        .cmd = NEO_M8N_CMD_START,
    };
    if (xQueueSend(g_gps_cmd_queue_handle, &cmd_gps_start, pdMS_TO_TICKS(200)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending start cmd to gps");
    }

    EventBits_t bits = 0; 
    do
    {
        ESP_LOGI(TAG, "Waiting 10s for GPS to start");
        bits = xEventGroupWaitBits(g_gps_event_group, NEO_M8N_STATE_STARTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
    } while ((bits & NEO_M8N_STATE_STARTED) == 0);

    gps_coords_t coord;
    neo_m8n_get_last_coord(&coord);

    // Start imu
    gps_coords_t origin = {
        .lat = coord.lat,
        .lon = coord.lon,
        .alt = coord.alt,
    };
    imu_cmd_t cmd_origin = {
        .cmd = IMU_CMD_SET_ORIGIN,
        .origin = &origin,
    };

    if (xQueueSend(g_imu_cmd_queue_handle, &cmd_origin, pdMS_TO_TICKS(200)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending origin cmd");
    }

    imu_cmd_t cmd_start = {
        .cmd = IMU_CMD_START,
        .origin = NULL,
    };
    if (xQueueSend(g_imu_cmd_queue_handle, &cmd_start, pdMS_TO_TICKS(200)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending start cmd");
    }

    state_vector_t c_state;
    gps_coords_t gps_data;

    static uint32_t last_time = esp_timer_get_time();
    for (;;)
    {
        if (xQueueReceive(g_imu_data_queue_handle, &c_state, pdMS_TO_TICKS(100)))
        {
            uint32_t c_time = esp_timer_get_time();
            uint32_t time_diff = c_time - last_time;
            time_diff *= 1e-6;
            last_time = c_time;

            // kalman_filter->UpdateNominalSystem(state);
            //  Increase nominal state to include angles
            kalman_filter->UpdateNominalSystem(c_state.lat.integral, c_state.lon.integral, c_state.alt.integral, c_state.vn.integral, c_state.ve.integral, c_state.vd.integral, c_state.fn, c_state.fe, c_state.fd, c_state.yaw);

            kalman_filter->Process(0, (float)time_diff);

            printf("NCF: ");
            kalman_filter->PrintXState();
        }

        if (xQueueReceive(g_gps_data_queue_handle, &gps_data, pdMS_TO_TICKS(10)))
        {
            float measured[] = {(float)gps_data.lat, (float)gps_data.lon, (float)gps_data.alt, 0};
            float R[] = {(float)gps_data.hdop, (float)gps_data.hdop, (float)gps_data.hdop, (float)gps_data.hdop};
            kalman_filter->Update(measured, R, Nav_EKF::MeasureSource::GPS);

            printf("GPS: ");
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