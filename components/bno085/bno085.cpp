extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
}

#include "Arduino.h"
#include "Wire.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

#include "bno085_task_common.h"
#include "bno085.h"

#define MAP_ANGLE(angle) (angle < 0 ? 2 * PI + angle : angle)

static const char TAG[] = "BNO085";

typedef struct
{
	float accx;
	float accy;
	float accz;
} accel_data_t;

static BNO08x g_IMU;
static QueueHandle_t g_bno085_queue_handle;
static accel_data_t g_imu_data = {
	.accx = 0.0f,
	.accy = 0.0f,
	.accz = 0.0f,
};
static state_vector_t g_state_vector;

void init_integrator(state_vector_t &pose)
{
	pose.north.integral = 0.0f;
	pose.east.integral = 0.0f;
	pose.down.integral = 0.0f;
	pose.north.prev_value = 0.0f;
	pose.east.prev_value = 0.0f;
	pose.down.prev_value = 0.0f;
	pose.north_p.integral = 0.0f;
	pose.east_p.integral = 0.0f;
	pose.down_p.integral = 0.0f;
	pose.north_p.prev_value = 0.0f;
	pose.east_p.prev_value = 0.0f;
	pose.down_p.prev_value = 0.0f;
}

static void position_update_loop(void *args)
{
	float fn = g_imu_data.accx - g_state_vector.yaw * g_imu_data.accy + g_state_vector.pitch * g_imu_data.accz;
	float fe = g_state_vector.yaw * g_imu_data.accx + g_imu_data.accy - g_state_vector.roll * g_imu_data.accz;
	float fd = -g_state_vector.pitch * g_imu_data.accx + g_state_vector.roll * g_imu_data.accy + g_imu_data.accz;

	// Speed integration
	g_state_vector.north_p.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.north_p.prev_value + fn);
	g_state_vector.north_p.prev_value = fn;
	g_state_vector.east_p.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.east_p.prev_value + fe);
	g_state_vector.east_p.prev_value = fe;
	g_state_vector.down_p.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.down_p.prev_value + fd);
	g_state_vector.down_p.prev_value = fd;

	// Position integration
	g_state_vector.north.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.north.prev_value + g_state_vector.north_p.integral);
	g_state_vector.north.prev_value = g_state_vector.north_p.integral;
	g_state_vector.east.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.east.prev_value + g_state_vector.east_p.integral);
	g_state_vector.east.prev_value = g_state_vector.east_p.integral;
	g_state_vector.down.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.down.prev_value + g_state_vector.down_p.integral);
	g_state_vector.down.prev_value = g_state_vector.down_p.integral;

	if (xQueueSend(g_bno085_queue_handle, &g_state_vector, portMAX_DELAY) != pdPASS)
	{
		ESP_LOGE(TAG, "Error sending to queue");
	}
}

esp_err_t bno085_config_reports(void)
{

	while (g_IMU.enableLinearAccelerometer() != true)
	{
		ESP_LOGE(TAG, "Linear Accel could not be enabled. Retrying...");
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	ESP_LOGI(TAG, "Linear Accel enabled");
	ESP_LOGI(TAG, "Output in form x, y, z, in m/s2");

	while (g_IMU.enableRotationVector() != true)
	{
		ESP_LOGE(TAG, "Rotation vector could not be enabled. Retrying...");
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	ESP_LOGI(TAG, "Rotation vector enabled");
	ESP_LOGI(TAG, "Output in form x, y, z, in rad");

	return ESP_OK;
}

static void bno085_task(void *pvParamenters)
{
	ESP_LOGI(TAG, "Creating task");

	Wire.begin();

	while (g_IMU.begin(BNO085_ADDR, Wire, BNO085_INT, BNO085_RST) == false)
	{
		ESP_LOGE(TAG, "BNO085 not detected at default I2C address.");
		vTaskDelay(pdMS_TO_TICKS(500));
	}

	ESP_ERROR_CHECK(bno085_config_reports());

	init_integrator(g_state_vector);

	g_bno085_queue_handle = xQueueCreate(4, sizeof(state_vector_t));

	for (;;)
	{
		if (g_IMU.wasReset())
		{
			ESP_LOGE(TAG, "Sensor was reset");
			ESP_ERROR_CHECK(bno085_config_reports());
		}
		if (g_IMU.getSensorEvent() == true)
		{

			if (g_IMU.getSensorEventID() == SENSOR_REPORTID_LINEAR_ACCELERATION)
			{
				g_imu_data.accx = g_IMU.getLinAccelX();
				g_imu_data.accy = g_IMU.getLinAccelY();
				g_imu_data.accz = g_IMU.getLinAccelZ();
				byte accel_acc = g_IMU.getLinAccelAccuracy();

				// printf("%f,%f,%f,%d\r\n", g_imu_data.accx, g_imu_data.accy, g_imu_data.accz, accel_acc);
			}
			else if (g_IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
			{
				g_state_vector.roll = MAP_ANGLE(g_IMU.getRoll());
				g_state_vector.pitch = MAP_ANGLE(g_IMU.getPitch());
				g_state_vector.yaw = MAP_ANGLE(g_IMU.getYaw());
				// printf("/*rot,x,%f,y,%f,z,%f,calib,%d*/\r\n", x, y, z, 0);
			}

		}
		
	}

	vTaskDelay(pdMS_TO_TICKS(1));
}


void bno085_start_task(void)
{
	ESP_LOGI(TAG, "Iniatilizing task");

	xTaskCreatePinnedToCore(&bno085_task, "bno085", BNO085_STACK_SIZE,
							NULL, BNO085_TASK_PRIORITY, NULL, BNO085_CORE_ID);
}