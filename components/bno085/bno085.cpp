extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
}

#include "Arduino.h"
#include "Wire.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

#include "bno085_task_common.h"
#include "bno085.h"

// #define MAP_ANGLE(angle) (angle < 0 ? 2 * PI + angle : angle)

typedef enum {
	BNO085_ORI_BIT = 0,
	BNO085_ACCEL_BIT,
} bno85_data_source_e;

static const char TAG[] = "BNO085";

static bno085_state_t g_bno085_state = BNO085_STOPPED;
static BNO08x g_IMU;
static QueueHandle_t g_bn085_data_queue_handle = NULL;
static QueueHandle_t g_bn085_cmd_queue_handle = NULL;
static state_vector_t g_state_vector;
static byte g_reading_flag = 0x0;
static esp_timer_handle_t g_state_loop_timer;

typedef struct
{
	float accx;
	float accy;
	float accz;
} accel_data_t;

static accel_data_t g_imu_data = {
	.accx = 0.0f,
	.accy = 0.0f,
	.accz = 0.0f,
};

void bno085_init_integrator(state_vector_t &pose)
{
	pose.lat.integral = 0.0f;
	pose.lon.integral = 0.0f;
	pose.alt.integral = 0.0f;
	pose.lat.prev_diff_val = 0.0f;
	pose.lon.prev_diff_val = 0.0f;
	pose.alt.prev_diff_val = 0.0f;
	pose.vn.integral = 0.0f;
	pose.ve.integral = 0.0f;
	pose.vd.integral = 0.0f;
	pose.vn.prev_diff_val = 0.0f;
	pose.ve.prev_diff_val = 0.0f;
	pose.vd.prev_diff_val = 0.0f;
}

void bno085_set_state(bno085_state_t state)
{
	char state_str[10];
	switch (state)
	{
	case BNO085_OK:
		strcpy(state_str, "OK");
		break;
	case BNO085_STOPPED:
		strcpy(state_str, "STOPPED");
		break;
	case BNO085_RESET:
		strcpy(state_str, "RESET");
		break;
	default:
		strcpy(state_str, "UNKNOWN");
		break;
	}

	ESP_LOGI(TAG, "Setting state to %s", state_str);
	g_bno085_state = state;
}

static void bno085_update_pose(void *args)
{
	// Specific force to local geographic reference, angles must be in radians?
	static int64_t prev_time = 0;
	int64_t c_time = esp_timer_get_time();
	float delta_time = (c_time - prev_time) / 1E6; // Convert microseconds to seconds
	prev_time = c_time;

	float fn = g_imu_data.accx - g_state_vector.yaw * g_imu_data.accy + g_state_vector.pitch * g_imu_data.accz;
	float fe = g_state_vector.yaw * g_imu_data.accx + g_imu_data.accy - g_state_vector.roll * g_imu_data.accz;
	float fd = -g_state_vector.pitch * g_imu_data.accx + g_state_vector.roll * g_imu_data.accy + g_imu_data.accz;

	const float vn = g_state_vector.vn.integral;
	const float ve = g_state_vector.ve.integral;
	const float vd = g_state_vector.vd.integral;
	const float lat = DEG_TO_RAD * g_state_vector.lat.integral;
	const float alt = g_state_vector.alt.integral;
	const float R_0_alt = R_0 + alt;

	float vn_dot = fn - 2 * OMEGA_E * ve * sinf(lat) + (vn * vd - powf(ve, 2) * tanf(lat)) / (R_0_alt);
	float ve_dot = fe + 2 * OMEGA_E * (vn * sinf(lat) + ve * cosf(lat)) + (ve) / (R_0_alt) * (vd + vn * tanf(lat));
	float vd_dot = fd - 2 * OMEGA_E * ve * cosf(lat) - (powf(ve, 2) + powf(vn, 2)) / (R_0_alt);

	// Speed integration
	g_state_vector.vn.integral += delta_time * 0.5f * (g_state_vector.vn.prev_diff_val + vn_dot);
	g_state_vector.vn.prev_diff_val = vn_dot;

	g_state_vector.ve.integral += delta_time * 0.5f * (g_state_vector.ve.prev_diff_val + ve_dot);
	g_state_vector.ve.prev_diff_val = ve_dot;

	g_state_vector.vd.integral += delta_time * 0.5f * (g_state_vector.vd.prev_diff_val + vn_dot);
	g_state_vector.vd.prev_diff_val = vd_dot;

	// Position integration
	const float lat_dot = g_state_vector.vn.integral / (R_n + alt);
	const float lon_dot = g_state_vector.ve.integral / ((R_e + alt) * cosf(lat));
	const float alt_dot = -g_state_vector.vd.integral;

	g_state_vector.lat.integral += delta_time * 0.5f * (g_state_vector.lat.prev_diff_val + lat_dot);
	g_state_vector.lat.prev_diff_val = lat_dot;
	g_state_vector.lon.integral += delta_time * 0.5f * (g_state_vector.lon.prev_diff_val + lon_dot);
	g_state_vector.lon.prev_diff_val = lon_dot;
	g_state_vector.alt.integral += delta_time * 0.5f * (g_state_vector.alt.prev_diff_val + alt_dot);

	if (xQueueSend(g_bn085_data_queue_handle, &g_state_vector, pdMS_TO_TICKS(100)) != pdPASS)
	{
		ESP_LOGE(TAG, "Error sending to queue");
	}
}

esp_err_t bno085_config_reports(void)
{
	// while (g_IMU.enableAccelerometer() != true)
	//{
	//	ESP_LOGE(TAG, "Accel could not be enabled. Retrying...");
	//	vTaskDelay(pdMS_TO_TICKS(500));
	// }
	// ESP_LOGI(TAG, "Accel enabled");
	// ESP_LOGI(TAG, "Output in form x, y, z, in m/s2");

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

	bno085_set_state(BNO085_OK);

	return ESP_OK;
}

bno085_state_t bno085_get_state(void)
{
	return g_bno085_state;
}

esp_err_t bno085_get_queue_handle(QueueHandle_t *queue)
{
	ESP_RETURN_ON_FALSE(g_bn085_data_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");

	*queue = g_bn085_data_queue_handle;

	return ESP_OK;
}

void bno085_info_reader(void)
{
	if (g_IMU.wasReset())
	{
		ESP_LOGE(TAG, "Sensor was reset");

		bno085_set_state(BNO085_RESET);

		ESP_ERROR_CHECK(bno085_config_reports());
	}
	if (g_IMU.getSensorEvent() == true)
	{
		if (g_IMU.getSensorEventID() == SENSOR_REPORTID_LINEAR_ACCELERATION)
		{
			g_imu_data.accx = g_IMU.getLinAccelX();
			g_imu_data.accy = g_IMU.getLinAccelY();
			g_imu_data.accz = g_IMU.getLinAccelZ();
			byte accel_acc = g_IMU.getAccelAccuracy();

			g_reading_flag = g_reading_flag | 1 << BNO085_ACCEL_BIT;

			// printf("%f,%f,%f,%d\r\n", g_imu_data.accx, g_imu_data.accy, g_imu_data.accz, accel_acc);
		}
		else if (g_IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
		{
			g_state_vector.roll = g_IMU.getRoll();
			g_state_vector.pitch = g_IMU.getPitch();
			g_state_vector.yaw = g_IMU.getYaw();

			g_reading_flag = g_reading_flag | 1 << BNO085_ORI_BIT;
			// printf("/*rot,x,%f,y,%f,z,%f,calib,%d*/\r\n", RAD_TO_DEG * g_state_vector.roll, RAD_TO_DEG * g_state_vector.pitch, RAD_TO_DEG * g_state_vector.yaw, 0);
		}

		// Check if both readings have been received
		if(g_reading_flag == 0b00000011)
		{
			bno085_update_pose(NULL);
			g_reading_flag = 0;
		}
	}
}

static void bno085_task(void *pvParamenters)
{
	ESP_LOGI(TAG, "Initializing task");

	g_bn085_data_queue_handle = xQueueCreate(5, sizeof(state_vector_t));

	Wire.begin();

	while (g_IMU.begin(BNO085_ADDR, Wire, BNO085_INT, BNO085_RST) == false)
	{
		ESP_LOGE(TAG, "BNO085 not detected at default I2C address. Retrying..");
		vTaskDelay(pdMS_TO_TICKS(500));
	}

	ESP_ERROR_CHECK(bno085_config_reports());

	bno085_init_integrator(g_state_vector);

	// Iniatilize the state vector
	g_state_vector.alt.integral = INIT_ALT;
	g_state_vector.lat.integral = INIT_LAT;
	g_state_vector.lon.integral = INIT_LON;

	for (;;)
	{
		bno085_info_reader();
		vTaskDelay(pdMS_TO_TICKS(10));
	}

}

void bno085_start_task(void)
{
	ESP_LOGI(TAG, "Starting task");

	xTaskCreatePinnedToCore(&bno085_task, "bno085", BNO085_STACK_SIZE,
							NULL, BNO085_TASK_PRIORITY, NULL, BNO085_CORE_ID);
}