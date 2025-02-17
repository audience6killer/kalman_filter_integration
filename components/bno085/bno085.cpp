extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

static const char TAG[] = "BNO085";

static bno085_state_t g_bno085_state = BNO085_STOPPED;
static BNO08x g_IMU;
static QueueHandle_t g_bno085_queue_handle = NULL;
static state_vector_t g_state_vector;
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

void init_integrator(state_vector_t &pose)
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

static void position_update_loop(void *args)
{
	// Specific force to local geographic reference
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
	g_state_vector.vn.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.vn.prev_diff_val + vn_dot);
	g_state_vector.vn.prev_diff_val = vn_dot;
	
	g_state_vector.ve.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.ve.prev_diff_val + ve_dot);
	g_state_vector.ve.prev_diff_val = ve_dot;

	g_state_vector.vd.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.vd.prev_diff_val + vn_dot);
	g_state_vector.vd.prev_diff_val = vd_dot;

	// Position integration
	const float lat_dot = g_state_vector.vn.integral / (R_n + alt);
	const float lon_dot = g_state_vector.ve.integral / ((R_e + alt) * cosf(lat));
	const float alt_dot = -g_state_vector.vd.integral;

	g_state_vector.lat.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.lat.prev_diff_val + lat_dot);
	g_state_vector.lat.prev_diff_val = lat_dot;
	g_state_vector.lon.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.lon.prev_diff_val + lon_dot);
	g_state_vector.lon.prev_diff_val = lon_dot;
	g_state_vector.alt.integral += BNO085_SAMPLE_TIME * 1E-3 * 0.5f * (g_state_vector.alt.prev_diff_val + alt_dot);

	if (xQueueSend(g_bno085_queue_handle, &g_state_vector, portMAX_DELAY) != pdPASS)
	{
		ESP_LOGE(TAG, "Error sending to queue");
	}
	// else
	//{
	//	ESP_LOGI(TAG, "Sent to queue");
	// }
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
	ESP_RETURN_ON_FALSE(g_bno085_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");

	*queue = g_bno085_queue_handle;

	return ESP_OK;
}

static void bno085_task(void *pvParamenters)
{
	ESP_LOGI(TAG, "Creating task");

	g_bno085_queue_handle = xQueueCreate(4, sizeof(state_vector_t));

	Wire.begin();

	while (g_IMU.begin(BNO085_ADDR, Wire, BNO085_INT, BNO085_RST) == false)
	{
		ESP_LOGE(TAG, "BNO085 not detected at default I2C address.");
		vTaskDelay(pdMS_TO_TICKS(500));
	}

	ESP_ERROR_CHECK(bno085_config_reports());

	init_integrator(g_state_vector);

	// Iniatilize the state vector
	g_state_vector.alt.integral = INIT_ALT;
	g_state_vector.lat.integral = INIT_LAT;
	g_state_vector.lon.integral = INIT_LON;

	// Setting up the timer
	ESP_LOGI(TAG, "Setting up timer");
	esp_timer_create_args_t timer_args = {
		.callback = &position_update_loop,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "position_update_loop"};

	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &g_state_loop_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(g_state_loop_timer, BNO085_SAMPLE_TIME * 1E3));

	for (;;)
	{
		if (g_IMU.wasReset())
		{
			ESP_LOGE(TAG, "Sensor was reset");
			ESP_LOGW(TAG, "Stopping timer");
			ESP_ERROR_CHECK(esp_timer_stop(g_state_loop_timer));

			bno085_set_state(BNO085_RESET);

			ESP_ERROR_CHECK(bno085_config_reports());

			if (!esp_timer_is_active(g_state_loop_timer))
			{
				ESP_LOGW(TAG, "Timer is not active. Starting it");
				ESP_ERROR_CHECK(esp_timer_start_periodic(g_state_loop_timer, BNO085_SAMPLE_TIME * 1E3));
			}
		}
		if (g_IMU.getSensorEvent() == true)
		{

			// if (g_IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER)
			//{
			//	g_imu_data.accx = g_IMU.getAccelX();
			//	g_imu_data.accy = g_IMU.getAccelY();
			//	g_imu_data.accz = g_IMU.getAccelZ();
			//	byte accel_acc = g_IMU.getAccelAccuracy();

			//	//printf("%f,%f,%f,%d\r\n", g_imu_data.accx, g_imu_data.accy, g_imu_data.accz, accel_acc);
			//}
			if (g_IMU.getSensorEventID() == SENSOR_REPORTID_LINEAR_ACCELERATION)
			{
				g_imu_data.accx = g_IMU.getLinAccelX();
				g_imu_data.accy = g_IMU.getLinAccelY();
				g_imu_data.accz = g_IMU.getLinAccelZ();
				byte accel_acc = g_IMU.getAccelAccuracy();

				//printf("%f,%f,%f,%d\r\n", g_imu_data.accx, g_imu_data.accy, g_imu_data.accz, accel_acc);
			}
			else if (g_IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
			{
				g_state_vector.roll = g_IMU.getRoll();
				g_state_vector.pitch = g_IMU.getPitch();
				g_state_vector.yaw = g_IMU.getYaw();
				// g_state_vector.roll = MAP_ANGLE(g_IMU.getRoll());
				// g_state_vector.pitch = MAP_ANGLE(g_IMU.getPitch());
				// g_state_vector.yaw = MAP_ANGLE(g_IMU.getYaw());
				// printf("/*rot,x,%f,y,%f,z,%f,calib,%d*/\r\n", g_state_vector.roll, g_state_vector.pitch, g_state_vector.yaw, 0);
				// printf("/*rot,x,%f,y,%f,z,%f,calib,%d*/\r\n", g_state_vector.roll * RAD_TO_DEG, g_state_vector.pitch * RAD_TO_DEG, g_state_vector.yaw * RAD_TO_DEG, 0);
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