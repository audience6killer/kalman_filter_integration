extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "common_types.h"
}

#include "Arduino.h"
#include "Wire.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

#include "bno085_task_common.h"
#include "bno085.h"

// #define MAP_ANGLE(angle) (angle < 0 ? 2 * PI + angle : angle)

typedef enum
{
	IMU_ORI_BIT = 0,
	IMU_ACCEL_BIT,
} bno85_data_source_e;

static const char TAG[] = "imu";

static BNO08x g_IMU;
static QueueHandle_t g_imu_data_queue_handle = NULL;
static QueueHandle_t g_imu_cmd_queue_handle = NULL;
static imu_state_e g_imu_state = IMU_STOPPED;
static state_vector_t g_state_vector;
static EventGroupHandle_t g_event_group;
static EventGroupHandle_t g_error_group;
static byte g_reading_flag = 0;
static bool g_origin_is_set = false;

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

void imu_init_integrator(state_vector_t &pose)
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

void imu_set_state(imu_state_e state)
{
	xEventGroupSetBits(g_event_group, state);
	g_imu_state = state;
}

void imu_update_pose(void)
{
	// Specific force to local geographic reference, angles must be in radians?
	static int64_t prev_time = 0;
	int64_t c_time = esp_timer_get_time();
	float delta_time = (c_time - prev_time) / 1E6; // Convert microseconds to seconds
	prev_time = c_time;

	// printf("Delta time: %f\n", delta_time);

	float fn = g_imu_data.accx - g_state_vector.yaw * g_imu_data.accy + g_state_vector.pitch * g_imu_data.accz;
	float fe = g_state_vector.yaw * g_imu_data.accx + g_imu_data.accy - g_state_vector.roll * g_imu_data.accz;
	float fd = -g_state_vector.pitch * g_imu_data.accx + g_state_vector.roll * g_imu_data.accy + g_imu_data.accz;

	g_state_vector.fn = fn;
	g_state_vector.fe = fe;
	g_state_vector.fd = fd;

	// printf("/*%f,%f,%f*/\n", g_state_vector.fn, g_state_vector.fe, g_state_vector.fd);

	const float vn = g_state_vector.vn.integral;
	const float ve = g_state_vector.ve.integral;
	const float vd = g_state_vector.vd.integral;
	const float lat = g_state_vector.lat.integral;
	const float alt = g_state_vector.alt.integral;
	const float R_0_alt = R_0 + alt;

	float vn_dot = fn - 2 * OMEGA_E * ve * sinf(lat) + (vn * vd - powf(ve, 2) * tanf(lat)) / (R_0_alt);
	float ve_dot = fe + 2 * OMEGA_E * (vn * sinf(lat) + ve * cosf(lat)) + (ve) / (R_0_alt) * (vd + vn * tanf(lat));
	float vd_dot = fd - 2 * OMEGA_E * ve * cosf(lat) - (powf(ve, 2) + powf(vn, 2)) / (R_0_alt);

	// printf("/*%f,%f,%f*/\n", ve_dot, vn_dot, vd_dot);

	// Speed integration
	g_state_vector.vn.integral += delta_time * 0.5f * (g_state_vector.vn.prev_diff_val + vn_dot);
	g_state_vector.vn.prev_diff_val = vn_dot;

	g_state_vector.ve.integral += delta_time * 0.5f * (g_state_vector.ve.prev_diff_val + ve_dot);
	g_state_vector.ve.prev_diff_val = ve_dot;

	g_state_vector.vd.integral += delta_time * 0.5f * (g_state_vector.vd.prev_diff_val + vn_dot);
	g_state_vector.vd.prev_diff_val = vd_dot;

	// printf("/*%f,%f,%f*/\n", g_state_vector.ve.integral, g_state_vector.vn.integral, g_state_vector.vd.integral);

	// Position integration
	const double lat_dot = g_state_vector.vn.integral / (R_n + alt);
	const double lon_dot = g_state_vector.ve.integral / ((R_e + alt) * cosf(lat));
	const double alt_dot = -g_state_vector.vd.integral;

	// printf("/*%.10lf,%.10lf,%.10lf*/\n", lat_dot, lon_dot, alt_dot);

	g_state_vector.lat.integral += delta_time * 0.5 * (g_state_vector.lat.prev_diff_val + lat_dot);
	g_state_vector.lat.prev_diff_val = lat_dot;
	g_state_vector.lon.integral += delta_time * 0.5 * (g_state_vector.lon.prev_diff_val + lon_dot);
	g_state_vector.lon.prev_diff_val = lon_dot;
	g_state_vector.alt.integral += delta_time * 0.5 * (g_state_vector.alt.prev_diff_val + alt_dot);
	g_state_vector.alt.prev_diff_val = alt_dot;

	// printf("/*%.10lf,%.10lf,%.10lf*/\n", g_state_vector.lat.integral, g_state_vector.lon.integral, g_state_vector.alt.integral);

	if (xQueueSend(g_imu_data_queue_handle, &g_state_vector, pdMS_TO_TICKS(100)) != pdPASS)
	{
		ESP_LOGE(TAG, "Error sending to queue");
	}
}

void imu_config_reports(void)
{
	while (g_IMU.enableLinearAccelerometer() != true)
	{
		ESP_LOGE(TAG, "Linear Accel could not be enabled. Retrying...");
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	ESP_LOGI(TAG, "Linear Accel enabled");
	//ESP_LOGI(TAG, "Output in form x, y, z, in m/s2");

	while (g_IMU.enableRotationVector() != true)
	{
		ESP_LOGE(TAG, "Rotation vector could not be enabled. Retrying...");
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	ESP_LOGI(TAG, "Rotation vector enabled");
	// ESP_LOGI(TAG, "Output in form x, y, z, in rad");

	imu_set_state(IMU_OK);
}

esp_err_t imu_get_cmd_queue_handle(QueueHandle_t *queue)
{
	ESP_RETURN_ON_FALSE(g_imu_cmd_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "CMD queue handle is NULL");

	*queue = g_imu_cmd_queue_handle;

	return ESP_OK;
}

esp_err_t imu_get_data_queue_handle(QueueHandle_t *queue)
{
	ESP_RETURN_ON_FALSE(g_imu_data_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Data queue handle is NULL");

	*queue = g_imu_data_queue_handle;

	return ESP_OK;
}

void imu_data_loop(void)
{
	/* Check if IMU was reset */
	if (g_IMU.wasReset())
	{
		ESP_LOGE(TAG, "Sensor was reset");

		imu_set_state(IMU_RESET);

		imu_config_reports();

		imu_set_state(IMU_STARTED);
	}

	/* Get readings */
	if (g_IMU.getSensorEvent() == true)
	{
		if (g_IMU.getSensorEventID() == SENSOR_REPORTID_LINEAR_ACCELERATION)
		{
			g_imu_data.accx = g_IMU.getLinAccelX();
			g_imu_data.accy = g_IMU.getLinAccelY();
			g_imu_data.accz = -g_IMU.getLinAccelZ();
	        //byte accel_acc = g_IMU.getAccelAccuracy();

			g_reading_flag = g_reading_flag | 1 << IMU_ACCEL_BIT;

			//printf("%f,%f,%f,%d\r\n", g_imu_data.accx, g_imu_data.accy, g_imu_data.accz, accel_acc);
		}
		else if (g_IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
		{
			g_state_vector.roll = g_IMU.getRoll();
			g_state_vector.pitch = g_IMU.getPitch();
			g_state_vector.yaw = g_IMU.getYaw();

			g_reading_flag = g_reading_flag | 1 << IMU_ORI_BIT;
			// printf("/*rot,x,%f,y,%f,z,%f,calib,%d*/\r\n", RAD_TO_DEG * g_state_vector.roll, RAD_TO_DEG * g_state_vector.pitch, RAD_TO_DEG * g_state_vector.yaw, 0);
		}

		// Check if both readings have been received
		if (g_reading_flag == 0b00000011)
		{
			imu_update_pose();
			g_reading_flag = 0;
		}
	}
}

/* Event handlers */
esp_err_t imu_start_event_handler(void)
{
	if (g_imu_state == IMU_STARTED)
	{
		ESP_LOGW(TAG, "IMU process already started!");
		return ESP_OK;
	}

	if (!g_origin_is_set)
	{
		ESP_LOGE(TAG, "Cannot start process without initializing the origin!");
		xEventGroupSetBits(g_event_group, IMU_ERROR);
		xEventGroupSetBits(g_error_group, IMU_ERR_ORIGIN_NOT_SET);
		return ESP_FAIL;
	}

	imu_set_state(IMU_STARTED);

	return ESP_OK;
}

esp_err_t imu_stop_event_handler(void)
{
	if (g_imu_state == IMU_STOPPED)
	{
		ESP_LOGW(TAG, "IMU process already stopped!");
		return ESP_OK;
	}
	else if (g_imu_state != IMU_STARTED)
	{
		ESP_LOGW(TAG, "IMU process is not started!");
		return ESP_OK;
	}

	imu_set_state(IMU_STOPPED);

	return ESP_OK;
}

esp_err_t imu_set_origin_event_handler(gps_coords_t *origin)
{
	ESP_RETURN_ON_FALSE(origin != NULL, ESP_ERR_INVALID_STATE, TAG, "Origin point is NULL");

	if (g_imu_state == IMU_STARTED)
	{
		ESP_LOGE(TAG, "Setting new origin while process already started!");
		return ESP_FAIL;
	}

	g_state_vector.alt.integral = origin->alt;
	g_state_vector.lat.integral = origin->lat;
	g_state_vector.lon.integral = origin->lon;

	ESP_LOGI(TAG, "New origin set: Lat = %f, Lon = %f, Alt = %f",
			 g_state_vector.lat.integral, g_state_vector.lon.integral, g_state_vector.alt.integral);
	g_origin_is_set = true;

	return ESP_OK;
}

void imu_cmd_handler(void)
{
	static imu_cmd_t cmd;

	if (xQueueReceive(g_imu_cmd_queue_handle, &cmd, pdMS_TO_TICKS(10)) == pdPASS)
	{
		switch (cmd.cmd)
		{
		case IMU_CMD_START:
			ESP_LOGI(TAG, "Event: Start process");
			if (imu_start_event_handler() != ESP_OK)
			{
				ESP_LOGE(TAG, "Start event failed");
			}
			break;

		case IMU_CMD_STOP:
			ESP_LOGI(TAG, "Event: Stop process");
			imu_stop_event_handler();
			break;

		default:
			ESP_LOGW(TAG, "Event: Unknown event");
			break;
		}
	}
}

static void imu_task(void *pvParamenters)
{
	ESP_LOGI(TAG, "Initializing task");

	/* Create queues */
	g_imu_data_queue_handle = xQueueCreate(5, sizeof(state_vector_t));
	g_imu_cmd_queue_handle = xQueueCreate(5, sizeof(imu_cmd_t));

	/* Create Event Groups */
	g_event_group = xEventGroupCreate();
	g_error_group = xEventGroupCreate();

	/* Begin I2C communication */
	Wire.begin();

	while (!g_IMU.begin(IMU_ADDR, Wire, IMU_INT, IMU_RST))
	{
		ESP_LOGE(TAG, "imu not detected at default I2C address. Retrying..");
		xEventGroupSetBits(g_event_group, IMU_ERROR);
		xEventGroupSetBits(g_error_group, IMU_ERR_I2C_ERROR);
		vTaskDelay(pdMS_TO_TICKS(500));
	}

	imu_config_reports();

	imu_init_integrator(g_state_vector);

	/* Notify parent end of initialization */
	imu_set_state(IMU_OK);

	for (;;)
	{
		if (g_imu_state == IMU_STARTED)
		{
			imu_data_loop();
		}

		imu_cmd_handler();

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void imu_start_task(void)
{
	ESP_LOGI(TAG, "Starting task");

	xTaskCreatePinnedToCore(&imu_task, "imu", IMU_STACK_SIZE,
							NULL, IMU_TASK_PRIORITY, NULL, IMU_CORE_ID);
}