#ifndef imu_H
#define imu_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "common_types.h"

// HOME
#define INIT_LAT 19.5037727355f
#define INIT_LON -99.1248474121f
#define INIT_ALT 2331.8000488f

    typedef struct
    {
        double integral;
        double prev_diff_val;
    } integrator_t;

    typedef struct
    {
        integrator_t lat; // In degrees
        integrator_t lon; // In degrees
        integrator_t alt;
        integrator_t vn; // North
        integrator_t ve; // East
        integrator_t vd; // Down
        float roll;
        float pitch;
        float yaw;
        float fn;
        float fe;
        float fd;
    } state_vector_t;

    typedef enum
    {
        IMU_OK = BIT0,
        IMU_STARTED = BIT1,
        IMU_STOPPED = BIT2,
        IMU_RESET = BIT3,
        IMU_ERROR = BIT4,
    } imu_state_e;

    typedef enum
    {
        IMU_ERR_I2C_ERROR = BIT0,
        IMU_ERR_ORIGIN_NOT_SET = BIT1,
    } imu_err_e;

    typedef enum
    {
        IMU_CMD_START = 0,
        IMU_CMD_STOP,
        IMU_CMD_SET_ORIGIN,
    } imu_cmd_e;

    typedef struct
    {
        imu_cmd_e cmd;
        gps_coords_t *origin;
    } imu_cmd_t;

    esp_err_t imu_get_data_queue_handle(QueueHandle_t *queue);

    esp_err_t imu_get_cmd_queue_handle(QueueHandle_t *queue);

    void imu_start_task(void);

#ifdef __cplusplus
}
#endif
#endif