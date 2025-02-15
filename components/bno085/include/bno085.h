#ifndef BNO085_H
#define BNO085_H
#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float integral;
        float prev_val;
    } integrator_t;

    typedef struct
    {
        integrator_t lat;
        integrator_t lon;
        integrator_t alt;
        integrator_t north_p; // North
        integrator_t east_p;  // East
        integrator_t down_p;  // Down
        float roll;
        float pitch;
        float yaw;
    } state_vector_t;

    typedef enum
    {
        BNO085_OK = 0,
        BNO085_STOPPED,
        BNO085_RESET,

    } bno085_state_t;

    esp_err_t bno085_get_queue_handle(QueueHandle_t *queue);

    bno085_state_t bno085_get_state(void);

    void bno085_start_task(void);

#ifdef __cplusplus
}
#endif
#endif