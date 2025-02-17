#ifndef BNO085_H
#define BNO085_H
#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float integral;
        float prev_diff_val;
    } integrator_t;

    typedef struct
    {
        integrator_t lat; // In degrees
        integrator_t lon; // In degrees
        integrator_t alt;
        integrator_t vn; // North
        integrator_t ve;  // East
        integrator_t vd;  // Down
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