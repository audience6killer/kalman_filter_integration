#ifndef BNO085_H
#define BNO085_H
#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float integral;
        float prev_value;
    } integrator_t;

    typedef struct
    {
        integrator_t north;   // North
        integrator_t east;    // East
        integrator_t down;    // Down
        integrator_t north_p; // North
        integrator_t east_p;  // East
        integrator_t down_p;  // Down
        float roll;
        float pitch;
        float yaw;
    } state_vector_t;

    void bno085_start_task(void);

#ifdef __cplusplus
}
#endif
#endif