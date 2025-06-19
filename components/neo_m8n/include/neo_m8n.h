#ifndef NEO_M8N_H
#define NEO_M8N_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        double lat;
        double lon;
        double alt;
        double hdop;
    } gps_coords_t;

    typedef enum 
    {
        NEO_M8N_CMD_STOP = 0,
        NEO_M8N_CMD_START,
    } neo_m8n_cmd_e;

    typedef enum
    {
        NEO_M8N_STATE_STOPPED = 0,
        NEO_M8N_STATE_STARTED,
    } neo_m8n_state_e;


    typedef struct 
    {
        neo_m8n_cmd_e cmd;
    } neo_m8n_cmd_t;

    esp_err_t neo_m8n_get_data_queue(QueueHandle_t *handle);

    esp_err_t neo_m8n_get_cmd_queue(QueueHandle_t *handle);

    void neo_m8n_task_start(void);

#ifdef __cplusplus
}
#endif

#endif