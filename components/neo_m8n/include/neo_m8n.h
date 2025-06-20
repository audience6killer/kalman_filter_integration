#ifndef NEO_M8N_H
#define NEO_M8N_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "common_types.h"

#ifdef __cplusplus
extern "C"
{
#endif


    typedef enum 
    {
        NEO_M8N_CMD_STOP = 0,
        NEO_M8N_CMD_START,
    } neo_m8n_cmd_e;

    typedef enum
    {
        NEO_M8N_STATE_STOPPED = BIT0,
        NEO_M8N_STATE_STARTED = BIT1,
        NEO_M8n_STATE_ERROR = BIT3,
    } neo_m8n_state_e;

    typedef enum
    {
        NEO_M8N_ERROR_NONE = BIT0,
    } neo_m8n_error_e;


    typedef struct 
    {
        neo_m8n_cmd_e cmd;
    } neo_m8n_cmd_t;

    esp_err_t neo_m8n_get_last_coord(gps_coords_t *coord);

    esp_err_t neo_m8n_get_event_group(EventGroupHandle_t *handle);

    esp_err_t neo_m8n_get_error_group(EventGroupHandle_t *handle);

    esp_err_t neo_m8n_get_data_queue(QueueHandle_t *handle);

    esp_err_t neo_m8n_get_cmd_queue(QueueHandle_t *handle);

    void neo_m8n_task_start(void);

#ifdef __cplusplus
}
#endif

#endif