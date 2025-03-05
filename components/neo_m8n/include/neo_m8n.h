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

    QueueHandle_t neo_m8n_get_queue(void);
    /**
     * @brief Latitude and longitude must be in radians and alt in meters
     * 
     * @param lat 
     * @param lon 
     * @param alt 
     */
    void neo_m8n_set_origin(float lat, float lon, float alt);

    void neo_m8n_task_start(void);

#ifdef __cplusplus
}
#endif

#endif