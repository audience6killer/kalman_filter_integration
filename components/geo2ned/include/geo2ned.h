#ifndef GEO2NED_H
#define GEO2NED_H

#include "freertos/FreeRTOS.h"
#include "common_types.h"

typedef struct
{
    double x;
    double y;
    double z;
    double n;
    double e;
    double d;
} ned_ecef_coords_t;

typedef struct
{
    ned_ecef_coords_t origin;
    bool is_origin_set;
    float mat_ecef2ned[3][3];
} geo2ned_handler;

typedef geo2ned_handler *geo2ned_handler_t;

/**
 * @brief Allocates memory for a geo2ned_handler_t object.
 * 
 * @return geo2ned_handler_t Pointer to the allocated geo2ned_handler_t, or NULL on failure.
 */
geo2ned_handler_t geo2ned_handler_create(void);

/**
 * @brief 
 * 
 * @param handler 
 * @param origin 
 */
void geo2ned_set_origin(geo2ned_handler_t handler, gps_coords_t origin);

/**
 * @brief 
 * 
 * @param handler 
 * @param coord 
 * @param ned_coord 
 * @return esp_err_t 
 */
esp_err_t geo2ned(geo2ned_handler_t handler, gps_coords_t coord, ned_ecef_coords_t *ned_coord);



#endif