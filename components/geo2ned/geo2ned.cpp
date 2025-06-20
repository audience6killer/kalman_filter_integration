/**
 * @file geo2ned.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "freertos/FreeRTOS.h"
#include "esp_check.h"
#include "math.h"

#include "common_types.h"

#include "geo2ned.h"

#define ECEF_A      6378137.0f
#define ECEF_B      6356752.3142f
#define ECEF_E_2    0.00669438f

static const char TAG[] = "geo2ned";

void geo2ecef(geo2ned_handler_t handler, gps_coords_t *coord, ned_ecef_coords_t *ecef_coord)
{
    double N0 = ECEF_A / sqrt(1 - ECEF_E_2 * pow(sin(coord->lat), 2));
    ecef_coord->x = (N0 + coord->alt) * cos(coord->lat) * cos(coord->lon);
    ecef_coord->y = (N0 + coord->alt) * cos(coord->lat) * sin(coord->lon);
    ecef_coord->z = (N0 * (1 - ECEF_E_2) + coord->alt) * sin(coord->lat);
}

void geo2ned_set_origin(geo2ned_handler_t handler, gps_coords_t origin)
{
    if (handler->is_origin_set)
        ESP_LOGW(TAG, "Origin will be overwritten!");

    
    geo2ecef(handler, &origin, &handler->origin);

    handler->mat_ecef2ned[0][0] = -sin(origin.lat) * cos(origin.lon);
    handler->mat_ecef2ned[0][1] = -sin(origin.lat) * sin(origin.lat);
    handler->mat_ecef2ned[0][2] = cos(origin.lat);
    handler->mat_ecef2ned[1][0] = -sin(origin.lon);
    handler->mat_ecef2ned[1][1] = cos(origin.lon);
    handler->mat_ecef2ned[1][2] = 0.0f;
    handler->mat_ecef2ned[2][0] = -cos(origin.lat) * cos(origin.lon);
    handler->mat_ecef2ned[2][1] = -cos(origin.lat) * sin(origin.lon);
    handler->mat_ecef2ned[2][2] = -sin(origin.lat);

    handler->is_origin_set = true;
}

esp_err_t geo2ned(geo2ned_handler_t handler, gps_coords_t coord, ned_ecef_coords_t *ned_coord)
{
    ESP_RETURN_ON_FALSE(handler->is_origin_set, ESP_ERR_INVALID_STATE, TAG, "Origin is not set yet!");

    /* The first step is convert to ECEF */
    double N0 = ECEF_A / sqrt(1 - ECEF_E_2 * pow(sin(coord.lat), 2));
    ned_coord->x = (N0 + coord.alt) * cos(coord.lat) * cos(coord.lon);
    ned_coord->y = (N0 + coord.alt) * cos(coord.lat) * sin(coord.lon);
    ned_coord->z = (N0 * (1 - ECEF_E_2) + coord.alt) * sin(coord.lat);

    float x_diff = ned_coord->x - handler->origin.x;
    float y_diff = ned_coord->y - handler->origin.y;
    float z_diff = ned_coord->z - handler->origin.z;

    ned_coord->n = handler->mat_ecef2ned[0][0] * x_diff + handler->mat_ecef2ned[0][1] * y_diff + handler->mat_ecef2ned[0][2] * z_diff;
    ned_coord->e = handler->mat_ecef2ned[1][0] * x_diff + handler->mat_ecef2ned[1][1] * y_diff + handler->mat_ecef2ned[1][2] * z_diff;
    ned_coord->d = handler->mat_ecef2ned[2][0] * x_diff + handler->mat_ecef2ned[2][1] * y_diff + handler->mat_ecef2ned[2][2] * z_diff;

    return ESP_OK;
}