#ifndef GEO2NED_H
#define GEO2NED_H

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
    bool is_origin_set = false;
    float mat_ecef2ned[3][3];
} geo2ned_handler;

typedef geo2ned_handler *geo2ned_handler_t;


esp_err_t geo2ned(float lat, float lon, float alt, gps_coords_t &coord);
void geo2ned_set_origin(geo2ned_handler_t handler, gps_coords_t origin);

#endif