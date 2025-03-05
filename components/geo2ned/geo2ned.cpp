#include "geo2ned.h"

#if false

typedef struct
{
    float x;
    float y;
    float z;
    float n;
    float e;
    float d;
} gps_coords_t;

static gps_coords_t g_gps_origin;

static bool g_origin_set = false;
float g_mat_ecef2ned[3][3];


void neo_m8n_geo2ecef(float lat, float lon, float alt, gps_coords_t &coord)
{
    float N0 = ECEF_A / sqrtf(1 - ECEF_E_2 * powf(sinf(lat), 2));
    coord.x = (N0 + alt) * cosf(lat) * cosf(lon);
    coord.y = (N0 + alt) * cosf(lat) * sinf(lon);
    coord.z = (N0 * (1 - ECEF_E_2) + alt) * sinf(lat);
}

esp_err_t neo_m8n_geo2ned(float lat, float lon, float alt, gps_coords_t &coord)
{
    ESP_RETURN_ON_FALSE(g_origin_set, ESP_ERR_INVALID_STATE, TAG, "Origin is not set yet!");
    neo_m8n_geo2ecef(lat, lon, alt, coord);

    float x_diff = coord.x - g_gps_origin.x;
    float y_diff = coord.y - g_gps_origin.y;
    float z_diff = coord.z - g_gps_origin.z;

    coord.n = g_mat_ecef2ned[0][0] * x_diff + g_mat_ecef2ned[0][1] * y_diff + g_mat_ecef2ned[0][2] * z_diff;
    coord.e = g_mat_ecef2ned[1][0] * x_diff + g_mat_ecef2ned[1][1] * y_diff + g_mat_ecef2ned[1][2] * z_diff;
    coord.d = g_mat_ecef2ned[2][0] * x_diff + g_mat_ecef2ned[2][1] * y_diff + g_mat_ecef2ned[2][2] * z_diff;

    ESP_RETURN_ON_FALSE(xQueueSend(g_gps_queue_handle, &coord, portMAX_DELAY) == pdPASS, ESP_ERR_INVALID_RESPONSE, TAG, "Error sending to queue");

    return ESP_OK;
}

void neo_m8n_set_origin(float lat, float lon, float alt)
{
    if (g_origin_set)
        ESP_LOGW(TAG, "Origin will be overwritten!");

    neo_m8n_geo2ecef(lat, lon, alt, g_gps_origin);

    g_mat_ecef2ned[0][0] = -sinf(lat) * cosf(lon);
    g_mat_ecef2ned[0][1] = -sinf(lat) * sinf(lat);
    g_mat_ecef2ned[0][2] = cosf(lat);
    g_mat_ecef2ned[1][0] = -sinf(lon);
    g_mat_ecef2ned[1][1] = cosf(lon);
    g_mat_ecef2ned[1][2] = 0.0f;
    g_mat_ecef2ned[2][0] = -cosf(lat) * cosf(lon);
    g_mat_ecef2ned[2][1] = -cosf(lat) * sinf(lon);
    g_mat_ecef2ned[2][2] = -sinf(lat);

    g_origin_set = true;
}

#endif