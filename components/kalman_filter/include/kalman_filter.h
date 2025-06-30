#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "common_types.h"
#include "geo2ned.h"

#include "ekf.h"

class Nav_EKF : public ekf
{
public:
    Nav_EKF();
    virtual ~Nav_EKF();
    virtual void Init();

    // X_corrected is the the difference between the INS and the X vector
    // L_ins - delta_L
    dspm::Mat X_corrected;

    enum class MeasureSource {FULL = 0, GPS, ODOMETRY};

    virtual dspm::Mat StateXdot(dspm::Mat &x, float *u);
    virtual void LinearizeFG(dspm::Mat &x, float *u);
    virtual void Process(float *u, float dt);

public:
    void UpdateNominalSystem(double lat, double lon, double h, float v_n, float v_e, float v_d, float f_n, float f_e, float f_d, float heading);
    void Update(float *measured, float *R, Nav_EKF::MeasureSource source);
    void SetOrigin(gps_coords_t origin);
    void PrintXState(void);

private:
    double nominal_sys[10]; // The Strapdown Navigation System
    dspm::Mat H_GPS;
    dspm::Mat H_ODOMETRY;
    dspm::Mat H_FULL;
    geo2ned_handler_t geo2ned_conv;
};

#endif