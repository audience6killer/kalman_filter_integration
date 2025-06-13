#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "ekf.h"

class Nav_EKF : public ekf
{
public:
    Nav_EKF();
    virtual ~Nav_EKF();
    virtual void Init();

    dspm::Mat h_gps_only;
    dspm::Mat h_odometry_only;
    dspm::Mat h_full_measure;

    // X_corrected is the the difference between the INS and the X vector
    // L_ins - delta_L
    dspm::Mat X_corrected;

    virtual dspm::Mat StateXdot(dspm::Mat &x, float *u);
    virtual void LinearizeFG(dspm::Mat &x, float *u);
    virtual void Process(float *u, float dt);

public:
    void UpdateNominalSystem(float lat, float lon, float h, float v_n, float v_e, float v_d, float f_n, float f_e, float f_d);
    // void UpdateNominalSystem(dspm::Mat state);
    void PrintXState(void);

private:
    float nominal_sys[9]; // The Strapdown Navigation System
};

#endif