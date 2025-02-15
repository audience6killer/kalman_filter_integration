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

    virtual dspm::Mat StateXdot(dspm::Mat &x, float *u);
    virtual void LinearizeFG(dspm::Mat &x, float *u);

};

#endif