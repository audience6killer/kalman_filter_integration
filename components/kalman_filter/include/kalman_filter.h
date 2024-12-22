#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "ekf.h"

class Nav_EKF : public ekf
{
public:
    Nav_EKF();
    virtual ~Nav_EKF();
    virtual void Init();

    virtual dspm::Mat StateXdot(dspm::Mat &x, float *u);
    virtual void LinearizeFG(dspm::Mat &x, float *u);

    /**
     * Non-linearized system matrice F, where xDot[n] = F*x[n] + G*u + W
    */
    dspm::Mat F_n;
}

#endif