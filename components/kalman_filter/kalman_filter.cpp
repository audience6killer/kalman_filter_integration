
#include "kalman_filter.h"

#define OMEGA_E 0.01f
#define R_e 0.0f
#define R_n 0.0f
#define G_f 0.0f

#define SIGMA_ANG  0.0037f // or 0.0012
#define SIGMA_FX   0.0004316788375805062f
#define SIGMA_FY   0.00036860956390441724f
#define SIGMA_FZ   0.00044968152477711375f
#define SIGMA_POS   SIGMA_FX + SIGMA_FY + SIGMA_FZ

Nav_EKF::Nav_EKF() : ekf(15, 15),
                     h_full_measure(7, 15)
{

}

Nav_EKF::~Nav_EKF()
{

}

void Nav_EKF::Init()
{
    // G will be constant
    this->G.Copy(dspm::Mat::eye(this->NUMX), 0, 0);

    this->Q.Copy(0.1f * dspm::Mat::eye(15), 0, 0);

    // All the measurement matrix will be linear
    this->h_full_measure *= 0.0f;
    this->h_full_measure(6, 2) = -1.0f;
    this->h_full_measure.Copy(dspm::Mat::eye(3), 3, 3);
    this->h_full_measure.Copy(dspm::Mat::eye(3), 0, 6);

    // Process covariance
    this->Q(0, 0) = SIGMA_ANG;
    this->Q(1, 1) = SIGMA_ANG;
    this->Q(2, 2) = SIGMA_ANG;
    this->Q(3, 3) = SIGMA_FX;
    this->Q(4, 4) = SIGMA_FY;
    this->Q(5, 5) = SIGMA_FZ;
    this->Q(6, 6) = SIGMA_POS;
    this->Q(7, 7) = SIGMA_POS;
    this->Q(8, 8) = SIGMA_POS;
    this->Q.Copy(0.001f * dspm::Mat::eye(7), 9, 9);

    // Initial state
    this->X *= 0.0f;
    this->P.Copy(0.001f * dspm::Mat::eye(this->NUMX), 0, 0);
    
}

void Nav_EKF::LinearizeFG(dspm::Mat &x, float *u)
{
    float lat = 0.0f;
    float lon = 0.0f;
    float h = 0.0f;
    float v_n = 0.0f;
    float v_e = 0.0f;
    float v_d = 0.0f;
    float f_n = 0.0f;
    float f_e = 0.0f;
    float f_d = 0.0f;
    float beta = 0.0f;

    // Initializate the process matrix with zeros
    this->F *= 0.0f;

    // First row
    F(0, 1) = -OMEGA_E * sinf(lat) - (v_e * tan(lat)) / (R_e + h);
    F(0, 2) = v_n / (R_n + h);
    F(0, 3) = 0.0f;
    F(0, 4) = 1 / (R_e + h);
    F(0, 6) = -v_e / powf(R_e + h, 2) - OMEGA_E * sinf(lat);
    F(0, 12) = -1.0f;
    // Second row
    F(1, 0) = OMEGA_E * sinf(lat) + (v_e * tan(lat)) / (R_e + h);
    F(1, 2) = v_e / (R_e + h) + OMEGA_E * cosf(lat);
    F(1, 3) = -1 / (R_n + h);
    F(1, 8) = v_n / powf((R_e + h), 2);
    F(1, 13) = -1.0f;
    // Third row
    F(2, 0) = -v_n / (R_n + h);
    F(2, 1) = -v_e / (R_e + h) - OMEGA_E * cosf(lat);
    F(2, 4) = -tan(lat) / (R_e + h);
    F(2, 6) = -OMEGA_E * cosf(lat) - v_e / (powf(cosf(lat), 2) * (R_e + h));
    F(2, 8) = (v_e * tan(lat)) / powf((R_e + h), 2);
    F(2, 14) = -1.0f;
    // Forth row
    F(3, 1) = -f_d;
    F(3, 2) = f_e;
    F(3, 3) = v_d / (R_n + h);
    F(3, 4) = -2 * sinf(lat) * (OMEGA_E + v_e / (cosf(lat) * (R_e + h)));
    F(3, 5) = v_n / (R_n + h);
    F(3, 6) = -2 * OMEGA_E * v_e * cosf(lat) - powf(v_e, 2) / (powf(cosf(lat), 2) * (R_e + h));
    F(3, 8) = (powf(v_e, 2) * tanf(lat)) / powf(R_e + h, 2) - (v_d * v_n) / powf(R_n + h, 2);
    F(3, 9) = 1.0f;
    // Fifth row
    F(4, 0) = f_d;
    F(4, 2) = -f_n;
    F(4, 3) = (v_e + tan(lat)) / (R_e + h) + 2 * OMEGA_E * sinf(lat);
    F(4, 4) = (v_d + v_n * tan(lat)) / (R_e + h);
    F(4, 5) = v_e / (R_e + h) + 2 * OMEGA_E * cosf(lat);
    F(4, 6) = 2 * OMEGA_E * (v_n * cosf(lat) - v_d * sinf(lat)) + (v_e * v_n) / (powf(cosf(lat), 2) * (R_e + h));
    F(4, 8) = -(v_e * (v_d + v_n * tan(lat))) / powf(R_e + h, 2);
    F(4, 10) = 1.0f;
    // Sixth row
    F(5, 0) = -f_e;
    F(5, 1) = f_n;
    F(5, 3) = -(2 * v_n) / (R_n + h);
    F(5, 4) = -(2 * v_e) / (R_e + h) - 2 * OMEGA_E * cosf(lat);
    F(5, 6) = 2 * OMEGA_E * v_e * sinf(lat);
    F(5, 8) = powf(v_e, 2) / powf(R_e + h, 2) - (2 * G_f) / (R_n + h) + powf(v_n, 2) / powf(R_n + h, 2);
    F(5, 11) = 1.0f;
    // Seventh row
    F(6, 3) = 1 / (R_n + h);
    F(6, 8) = -v_n / pow(R_n + h, 2);

    // Eight row
    F(7, 4) = 1 / (cosf(lat) * (R_e + h));
    F(7, 6) = (v_e * sinf(lat)) / (powf(cosf(lat), 2) * (R_e + h));
    F(7, 8) = -(v_e * cosf(lat)) / (R_e + h);

    // Ninth row
    F(8, 5) = -1.0f;

    F(12, 12) = -beta;
    F(13, 13) = -beta;
    F(14, 14) = -beta;
}