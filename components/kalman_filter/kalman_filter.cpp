
extern "C"
{
#include "common_types.h"
}

#include "kalman_filter.h"

#define OMEGA_E 0.01f
#ifndef R_n
#define R_n 6378137.0f
#define R_e 6356752.3142f
#define R_0 6371000.0f
#endif
#define G_f 0.0f

#define SIGMA_ANG 0.0037f // or 0.0012
#define SIGMA_FX 0.0004316788375805062f
#define SIGMA_FY 0.00036860956390441724f
#define SIGMA_FZ 0.00044968152477711375f
#define SIGMA_POS SIGMA_FX + SIGMA_FY + SIGMA_FZ
#define GPS_STD_BASE 2.5f

Nav_EKF::Nav_EKF() : ekf(15, 15),
                     H_GPS(3, 15),
                     H_ODOMETRY(4, 15),
                     H_FULL(7, 15)
{
    /* Initialize measurement matrix H */
    this->H_FULL *= 0.0f;
    this->H_FULL(6, 2) = -1.0f;
    this->H_FULL.Copy(dspm::Mat::eye(3), 3, 3);
    this->H_FULL.Copy(dspm::Mat::eye(3), 0, 6);

    this->H_GPS *= 0.0f;
    this->H_GPS.Copy(dspm::Mat::eye(3), 0, 6);

    this->H_ODOMETRY *= 0.0f;
    this->H_ODOMETRY(3, 2) = -1.0f;
    this->H_ODOMETRY.Copy(dspm::Mat::eye(3), 0, 3);

    this->geo2ned_conv = geo2ned_handler_create();

    this->X_corrected *= 0.0f;
}

Nav_EKF::~Nav_EKF()
{
}

void Nav_EKF::Init()
{
    // Initialize the nominal system
    for (size_t i = 0; i < 9; i++)
    {
        this->nominal_sys[i] = 0.0;
    }

    // G will be constant
    // this->G.Copy(dspm::Mat::eye(this->NUMX), 0, 0);
    this->G *= 0.0f;

    this->Q.Copy(0.1f * dspm::Mat::eye(15), 0, 0);

    // Process covariance
    this->Q(0, 0) = SIGMA_ANG;
    this->Q(1, 1) = SIGMA_ANG;
    this->Q(2, 2) = SIGMA_ANG;
    this->Q(3, 3) = SIGMA_FX * 10E3;
    this->Q(4, 4) = SIGMA_FY * 10E3;
    this->Q(5, 5) = SIGMA_FZ * 10E3;
    this->Q(6, 6) = SIGMA_POS * 10E4;
    this->Q(7, 7) = SIGMA_POS * 10E4;
    this->Q(8, 8) = SIGMA_POS * 10E4;
    this->Q.Copy(0.01f * dspm::Mat::eye(7), 9, 9);
    // Initial state
    this->X *= 0.0f;
    this->P.Copy(0.001f * dspm::Mat::eye(this->NUMX), 0, 0);
}

void Nav_EKF::Process(float *u, float dt)
{
    if(dt == 0)
    {
        printf("dt is zero!!!!!\n");
        return;
    }

    // ekf::Process(u, dt);
    this->LinearizeFG(this->X, u);
    this->RungeKutta(this->X, u, dt);
    this->CovariancePrediction(dt);

    // printf("%.7f, %.7f,%.7f,%.7f,%.7f,%.7f\n", this->X(6, 0), this->X(7, 0), this->X(8, 0), this->X(3, 0), this->X(4, 0), this->X(5, 0));

    this->X_corrected(0, 0) = nominal_sys[0] - this->X(6, 0);
    this->X_corrected(1, 0) = nominal_sys[1] - this->X(7, 0);
    this->X_corrected(2, 0) = nominal_sys[2] - this->X(8, 0);
    this->X_corrected(3, 0) = nominal_sys[3] - this->X(3, 0);
    this->X_corrected(4, 0) = nominal_sys[4] - this->X(4, 0);
    this->X_corrected(5, 0) = nominal_sys[5] - this->X(5, 0);
}

void Nav_EKF::LinearizeFG(dspm::Mat &x, float *u)
{
    /* The state vector is the SNIS error vector */
    double lat = nominal_sys[0];
    double lon = nominal_sys[1];
    double h = nominal_sys[2];
    double v_n = nominal_sys[3];
    double v_e = nominal_sys[4];
    double v_d = nominal_sys[5];
    double f_n = nominal_sys[6];
    double f_e = nominal_sys[7];
    double f_d = nominal_sys[8];

    // Initializate the process matrix with zeros
    this->F *= 0.0f;

    // First row
    F(0, 1) = -OMEGA_E * sin(lat) - (v_e * tan(lat)) / (R_e + h);
    F(0, 2) = v_n / (R_n + h);
    F(0, 3) = 0.0f;
    F(0, 4) = 1 / (R_e + h);
    F(0, 6) = -v_e / powf(R_e + h, 2) - OMEGA_E * sin(lat);
    F(0, 12) = -1.0f;
    // Second row
    F(1, 0) = OMEGA_E * sin(lat) + (v_e * tan(lat)) / (R_e + h);
    F(1, 2) = v_e / (R_e + h) + OMEGA_E * cos(lat);
    F(1, 3) = -1 / (R_n + h);
    F(1, 8) = v_n / powf((R_e + h), 2);
    F(1, 13) = -1.0f;
    // Third row
    F(2, 0) = -v_n / (R_n + h);
    F(2, 1) = -v_e / (R_e + h) - OMEGA_E * cos(lat);
    F(2, 4) = -tan(lat) / (R_e + h);
    F(2, 6) = -OMEGA_E * cos(lat) - v_e / (powf(cos(lat), 2) * (R_e + h));
    F(2, 8) = (v_e * tan(lat)) / powf((R_e + h), 2);
    F(2, 14) = -1.0f;
    // Forth row
    F(3, 1) = -f_d;
    F(3, 2) = f_e;
    F(3, 3) = v_d / (R_n + h);
    F(3, 4) = -2 * sin(lat) * (OMEGA_E + v_e / (cos(lat) * (R_e + h)));
    F(3, 5) = v_n / (R_n + h);
    F(3, 6) = -2 * OMEGA_E * v_e * cos(lat) - powf(v_e, 2) / (powf(cos(lat), 2) * (R_e + h));
    F(3, 8) = (powf(v_e, 2) * tanf(lat)) / powf(R_e + h, 2) - (v_d * v_n) / powf(R_n + h, 2);
    F(3, 9) = 1.0f;
    // Fifth row
    F(4, 0) = f_d;
    F(4, 2) = -f_n;
    F(4, 3) = (v_e + tan(lat)) / (R_e + h) + 2 * OMEGA_E * sin(lat);
    F(4, 4) = (v_d + v_n * tan(lat)) / (R_e + h);
    F(4, 5) = v_e / (R_e + h) + 2 * OMEGA_E * cos(lat);
    F(4, 6) = 2 * OMEGA_E * (v_n * cos(lat) - v_d * sin(lat)) + (v_e * v_n) / (powf(cos(lat), 2) * (R_e + h));
    F(4, 8) = -(v_e * (v_d + v_n * tan(lat))) / powf(R_e + h, 2);
    F(4, 10) = 1.0f;
    // Sixth row
    F(5, 0) = -f_e;
    F(5, 1) = f_n;
    F(5, 3) = -(2 * v_n) / (R_n + h);
    F(5, 4) = -(2 * v_e) / (R_e + h) - 2 * OMEGA_E * cos(lat);
    F(5, 6) = 2 * OMEGA_E * v_e * sin(lat);
    F(5, 8) = powf(v_e, 2) / powf(R_e + h, 2) - (2 * G_f) / (R_n + h) + powf(v_n, 2) / powf(R_n + h, 2);
    F(5, 11) = 1.0f;
    // Seventh row
    F(6, 3) = 1 / (R_n + h);
    F(6, 8) = -v_n / pow(R_n + h, 2);

    // Eight row
    F(7, 4) = 1 / (cos(lat) * (R_e + h));
    F(7, 6) = (v_e * sin(lat)) / (powf(cos(lat), 2) * (R_e + h));
    F(7, 8) = -(v_e * cos(lat)) / (R_e + h);

    // Ninth row
    F(8, 5) = -1.0f;

    F(12, 12) = -this->X(9, 1);
    F(13, 13) = -this->X(10, 1);
    F(14, 14) = -this->X(11, 1);
}

dspm::Mat Nav_EKF::StateXdot(dspm::Mat &x, float *u)
{
    dspm::Mat Xdot = (this->F * x);

    return Xdot;
}

void Nav_EKF::UpdateNominalSystem(double lat, double lon, double h, float v_n, float v_e, float v_d, float f_n, float f_e, float f_d, float heading)
{
    this->nominal_sys[0] = lat;
    this->nominal_sys[1] = lon;
    this->nominal_sys[2] = h;
    this->nominal_sys[3] = v_n;
    this->nominal_sys[4] = v_e;
    this->nominal_sys[5] = v_d;
    this->nominal_sys[6] = f_n;
    this->nominal_sys[7] = f_e;
    this->nominal_sys[8] = f_d;
    this->nominal_sys[9] = heading;
}

void Nav_EKF::Update(float *measured, float *R, Nav_EKF::MeasureSource source)
{
    if (source == Nav_EKF::MeasureSource::FULL)
    {
        float err_measure[7];

        for (size_t i = 0; i < 6; i++)
        {
            err_measure[i] = nominal_sys[i] - measured[i];
        }
        err_measure[6] = nominal_sys[9] - measured[6];

        dspm::Mat expected_X = H_FULL * X;
        float expected[7];

        for (size_t i = 0; i < 7; i++)
        {
            expected[i] = expected_X(i, 0);
        }

        ekf::Update(this->H_FULL, err_measure, expected, R);
    }
    else if (source == Nav_EKF::MeasureSource::GPS)
    {
        float err_measure[3];

        for (size_t i = 0; i < 3; i++)
        {
            err_measure[i] = nominal_sys[i] - measured[i];
        }

        dspm::Mat expected_X = H_GPS * X;
        float expected[3];

        for (size_t i = 0; i < 3; i++)
        {
            expected[i] = expected_X(i, 0);
        }

        // Scale R vector
        float sigma = R[0] * GPS_STD_BASE; // 1σ error in meters

        R[0] = pow((sigma / (R_e + measured[2])) * (sigma / (R_n + measured[2])), 2) * 10E3; // for delta phi_g
        R[1] = pow((sigma / ((R_n + measured[2]) * cos(measured[0]))), 2) * 10E3;            // for delta lambda_g
        R[2] = sigma * 10E-3;

        // printf("R[lat]=%.10f,R[lon]=%.10f,R[alt]=%.10f\n", R[0], R[1], R[2]);

        ekf::Update(this->H_GPS, err_measure, expected, R);
    }
    else if (source == Nav_EKF::MeasureSource::ODOMETRY)
    {
        float err_measure[4];

        for (size_t i = 0; i < 3; i++)
        {
            err_measure[i] = nominal_sys[i] - measured[i];
        }
        err_measure[6] = nominal_sys[9] - measured[3];

        dspm::Mat expected_X = H_ODOMETRY * X;
        float expected[4];

        for (size_t i = 0; i < 4; i++)
        {
            expected[i] = expected_X(i, 0);
        }
        ekf::Update(this->H_ODOMETRY, err_measure, expected, R);
    }
    else
    {
        return;
    }
}

void Nav_EKF::SetOrigin(gps_coords_t origin)
{
    geo2ned_set_origin(this->geo2ned_conv, origin);
}

void Nav_EKF::PrintXState(void)
{
    // printf("[\n\tdelta_a = %.4f\n\tdelta_b = %.4f\n\tdelta_y = %.4f\n\tdelta_v_n = %.4f\n\tdelta_v_e = %.4f\n\tdelta_v_d = %.4f\n\tdelta_lat = %.4f\n\tdelta_lon = %.4f\n\tdelta_h = %.4f\n\tB_n = %.4f\n\tB_e = %.4f\n\tB_d = %.4f\n\tD_n = %.4f\n\tD_e = %.4f\n\t\D_d = %.4f\n]\n", this->X(0, 0), this->X(1, 0),this->X(2, 0),this->X(3, 0),this->X(4, 0),this->X(5, 0),this->X(6, 0),this->X(7, 0),this->X(8, 0),this->X(9, 0),this->X(10, 0),this->X(11, 0),this->X(12, 0),this->X(13, 0),this->X(14, 0));

    // printf("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", nominal_sys[0], nominal_sys[1],nominal_sys[2], nominal_sys[3],nominal_sys[4],nominal_sys[5],nominal_sys[6],nominal_sys[7],nominal_sys[8]);
    // printf("/*%.10lf,%.10lf,%.10lf,%.10lf,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f*/\n", X_corrected(0, 0), nominal_sys[0], X_corrected(1, 0), nominal_sys[1], X_corrected(2, 0), nominal_sys[2], X_corrected(3, 0), nominal_sys[3], X_corrected(4, 0), nominal_sys[4], X_corrected(5, 0), nominal_sys[5]);
    // gps_coords_t corrected = {
    //     .lat = X_corrected(0, 0),
    //     .lon = X_corrected(1, 0),
    //     .alt = X_corrected(2, 0),
    // };
    gps_coords_t nominal_geo = {
        .lat = nominal_sys[0],
        .lon = nominal_sys[1],
        .alt = nominal_sys[2],
    };
    ned_ecef_coords_t nominal_ned;
    geo2ned(this->geo2ned_conv, nominal_geo, &nominal_ned);

    gps_coords_t corrected = {
        .lat = X_corrected(0, 0),
        .lon = X_corrected(1, 0),
        .alt = X_corrected(2, 0),
    };
    ned_ecef_coords_t ned_corr;
    geo2ned(this->geo2ned_conv, corrected, &ned_corr);

    printf("/*%.10lf,%.10lf,%.10lf,%.10lf,%.10lf,%.10lf*/\n", nominal_ned.n, ned_corr.n, nominal_ned.e, ned_corr.e, nominal_ned.d, ned_corr.d);
}
