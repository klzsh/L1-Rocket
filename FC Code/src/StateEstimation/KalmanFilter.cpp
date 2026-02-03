/*
  genuinely some of the worst code I have written.
*/

#include "KalmanFilter.h"

SixStateLKF::SixStateLKF() : LinearKalmanFilter(MEASUREMENT_SIZE, CONTROL_SIZE, STATE_SIZE){}


Matrix SixStateLKF::getF(double dt) {
    double *data = new double[STATE_SIZE*STATE_SIZE]{
      F_MATRIX_COEFS(dt)
    };
    return Matrix(STATE_SIZE, STATE_SIZE, data);
}

Matrix SixStateLKF::getG(double dt) {
    double *data = new double[STATE_SIZE*CONTROL_SIZE]{
        G_MATRIX_COEFS(dt)
    };
    return Matrix(STATE_SIZE, CONTROL_SIZE, data);
}

Matrix SixStateLKF::getH() {
    double *data = new double[MEASUREMENT_SIZE*STATE_SIZE]{
        H_MATRIX_COEFS
    };
    return Matrix(MEASUREMENT_SIZE, STATE_SIZE, data);
}

Matrix SixStateLKF::getR() {
    // Measurement noise covariance - how much we trust sensors
    // Lower values = trust sensors more (good for simulation with low/no noise)
    // Higher values = trust sensors less (good for real hardware with noisy sensors)

    // For simulation: use smaller values since sim sensors are typically accurate
    double r_gps_xy = 0.25;  // GPS horizontal: ~0.5m std dev (good GPS)
    double r_alt = 0.1;      // Barometric altitude: ~0.3m std dev (good baro)

    double *data = new double[MEASUREMENT_SIZE*MEASUREMENT_SIZE]{
        r_gps_xy, 0, 0,
        0, r_gps_xy, 0,
        0, 0, r_alt
    };
    return Matrix(MEASUREMENT_SIZE, MEASUREMENT_SIZE, data);
}

Matrix SixStateLKF::getQ(double dt) {
    // Process noise covariance - accounts for model uncertainty
    // Higher values = less trust in the constant-acceleration model
    // For rockets: acceleration changes rapidly (thrust curves, drag, gravity turn)

    // Position process noise: small (model is good for position prediction)
    double q_pos = 0.01;

    // Velocity process noise: moderate (velocity can change due to drag, thrust variations)
    double q_vel = 1.0;

    double *data = new double[STATE_SIZE*STATE_SIZE]{
        q_pos, 0, 0, 0, 0, 0,
        0, q_pos, 0, 0, 0, 0,
        0, 0, q_pos, 0, 0, 0,
        0, 0, 0, q_vel, 0, 0,
        0, 0, 0, 0, q_vel, 0,
        0, 0, 0, 0, 0, q_vel
    };
    return Matrix(STATE_SIZE, STATE_SIZE, data);
}