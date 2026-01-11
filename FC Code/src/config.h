#pragma once
#define LOG_TO_SERIAL
#define LOG_DATA_TO_SERIAL
// #define NO_DATA_LOGGING
/*
  * SENSOR CONFIGS
*/
#define GYRO_ADDRESS_I2C 0x68
#define ACCEL_ADDRESS_I2C 0x18
#define BARO_ADDRESS_I2C 0x77
#define GPS_ADDRESS_I2C 0x42

#define INTERNAL_I2C_BUS Wire
#define EXTERNAL_I2C_BUS Wire2
#define RADIO_UART Serial1

/*
  * STATUS LED CONFIGS
*/
#define SERIAL_LED 2
#define EXTRA_LED 3
#define IMU_LED 4
#define BARO_LED 5
#define GPS_LED 6

#define BATTERY_VOLTAGE_PIN A0
#define BATTERY_RESISTOR_1 100'000
#define BATTERY_RESISTOR_2 78'700
#define REFERENCE_VOLTAGE 3.3

/*
  * LOGGING CONFIGS
*/
// bytes
#define LOG_FILE_SIZE 10'000
// sync these with the logger
#define DATA_HEADERS "Time,Baro Altitude,Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z,GPS Lat,GPS Lon,GPS Vel X,GPS Vel Y,GPS Vel Z,State X,State Y,State Z,State Vel X,State Vel Y,State Vel Z,State Rot X,State Rot Y,State Rot Z,Rocket State,Battery Voltage"
// bytes
#define LOG_MSG_MAX_LEN 64

// number of entries
#define RING_BUFFER_SIZE 128

/*
  * SCHEDULER CONFIGS
*/

#define NUM_TASKS 11

/*
  * STATE ESTIMATION CONFIGS
*/

// number of satellites before a fix is considered "good"
#define NUM_SATELLITES 4
// state detection params. These should be obtained from simulation data
#define BOOST_ACCEL_THRESHOLD 27 // in m/s^2
#define LIFTOFF_DURATION 100 // ms
#define BOOST_DURATION 2200 // ms 
#define BURNOUT_ACCEL_THRESHOLD 18 // m/s^2
#define BURNOUT_DURATION 200 // ms
#define APOGEE_VELOCITY_THRESHOLD 2.0 // m/s
#define LANDING_VELOCITY_THRESHOLD 1.5 // m/s
#define LANDING_DURATION 3000 // ms
#define DESCENT_RATE_MIN 2.0 // m/s
#define DESCENT_RATE_MAX 10.0 // m/s

/*
  These defines are here because I did not account for IMU direction when placing the BMI088.
  So I have to translate the IMU reference to the body reference orientation
*/
// using ENU orientation

#define BODY_ACCEL_REFERENCE_X accel.getAccelY_mss()
#define BODY_ACCEL_REFERENCE_Y accel.getAccelZ_mss()
#define BODY_ACCEL_REFERENCE_Z accel.getAccelX_mss()

#define BODY_GYRO_REFERENCE_X gyro.getGyroY_rads()
#define BODY_GYRO_REFERENCE_Y gyro.getGyroZ_rads()
#define BODY_GYRO_REFERENCE_Z gyro.getGyroX_rads()



/*
  Orientation filter config
*/

#define ORIENTATION_KP 0
#define ORIENTATION_KI 0
#define ORIENTATION_KD 0



/*
  Kalman filter configuration. 
  These are the only matrices and settings you need to change related to the kalman filter.
  You will also have to update the kalman filter setMeasurement() and setControl() call sites to reflect the change in control and measurement size
*/

// X matrix
#define STATE_SIZE 6
// Z matrix (GPS)
#define MEASUREMENT_SIZE 3
// U matrix (Accel)
#define CONTROL_SIZE 3

#define PROCESS_VARIANCE 2.1f
// See TRT/SRAD-Avionics/Teensy-Based-Avionics for these matrices
#define F_MATRIX_COEFS(dt) 1.0f, 0, 0, dt, 0, 0, \
                       0, 1.0f, 0, 0, dt, 0, \
                       0, 0, 1.0f, 0, 0, dt, \
                       0, 0, 0, 1.0f, 0, 0,           \
                       0, 0, 0, 0, 1.0f, 0,           \
                       0, 0, 0, 0, 0, 1.0f

#define G_MATRIX_COEFS(dt) 0.5f * dt *dt, 0, 0, \
                       0, 0.5f * dt *dt, 0, \
                       0, 0, 0.5f * dt *dt, \
                       dt, 0, 0,                    \
                       0, dt, 0,                    \
                       0, 0, dt

#define H_MATRIX_COEFS 1.0f, 0, 0, 0, 0, 0, \
                       0, 1.0f, 0, 0, 0, 0, \
                       0, 0, 1.0f, 0, 0, 0

#define R_MATRIX_COEFS 1.0f, 0, 0, \
                       0, 1.0f, 0, \
                       0, 0, 0.5f

#define Q_MATRIX_COEFS 0.1f, 0, 0, 0, 0, 0, \
                       0, 0.1f, 0, 0, 0, 0, \
                       0, 0, 0.1f, 0, 0, 0, \
                       0, 0, 0, 0.1f, 0, 0, \
                       0, 0, 0, 0, 0.1f, 0, \
                       0, 0, 0, 0, 0, 0.1f
