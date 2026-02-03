#include <Arduino.h>

#include "sensors/baro.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/batteryVoltage.h"

#include "scheduler/scheduler.h"

#include "radio/radioInterface.h"
#include "scheduler/loopTasks.h"
#include "logging/logger.h"

#include "Mahony.h"
#include "StateEstimation/KalmanFilter.h"

MahonyAHRS AHRS;
SixStateLKF kFilter;

void setup(void)
{
  initLogger(micros());

  pinMode(GPS_LED, OUTPUT);
  pinMode(IMU_LED, OUTPUT);
  pinMode(BARO_LED, OUTPUT);
  pinMode(EXTRA_LED, OUTPUT);
  pinMode(SERIAL_LED, OUTPUT);

  // USB Serial
  Serial.begin(115200);
  logToBuffer("Started Serial", micros());
  if (CrashReport)
  {
    Serial.print(CrashReport);
  }

  // init serial for radio
  RADIO_UART.begin(115200);
  logToBuffer("Started Serial to radio", micros());

  // IMU and Baro
  INTERNAL_I2C_BUS.begin();
  INTERNAL_I2C_BUS.setClock(400000UL);
  logToBuffer("Started internal I2C bus", micros());

  // GPS
  EXTERNAL_I2C_BUS.begin();
  EXTERNAL_I2C_BUS.setClock(400000UL);
  logToBuffer("Started external I2C bus", micros());

  bool GPSStatus = initGPS();
  logToBuffer(GPSStatus ? "GPS Init" : "GPS Init Failed", micros());

  analogWrite(GPS_LED, 128);
  while (!calculateInitialGPSCoords())
  {
  }
  digitalWrite(GPS_LED, HIGH);

  bool baroStatus = initBaro();
  logToBuffer(baroStatus ? "Baro Init" : "Baro Init Failed", micros());
  digitalWrite(BARO_LED, HIGH);

  bool IMUStatus = initBMI088();
  logToBuffer(IMUStatus ? "IMU Init" : "IMU Init Failed", micros());
  digitalWrite(IMU_LED, HIGH);

  initVoltageSensor();
  logToBuffer("Voltage Sens Init", micros());

  bool radioStatus = initRadio();
  logToBuffer(radioStatus ? "Radio Init" : "Radio Init Failed", micros());
  clearQueue();
  logToBuffer("Queue Cleared", micros());

  initScheduler();
  logToBuffer("Initialized Scheduler", micros());

  AHRS.setMode(MahonyMode::CALIBRATING);
  const int calibSamples = 200;
  for (int i = 0; i < calibSamples; i++)
  {
    // Assume ~100Hz for the delay(10) loop
    floatVector_3 accelOutput = {0,0,0};
    readAccelData(&accelOutput);
    Vector<3> accel = Vector<3>(accelOutput.x, accelOutput.y, accelOutput.z);

    floatVector_3 gyroOutput = {0,0,0};
    readGyroData(&gyroOutput);
    Vector<3> gyro = Vector<3>(gyroOutput.x, gyroOutput.y, gyroOutput.z);
    AHRS.update(accel, gyro, 0.01);
    delay(10);
  }
  AHRS.finalizeCalibration();
  initializeRocketState();

  logToBuffer("====Starting Scheduler Loop====", micros());
  writeToDisk(micros());
}

void loop(void)
{
  schedulerLoop();

}