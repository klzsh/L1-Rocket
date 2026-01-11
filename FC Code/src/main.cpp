#include <Arduino.h>

#include "sensors/baro.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/batteryVoltage.h"

#include "scheduler/scheduler.h"

#include "radio/radioInterface.h"
#include "scheduler/loopTasks.h"
#include "logging/logger.h"


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
  while(!calculateInitialGPSCoords()){}
  digitalWrite(GPS_LED, HIGH);

  bool baroStatus = initBaro();
  logToBuffer(baroStatus ? "Baro Init" : "Baro Init Failed", micros());
  digitalWrite(BARO_LED, HIGH);

  bool IMUStatus = initBMI088();
  logToBuffer(IMUStatus ? "IMU Init" : "IMU Init Failed", micros());
  digitalWrite(IMU_LED, HIGH);


  initVoltageSensor();
  logToBuffer("Voltage Sens Init", micros());

  // initRadio();
  // logToBuffer("Radio Initialized", micros());
  clearQueue();
  logToBuffer("Queue Cleared", micros());

  initScheduler();
  logToBuffer("Initialized Scheduler", micros());

  initializeKF(micros());
  initializeRocketState();

  logToBuffer("====Starting Scheduler Loop====", micros());
  writeToDisk(micros());
}
GPSOutput_t output = {0};
void loop(void)
{
  schedulerLoop();
  // getGPSData(&output);
  // Serial.println(output.latitude);
  // delay(100);
}