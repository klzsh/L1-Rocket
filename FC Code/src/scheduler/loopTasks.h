#pragma once
#include "datatypes.h"
#include "config.h"

#include "sensors/baro.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/batteryVoltage.h"

#include "logging/logger.h"
#include "radio/radioInterface.h"
#include "StateEstimation/StageEstimator.h"

#include "scheduler/scheduler.h"

void initScheduler();
void initializeRocketState();
void task_readBaro(uint32_t time);
void task_readAccel(uint32_t time);
void task_readGyro(uint32_t time);
void task_readGPS(uint32_t time);
void task_updateState(uint32_t time);
void task_updateKalmanFilter(uint32_t time);
void task_WriteBufferToDisk(uint32_t time);
void task_LogData(uint32_t time);
void task_sendRadioData(uint32_t time);
void task_system(uint32_t time);
void task_readBatteryVoltage(uint32_t time);