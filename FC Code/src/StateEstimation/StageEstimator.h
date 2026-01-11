#pragma once
#include "StateEstimation/KalmanFilter.h"
#include "datatypes.h"

rocketStages_e getCurrentState();
bool setRocketState(rocketStages_e state);
void calculateState(uint32_t timestamp, kalmanState_t stateOutput, floatVector_3 accelReading);

