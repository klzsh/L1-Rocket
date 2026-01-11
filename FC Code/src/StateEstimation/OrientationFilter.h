#pragma once
#include "datatypes.h"
#include "config.h"
#include <arm_math.h>
void initialize();
void calibrate(floatVector_3 *accelSamples, floatVector_3 *gyroSamples);
void update(floatVector_3 *accelSamples, floatVector_3 *gyroSamples, float dt);
floatVector_3 getEarthAcceleration(const floatVector_3 *accelSamples);