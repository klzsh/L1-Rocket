#pragma once
#include <BMI088.h>
#include "datatypes.h"
#include "config.h"

bool initBMI088(void);


bool readAccelData(floatVector_3 *data);
bool readGyroData(floatVector_3 *data);

