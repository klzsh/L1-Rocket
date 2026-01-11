#pragma once
#include "math.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include "config.h"
#include "datatypes.h"

bool initGPS();
bool getGPSData(GPSOutput_t *dataObj);
bool calculateInitialGPSCoords();
GPSTime_t getGPSTime();