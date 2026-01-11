#pragma once
#include <MS5611.h>
#include "config.h"


bool initBaro(void);
float readBaroAltitude(void);