#pragma once
#include <Arduino.h>
#include "config.h"
bool initRadio();
bool transmitData(char *data, uint32_t timestamp);
char *recordSerialData();