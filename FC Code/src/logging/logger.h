#pragma once
#include <SdFat.h>
#include "datatypes.h"
#include "config.h"

bool initFS();
bool initLogger(uint32_t timestamp);
void logToBuffer(const char data[LOG_MSG_MAX_LEN], uint32_t timestamp, logLevels_e priority);
void logToBuffer(const char data[LOG_MSG_MAX_LEN], uint32_t timestamp);
void writeToDisk(uint32_t timestamp);
void logDataStruct(uint32_t timestamp, loggedData_t *data);
void flushFile(bool flushLogData);