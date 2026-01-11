#pragma once
#include "stdint.h"
#include "config.h"

typedef struct
{
  float x;
  float y;
  float z;
} floatVector_3;

typedef struct
{
  long x;
  long y;
  long z;
} longVector_3;

typedef struct
{
  float x;
  float y;
} floatVector_2;

typedef struct{
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

typedef enum {
  UNINITIALIZED = -1,
  PAD = 0,
  BOOST,
  COAST,
  APOGEE,
  DESCENT,
  LANDED
} rocketStages_e;

typedef struct
{
  floatVector_3 position;
  floatVector_3 velocity;
} kalmanState_t; // output of the kalman filter. This will be used to detect apogee as well as boost, landing, etc

typedef struct
{
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t day;
  uint8_t month;
  uint16_t year;
} GPSTime_t;

typedef struct
{
  float latitude;
  float longitude;
  float altitude;
  float heading;
  int fixQuality;
  floatVector_3 position;
  floatVector_3 velocity;
  floatVector_2 displacement;
  GPSTime_t time;

} GPSOutput_t;

typedef enum
{
  CALIBRATION,
  CORRECTION,
  GYRO_ONLY
} orientationFilterMode_e;

/* SCHEDULER DATATYPES */
typedef enum
{
  FREQ_1HZ = 1'000'000,
  FREQ_2HZ = 1'000'000/2,
  FREQ_5HZ = 1'000'000 / 5,
  FREQ_10HZ = 1'000'000 / 10,
  FREQ_20HZ = 1'000'000/20,
  FREQ_50HZ = 1'000'000 / 50,
  FREQ_100HZ = 1'000'000 / 100,
  FREQ_200HZ = 1'000'000 / 200,
  FREQ_500HZ = 1'000'000 / 500,
  FREQ_800HZ = 1'000'000 / 800,
  FREQ_1000HZ = 1'000'000 / 1000,
  FREQ_2000HZ = 1'000'000 / 2000,
  FREQ_5000HZ = 1'000'000 / 5000,
  FREQ_8000HZ = 1'000'000 / 8000
} updateFreq_e;

typedef enum
{
  PRIORITY_IDLE = 1,
  PRIORITY_LOW = 2,
  PRIORITY_MEDIUM = 5,
  PRIORITY_HIGH = 10,
  PRIORITY_REALTIME = 25,
  PRIORITY_MAX = 50

} taskPriority_e;

typedef struct
{
  // average per loop
  uint16_t averageExecutionDuration;
  // previous runtime
  uint16_t lastExecutionDuration;
  // last absolute time the task was run
  uint32_t lastRunTime;
  // longest loop time
  uint16_t longestExecutionDuration;
  // number of times the task has been scheduled
  uint32_t numRuns;
  // total time since boot
  uint32_t totalExecutionDuration;

} taskStatistics_t;

typedef struct
{
  const char *name;
  // freq to update the func at in microsecond intervals
  updateFreq_e freq;
  // normal priority
  taskPriority_e taskPriority;
  // if the func has not been run in a while, boost priority up (measured in loops deferred)
  uint16_t taskStaleness;
  // priority * staleness
  uint32_t effectivePriority;
  // task stats
  taskStatistics_t statistics;
  // the function which will run on a scheduler cycle, passing in the current time
  void (*taskToRun)(uint32_t);
  // if we should actually run the task
  bool enabled;
} task_t;

typedef struct
{
  float altitude;
  floatVector_3 accelReading;
  floatVector_3 gyroReading;
  GPSOutput_t gpsOutput;
  kalmanState_t kalmanFilterOutput;
  rocketStages_e stage;
  float batteryVoltage;
} loggedData_t;

typedef enum
{
  LOGLEVEL_NONE,
  LOGLEVEL_INFO,
  LOGLEVEL_WARN,
  LOGLEVEL_ERROR
} logLevels_e;

typedef struct
{
  uint32_t timestamp;
  logLevels_e priority;
  char message[LOG_MSG_MAX_LEN];
} logEntry_t;

typedef struct
{
  logEntry_t buffer[RING_BUFFER_SIZE];
  uint16_t head;
  uint16_t tail;
} logRingBuffer_t;