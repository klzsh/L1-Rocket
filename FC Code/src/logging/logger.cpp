/*
  Logs data to an SD Card

*/
#include "logger.h"

#define RING_NEXT(idx) (((idx) + 1) % RING_BUFFER_SIZE)
#define FLUSH_DATA_FILE true
#define FLUSH_LOG_FILE false

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)
static SdFs sd;
// periodically logged stuff
static FsFile flightDataLog;
// exceptions, errors, etc
static FsFile extraLogData;

static logRingBuffer_t ringBuffer = {0};
static uint16_t droppedLogs = 0;

bool initFS()
{
  if (!sd.begin(SdioConfig(FIFO_SDIO)))
  {
    return false;
  }
  return true;
}
bool initLogger(const uint32_t timestamp)
{
  bool fsReady = initFS();
  if (!fsReady)
  {
    return false;
  }
  char flightDataFilename[32];
  snprintf(flightDataFilename, sizeof(flightDataFilename), "%lu-LOG.csv", timestamp);
  char logFilename[32];
  snprintf(logFilename, sizeof(logFilename), "%lu-EXT.csv", timestamp);

  if (flightDataLog.exists(flightDataFilename))
  {
    flightDataLog.remove(flightDataFilename);
  }
  if (extraLogData.exists(logFilename))
  {
    extraLogData.remove(logFilename);
  }
  if (!flightDataLog.open(flightDataFilename, O_RDWR | O_CREAT))
  {
    return false;
  }
  if (!extraLogData.open(logFilename, O_RDWR | O_CREAT))
  {
    return false;
  }
  if (!flightDataLog.preAllocate(LOG_FILE_SIZE))
  {
    flightDataLog.close();
    return false;
  }
  if (!extraLogData.preAllocate(LOG_FILE_SIZE))
  {
    extraLogData.close();
    return false;
  }

  flightDataLog.seek(0);
  extraLogData.seek(0);
  flightDataLog.println(DATA_HEADERS);
  return true;
}

/* helper methods to log to buffer*/
static inline bool ringIsFull(const logRingBuffer_t *r)
{
  return RING_NEXT(r->head) == r->tail;
}

static inline bool ringIsEmpty(const logRingBuffer_t *r)
{
  return r->head == r->tail;
}

void logToBuffer(const char data[LOG_MSG_MAX_LEN], uint32_t timestamp, logLevels_e priority)
{
#ifndef LOG_TO_SERIAL
  // pointer for easier access
  logRingBuffer_t *r = &ringBuffer;
  uint16_t next = RING_NEXT(r->head);
  if (next == r->tail)
  {
    droppedLogs++;
    return;
  }
  logEntry_t *entry = &r->buffer[r->head];
  entry->timestamp = timestamp;
  strncpy(entry->message, data, LOG_MSG_MAX_LEN - 1);
  entry->message[LOG_MSG_MAX_LEN - 1] = '\0';
  entry->priority = priority;
  r->head = next;
#endif
#ifdef DEBUG
  Serial.printf("%lu: %s\n", timestamp, data);
#endif
}
// if you do not need to specify the log level
void logToBuffer(const char data[LOG_MSG_MAX_LEN], uint32_t timestamp)
{
  logToBuffer(data, timestamp, LOGLEVEL_NONE);
}
// this function should be run at a much slower frequency than the logToBuffer function since it will take a lot of time to log the data to disk
void writeToDisk(uint32_t timestamp)
{
  logRingBuffer_t *r = &ringBuffer;
  flightDataLog.printf("Data logged to disk at %lu\n", timestamp);

  while (!ringIsEmpty(r))
  {
    logEntry_t *e = &r->buffer[r->tail];
    flightDataLog.print(e->timestamp);
    flightDataLog.print("- ");
    switch (e->priority)
    {
    case LOGLEVEL_INFO:
      flightDataLog.print("INFO: ");
      break;
    case LOGLEVEL_WARN:
      flightDataLog.print("WARN: ");
      break;
    case LOGLEVEL_ERROR:
      flightDataLog.print("ERROR: ");
      break;
    case LOGLEVEL_NONE:
      // do not print a prefix
      break;
    }
    flightDataLog.println(e->message);
    r->tail = RING_NEXT(r->tail);
  }
  flushFile(FLUSH_LOG_FILE);
}

void logDataStruct(uint32_t timestamp, loggedData_t *data)
{
#ifndef LOG_DATA_TO_SERIAL
#define FLIGHT_DATA_LOG_OUTPUT flightDataLog
#else
#define FLIGHT_DATA_LOG_OUTPUT Serial
#endif

#ifdef LOG_DATA_TO_SERIAL
  Serial.println("=============");
  Serial.println(DATA_HEADERS);
#endif
  FLIGHT_DATA_LOG_OUTPUT.print(timestamp);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->altitude);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->accelReading.x);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->accelReading.y);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->accelReading.z);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->gyroReading.x);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->gyroReading.y);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->gyroReading.z);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->gpsOutput.latitude);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->gpsOutput.longitude);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->gpsOutput.velocity.x);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->gpsOutput.velocity.y);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->gpsOutput.velocity.z);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->kalmanFilterOutput.position.x);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->kalmanFilterOutput.position.y);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->kalmanFilterOutput.position.z);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->kalmanFilterOutput.velocity.x);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->kalmanFilterOutput.velocity.y);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->kalmanFilterOutput.velocity.z);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->stage);
  FLIGHT_DATA_LOG_OUTPUT.print(',');
  FLIGHT_DATA_LOG_OUTPUT.print(data->batteryVoltage);
  FLIGHT_DATA_LOG_OUTPUT.println();
  #ifdef LOG_DATA_TO_SERIAL
  Serial.println("=============");
  #endif
  #ifndef LOG_DATA_TO_SERIAL
  flushFile(FLUSH_DATA_FILE);
  #endif
}
// if true, flush logData, if not, flush extraLogData
void flushFile(bool flushLogData)
{
  if (flushLogData)
  {
    flightDataLog.flush();
  }
  else
  {
    extraLogData.flush();
  }
}