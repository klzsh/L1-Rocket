#include "loopTasks.h"
// this file is where all the main state estimation and tasks will happen

loggedData_t rocketState;

task_t tasks[NUM_TASKS + 1] = {
    // sys task
    [0] = {
        .name = "System",
        .freq = FREQ_50HZ,
        .taskPriority = PRIORITY_LOW,
        .taskToRun = task_system},
    [1] = {
        .name = "Read Barometer",
        .freq = FREQ_100HZ,
        .taskPriority = PRIORITY_HIGH,
        .taskToRun = task_readBaro},
    [2] = {
        .name = "Read Accel",
        .freq = FREQ_100HZ,
        .taskPriority = PRIORITY_HIGH,
        .taskToRun = task_readAccel},
    [3] = {
        .name = "Read Gyro",
        .freq = FREQ_100HZ,
        .taskPriority = PRIORITY_HIGH,
        .taskToRun = task_readGyro},
    [4] = {
        .name = "Read GPS",
        .freq = FREQ_10HZ,
        .taskPriority = PRIORITY_HIGH,
        .taskToRun = task_readGPS},
    [5] = {
        .name = "Read Battery",
        .freq = FREQ_2HZ,
        .taskPriority = PRIORITY_LOW,
        .taskToRun = task_readBatteryVoltage},
    [6] = {
        .name = "Update Kalman",
        .freq = FREQ_50HZ,
        .taskPriority = PRIORITY_HIGH,
        .taskToRun = task_updateKalmanFilter},
    [7] = {
        .name = "Update State",
        .freq = FREQ_20HZ,
        .taskPriority = PRIORITY_HIGH,
        .taskToRun = task_updateState},
    [8] = {
        .name = "Log Data",
        .freq = FREQ_20HZ,
        .taskPriority = PRIORITY_MEDIUM,
        .taskToRun = task_LogData},
    [9] = {
        .name = "Write Buffer to Disk",
        .freq = FREQ_2HZ,
        .taskPriority = PRIORITY_LOW,
        .taskToRun = task_WriteBufferToDisk},
    [10] = {
        .name = "Transmit Radio",
        .freq = FREQ_10HZ,
        .taskPriority = PRIORITY_LOW,
        .taskToRun = task_sendRadioData},
    [11] = {0}
};
void initializeRocketState(){
    rocketState.accelReading = {0, 0, 0};
    rocketState.altitude = 0;
    rocketState.batteryVoltage = 0;
    rocketState.gpsOutput = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };
    rocketState.gyroReading = {0, 0, 0};
    rocketState.kalmanFilterOutput = {.position = {0,0,0}, .velocity = {0,0,0}};
    rocketState.stage = UNINITIALIZED;
}
void initScheduler(){
    for (int i = 0; i < NUM_TASKS + 1; i++){
        addTaskToQueue(&tasks[i]);
    }
    resetAllStatistics();
    enableAllTasks();
}

void task_readBaro(uint32_t time)
{
    float previousAltitude = rocketState.altitude;
    rocketState.altitude = readBaroAltitude();
    if (previousAltitude - rocketState.altitude <= 0.002 && rocketState.stage >= PAD)
    {
        // logToBuffer("Could not read Baro Data", time, LOGLEVEL_INFO);
    }
}
void task_readAccel(uint32_t time)
{
    if (!readAccelData(&rocketState.accelReading))
    {
        logToBuffer("Could not read Accel Data", time);
    }
}
void task_readGyro(uint32_t time)
{
    if (!readGyroData(&rocketState.gyroReading))
    {
        logToBuffer("Could not read Gyro Data", time);
    }
}
void task_readGPS(uint32_t time)
{
    getGPSData(&rocketState.gpsOutput);
    // if (!getGPSData(&rocketState.gpsOutput))
    // {
    //     logToBuffer("Could not read GPS data", time, LOGLEVEL_INFO);
    // } else {
    //     logToBuffer("read GPS data", time, LOGLEVEL_INFO);

    // }
}
void task_readBatteryVoltage(uint32_t time)
{
    float previousReading = rocketState.batteryVoltage;
    rocketState.batteryVoltage = readBatteryVoltage();

    if (previousReading - rocketState.batteryVoltage <= 0.002)
    {
        // logToBuffer("Could not read Battery Voltage", time, LOGLEVEL_INFO);
    }
}

void task_updateState(uint32_t time)
{
    calculateState(time, rocketState.kalmanFilterOutput, rocketState.accelReading);
}

void task_updateKalmanFilter(uint32_t time)
{
    float controlData[CONTROL_SIZE] = {rocketState.accelReading.x, rocketState.accelReading.y, rocketState.accelReading.z};
    float measurementData[MEASUREMENT_SIZE] = {
        rocketState.gpsOutput.displacement.x,
        rocketState.gpsOutput.displacement.y,
        rocketState.altitude};
    setKalmanControl(controlData);
    setKalmanMeasurement(measurementData);
    iterateFilter(time);
    rocketState.kalmanFilterOutput = getKalmanState();
}

void task_WriteBufferToDisk(uint32_t time)
{
#ifndef LOG_TO_SERIAL
    writeToDisk(time);
#endif
}
void task_LogData(uint32_t time)
{
#ifndef NO_DATA_LOGGING
    logDataStruct(time, &rocketState);
#endif
}
void task_sendRadioData(uint32_t time)
{
}
void task_system(uint32_t time)
{
    // task overruns
    // sensor health
    // battery health
    //
}
