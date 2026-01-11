#pragma once
#include <Arduino.h>
#include "datatypes.h"
#include "config.h"

bool addTaskToQueue(task_t *task);
int getQueueSize(void);
void clearQueue(void);
bool queueContainsTask(task_t *task);
bool rescheduleTask(task_t *task, updateFreq_e freq);
task_t *getFirstInQueue();
task_t *getNextInQueue();
// main loop
void schedulerLoop(void);

void enableAllTasks();
void disableAllTasks();
void enableTask(task_t *task, bool enabled);
void resetStatistics(task_t *task);
void resetAllStatistics();