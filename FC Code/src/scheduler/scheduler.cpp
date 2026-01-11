#include "scheduler.h"
#define MAX(a, b) a > b ? a : b

/*
  Base scheduler for future FC code iterations
  Most of this code is taken from INAV
*/

task_t *mainLoopTasks[NUM_TASKS+1];
int numTasks = 0;
int taskIndex = 0;

void enableAllTasks()
{
  for (task_t *task = getFirstInQueue(); task != NULL; task = getNextInQueue())
  {
    task->enabled = true;
  }
}
void disableAllTasks()
{
  for (task_t *task = getFirstInQueue(); task != NULL; task = getNextInQueue())
  {
    task->enabled = false;
  }
}
void enableTask(task_t *task, bool enabled)
{
  task->enabled = enabled;
}

void resetStatistics(task_t *task){
  task->statistics = {
    .averageExecutionDuration = 0,
    .lastExecutionDuration = 0,
    .lastRunTime = 0,
    .longestExecutionDuration = 0,
    .numRuns = 0,
    .totalExecutionDuration = 0
};
}
void resetAllStatistics(){
  for (task_t *task = getFirstInQueue(); task != NULL; task = getNextInQueue())
  {
    resetStatistics(task);
  }
}
bool addTaskToQueue(task_t *task)
{
  if (numTasks >= NUM_TASKS || queueContainsTask(task))
  {
    return false;
  }
  mainLoopTasks[numTasks] = task;
  numTasks++;
  return true;
}

int getQueueSize(void)
{
  return numTasks;
}
void clearQueue(void)
{
  memset(mainLoopTasks, 0, sizeof(task_t));
  numTasks = 0;
  taskIndex = 0;
}
bool queueContainsTask(task_t *taskToCheck)
{
  for (task_t *task = getFirstInQueue(); task != NULL; task = getNextInQueue())
  {
    if (taskToCheck == task)
    {
      return true;
    }
  }
  return false;
}
// change the frequency a task is scheduled
bool rescheduleTask(task_t *task, updateFreq_e freq)
{
  if (task->freq == freq)
  {
    return false;
  }
  task->freq = freq;
  return true;
}

task_t *getFirstInQueue()
{
  taskIndex = 0;
  return mainLoopTasks[taskIndex];
}
task_t *getNextInQueue()
{
  taskIndex++;
  return mainLoopTasks[taskIndex];
}

// main loop, taken mostly from INAV/Cleanflight
void schedulerLoop(void)
{


  uint32_t currentTime = micros();
  bool overdueRealtimeTask = false;
  task_t *currentTask = NULL;
  uint32_t currentTaskEffectivePriority = 0;



  // for loop iterating through tasks array
  for (task_t *taskToCheck = getFirstInQueue(); taskToCheck != NULL; taskToCheck = getNextInQueue())
  {
    // skip over disabled tasks
    if (!taskToCheck->enabled)
    {
      continue;
    }
    
   // first check if a realtime task is due to run
   if(taskToCheck->taskPriority == PRIORITY_REALTIME){
    if((currentTime - taskToCheck->statistics.lastRunTime) > taskToCheck->freq){
      currentTaskEffectivePriority = taskToCheck->effectivePriority;
      currentTask = taskToCheck;
      overdueRealtimeTask = true;
    }
   } else {
     taskToCheck->taskStaleness = (currentTime - taskToCheck->statistics.lastRunTime) / taskToCheck->freq;
     taskToCheck->effectivePriority = taskToCheck->taskPriority * taskToCheck->taskStaleness;
   }
   if(!overdueRealtimeTask && taskToCheck->effectivePriority > currentTaskEffectivePriority){
     currentTaskEffectivePriority = taskToCheck->effectivePriority;
     currentTask = taskToCheck;
   }
  }


  
  if (currentTask)
  {
    uint32_t startTime = micros();
    currentTask->taskToRun(currentTime);
    uint32_t executionTime = micros() - startTime;
    // statistics
    currentTask->statistics.numRuns++;
    currentTask->statistics.lastExecutionDuration = executionTime;
    currentTask->statistics.lastRunTime = currentTime;
    currentTask->statistics.totalExecutionDuration += startTime;
    currentTask->statistics.longestExecutionDuration = MAX(currentTask->statistics.longestExecutionDuration, executionTime);
    currentTask->statistics.averageExecutionDuration = (currentTask->statistics.totalExecutionDuration) / currentTask->statistics.numRuns;
    currentTask->taskStaleness = 0;
  }
}