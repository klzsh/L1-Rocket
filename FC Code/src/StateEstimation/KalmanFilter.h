#include "datatypes.h"
#include "config.h"
#include <arm_math.h>
void initializeKF(uint32_t timestamp);
void iterateFilter(uint32_t timestamp);
kalmanState_t getKalmanState();
void setKalmanMeasurement(float32_t *data);
void setKalmanControl(float32_t *data);