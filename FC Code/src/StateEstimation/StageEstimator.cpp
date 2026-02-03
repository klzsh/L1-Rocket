#include "StageEstimator.h"

static rocketStages_e globalState = UNINITIALIZED;
static uint32_t boostStartTime = 0;
static uint32_t coastStartTime = 0;
static uint32_t landingStartTime = 0;
static bool boostAccelDetected = false;
static bool coastAccelDetected = false;

static bool landed = false;

float abs(float x){
  return x < 0 ? -x : x;
}

rocketStages_e getCurrentState()
{
  return globalState;
}

inline uint32_t getTimestampSeconds(uint32_t timestamp){
  return timestamp / 1'000'000;
}

bool setRocketState(rocketStages_e state){
  globalState = state;
  return globalState == state;
}
void calculateState(uint32_t timestamp, kalmanState_t stateOutput, floatVector_3 accelReading){
  rocketStages_e currentState = globalState;
  switch(currentState){
    case UNINITIALIZED: 

      break;
    case PAD:
      if(accelReading.z > BOOST_ACCEL_THRESHOLD){
        if(!boostAccelDetected){
          boostStartTime = timestamp;
          boostAccelDetected = true;
        } else if(timestamp - boostStartTime > LIFTOFF_DURATION){
          currentState = BOOST;
          //! LOG BOOST TIME/ HEIGHT
        }
      }
      break;
    case BOOST:
      if(accelReading.z <= BURNOUT_ACCEL_THRESHOLD){
        if(!coastAccelDetected){
          coastStartTime = timestamp;
          coastAccelDetected = true;
        } else if(timestamp - coastStartTime > BURNOUT_DURATION){
          currentState = COAST;
          //! LOG BURNOUT TIME/ HEIGHT
        }
      }
      break;
    case COAST:
      if(abs(stateOutput.velocity.z) < APOGEE_VELOCITY_THRESHOLD){
        currentState = APOGEE;
        //! LOG APOGEE HEIGHT AND TIME
      }
      break;
    case APOGEE:
      if(stateOutput.velocity.z < -2.0){
        currentState = DESCENT;
        //! LOG THAT THE ROCKET IS DESCENDING. 
      }
      break;
    case DESCENT:
      if(abs(stateOutput.velocity.z) < LANDING_VELOCITY_THRESHOLD){
        if(!landed){
          landingStartTime = timestamp;
          landed = true;
        } else if(timestamp - landingStartTime > LANDING_DURATION){
          currentState = LANDED;
          //! LOG LANDING TIME
        }
      }
      break;
    case LANDED:
      break;
  }
  if(currentState != globalState){
    setRocketState(currentState);
  }
}
