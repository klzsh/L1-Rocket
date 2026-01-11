#include "baro.h"

MS5611 barometer(BARO_ADDRESS_I2C);

bool initBaro(void){
  if(!barometer.begin()){
    return false;
  }
  barometer.setOversampling(OSR_ULTRA_HIGH);
  barometer.setCompensation(true);
  return barometer.reset(0);
}

float readBaroAltitude(){
  barometer.read();
  return barometer.getAltitude();
}