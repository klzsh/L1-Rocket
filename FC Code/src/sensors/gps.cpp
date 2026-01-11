#include "gps.h"

static const float EARTH_RADIUS= 6378.137e3;
static const float EARTH_FLATTENING = 1.0 / 298.257223563;
static const float ECC_SQRD = EARTH_FLATTENING * (2.0 - EARTH_FLATTENING);
static const float m = DEG_TO_RAD * EARTH_RADIUS;

SFE_UBLOX_GNSS gps;
floatVector_2 initialPosition = {0,0};
floatVector_2 currentPosition = {0,0};

bool initGPS()
{

  if (!gps.begin(EXTERNAL_I2C_BUS, GPS_ADDRESS_I2C))
  {
    return false;
  }
  gps.setI2COutput(COM_TYPE_UBX);
  gps.setNavigationFrequency(40);
  gps.setDynamicModel(DYN_MODEL_AIRBORNE4g);
  gps.setAutoPVT(true);
  gps.saveConfiguration();
  return true;
}

// taken from TRT/Astra/GPS
bool calculateInitialGPSCoords(){
  if(gps.getSIV() <= NUM_SATELLITES){
    Serial.println(gps.getSIV());
    return false;
  }
  //TODO: fix this so that the lat and lon are expressed as floats
  initialPosition.x = gps.getLatitude()/ 10000000.0f;
  initialPosition.y = gps.getLongitude()/ 10000000.0f;
  // Serial.printf("Lat: %f, Lon: %f\n", initialPosition.x, initialPosition.y);
  return true;
}
// taken from TRT/Astra/GPS
float wrapLongitude(double val)
{
    while (val > 180)
        val -= 360;
    while (val < -180)
        val += 360;
    return val;
}
// taken from TRT/Astra/GPS
floatVector_2 calculateDisplacement(){
  const float coslat = cos(currentPosition.x * DEG_TO_RAD);
  const float w2 = 1.0 / (1.0 - ECC_SQRD * (1.0 - coslat * coslat));
  const float w = sqrt(w2);

  float ky = m * w * coslat;                // IDK what this means
  float kx = m * w * w2 * (1.0 - ECC_SQRD); // IDK what this means

  float dx = wrapLongitude(currentPosition.x - initialPosition.x) * kx;
  float dy = (currentPosition.y - initialPosition.y) * ky;
  return {dx, dy};
}

bool getGPSData(GPSOutput_t *dataObj)
{
    
  if (!gps.getPVT() || gps.getInvalidLlh())
  {
    return false;
  }

  dataObj->latitude    = gps.getLatitude()  / 10000000.0;
  dataObj->longitude   = gps.getLongitude() / 10000000.0;
  dataObj->altitude    = gps.getAltitude()  / 1000.0;
  dataObj->heading     = gps.getHeading();
  dataObj->fixQuality  = gps.getSIV();
  dataObj->time = {
      .hour   = gps.getHour(),
      .minute = gps.getMinute(),
      .second = gps.getSecond(),
      .day    = gps.getDay(),
      .month  = gps.getMonth(),
      .year   = gps.getYear()};
  dataObj->velocity = {
      .x = gps.getNedNorthVel() / (float) 1000.0,
      .y = gps.getNedEastVel()  / (float) 1000.0,
      .z = gps.getNedDownVel()  / (float) 1000.0};
  dataObj->displacement = calculateDisplacement();

  return true;
}

GPSTime_t getGPSTime()
{
  return {
      .hour   = gps.getHour(),
      .minute = gps.getMinute(),
      .second = gps.getSecond(),
      .day    = gps.getDay(),
      .month  = gps.getMonth(),
      .year   = gps.getYear()};
}