#include "imu.h"

static Bmi088Gyro gyro(INTERNAL_I2C_BUS, GYRO_ADDRESS_I2C);
static Bmi088Accel accel(INTERNAL_I2C_BUS, ACCEL_ADDRESS_I2C);
floatVector_3 previousAccelData = {0, 0, 0};
floatVector_3 previousGyroData = {0, 0, 0};


bool initBMI088(void)
{
  int accelCode = accel.begin();
  int gyroCode = gyro.begin();

  if (accelCode < 0 || gyroCode < 0)
  {
    // LOG Errors
    return false;
  }

  accel.setOdr(accel.ODR_1600HZ_BW_280HZ);
  gyro.setOdr(gyro.ODR_2000HZ_BW_532HZ);

  accel.setRange(accel.RANGE_24G);

  gyro.setRange(gyro.RANGE_1000DPS);

  return true;
}


bool readAccelData(floatVector_3 *data)
{
  if (!accel.getDrdyStatus())
  {
    return false;
  }
  accel.readSensor();

  data->x = BODY_ACCEL_REFERENCE_X;
  data->y = BODY_ACCEL_REFERENCE_Y;
  data->z = BODY_ACCEL_REFERENCE_Z;
  return true;
}
bool readGyroData(floatVector_3 *data)
{
  if (!gyro.getDrdyStatus())
  {
    return false;
  }
  gyro.readSensor();

  data->x = BODY_GYRO_REFERENCE_X;
  data->y = BODY_GYRO_REFERENCE_X;
  data->z = BODY_GYRO_REFERENCE_X;
  return true;
}