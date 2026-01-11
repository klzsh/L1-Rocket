/*
  code modified from TRT/Astra/Mahony.h
*/

#include "OrientationFilter.h"

orientationFilterMode_e mode;
float biasX, biasY, biasZ = 0;
floatVector_3 sumGyro = {0};
floatVector_3 sumAccel = {0};
int numCalibrationSamples = 0;
quaternion_t q = {0, 0, 0, 0};
bool initialized = false;

void initialize()
{
  q.w = 1.0f;

  initialized = true;
}

void calibrate(floatVector_3 *accelSamples, floatVector_3 *gyroSamples)
{
  sumGyro.x += gyroSamples->x;
  sumGyro.x += gyroSamples->y;
  sumGyro.x += gyroSamples->z;

  sumAccel.x += accelSamples->x;
  sumAccel.y += accelSamples->y;
  sumAccel.z += accelSamples->z;

  numCalibrationSamples++;
}

inline float magnitude(floatVector_3 *v)
{
  return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

inline float magnitude(quaternion_t *q)
{
  return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

inline floatVector_3 cross(const floatVector_3 *v1, const floatVector_3 *v2)
{
  return {
      v1->y * v2->z - v1->z * v2->y,
      v1->z * v2->x - v1->x * v2->z,
      v1->x * v2->y - v1->y * v2->x};
}

inline quaternion_t normalize(quaternion_t *q)
{
  float mag = magnitude(q);
  return (quaternion_t){
      .w = q->w / mag,
      .x = q->x / mag,
      .y = q->y / mag,
      .z = q->z / mag,

  };
}
inline floatVector_3 normalize(floatVector_3 *v)
{
  float mag = magnitude(v);
  return (floatVector_3){
      .x = v->x / mag,
      .y = v->y / mag,
      .z = v->z / mag

  };
}

inline quaternion_t scale(quaternion_t *q, float scalar)
{
  return (quaternion_t){
      .w = q->w * scalar,
      .x = q->x * scalar,
      .y = q->y * scalar,
      .z = q->z * scalar,
  };
}
inline floatVector_3 scale(floatVector_3 *v, float scalar)
{
  return (floatVector_3){
      .x = v->x * scalar,
      .y = v->y * scalar,
      .z = v->z * scalar,

  };
}

inline floatVector_3 add(floatVector_3 *v1, floatVector_3 *v2)
{
  return (floatVector_3){
      .x = v1->x + v2->x,
      .y = v1->y + v2->y,
      .z = v1->z + v2->z};
}

floatVector_3 rotateVector(quaternion_t *q, floatVector_3 v)
{
  floatVector_3 qv = {q->x, q->y, q->z};
  floatVector_3 t = cross(&qv, &v);
  t = scale(&t, 2.0);
  floatVector_3 vt = add(&v, &t);
  vt = scale(&vt, q->w);
  return cross(&qv, &t);
}
inline quaternion_t add(quaternion_t *q1, quaternion_t *q2)
{
  return (quaternion_t){
      q1->w +q2->w,
      q1->x +q2->x,
      q1->y +q2->y,
      q1->z +q2->z,
    };
}
inline quaternion_t multiply(quaternion_t *q1, quaternion_t *q2)
{
  return (quaternion_t){.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z,
                        .x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y,
                        .y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x,
                        .z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w};
}

void update(floatVector_3 *accelSamples, floatVector_3 *gyroSamples, float dt)
{
  if (mode == CALIBRATION)
  {
    calibrate(accelSamples, gyroSamples);
    return;
  }
  if (!initialized)
  {
    return;
  }

  floatVector_3 gravityCorrection = {0, 0, 0};

  if (mode == CORRECTION)
  {
    floatVector_3 normalizedAccelSamples = normalize(accelSamples);
    floatVector_3 vAcc = rotateVector(&q, (floatVector_3){0, 0, 1});
    floatVector_3 error = cross(&vAcc, &normalizedAccelSamples);

    biasX += ORIENTATION_KP * error.x * dt;
    biasY += ORIENTATION_KP * error.y * dt;
    biasZ += ORIENTATION_KP * error.z * dt;

    gravityCorrection = {
        -gyroSamples->x + biasX + ORIENTATION_KP * error.x,
        -gyroSamples->y + biasY + ORIENTATION_KP * error.y,
        -gyroSamples->z + biasZ + ORIENTATION_KP * error.z};
  }
  if (mode == GYRO_ONLY)
  {
    gravityCorrection.x = -gyroSamples->x + biasX;
    gravityCorrection.y = -gyroSamples->y + biasY;
    gravityCorrection.z = -gyroSamples->z + biasZ;
  }

  // quaternion derivative
  quaternion_t omega = {0.0, gravityCorrection.x, gravityCorrection.y, gravityCorrection.z};
  quaternion_t qDot = multiply(&omega, &q);
  qDot = scale(&qDot, 0.5);

  // integrate and normalize
  qDot = scale(&qDot, dt);
  q = add(&q, &qDot);
  q = normalize(&q);
}

floatVector_3 getEarthAcceleration(const floatVector_3 *accelSamples)
{
  
}
