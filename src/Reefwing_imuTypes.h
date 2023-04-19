/******************************************************************
  @file       Reefwing_imuTypes.h
  @brief      Common structs, enums and classes for Reefwing Libraries
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        19/04/23

  1.0.0     Original Release.       19/04/23

  Credit - Uses the Madgwick Quaternion Class.
           (http://www.x-io.co.uk/quaternions/)

******************************************************************/

#ifndef Reefwing_imuTypes_h
#define Reefwing_imuTypes_h

#include <Arduino.h>

struct EulerAngles {
  float roll;     /* rotation around x axis in degrees */
  float pitch;    /* rotation around y axis in degrees */
  float yaw;      /* rotation around z axis in degrees */
  uint32_t  timeStamp;
};

struct InertialMessage {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t timeStamp;
};

struct RawData {
  int16_t rx, ry, rz;
  uint32_t  timeStamp;
};

struct ScaledData {
  float sx, sy, sz;
  uint32_t  timeStamp;
};

struct TempData {
  float celsius;
  uint32_t  timeStamp;
};

struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  uint32_t gTimeStamp, aTimeStamp, mTimeStamp;
};

/******************************************************************
 *
 * Quaternion Class Definition - 
 * 
 ******************************************************************/

class Quaternion {
    public:
        Quaternion(void);
        Quaternion(const float w, const float x, const float y, const float z);
        Quaternion getConjugate(void) const;
        EulerAngles getEulerAngles(void) const;

        float q[4];     //  Euler Parameters
        uint32_t  timeStamp;

    private:
        float radiansToDegrees (float radians) const;
};

#endif