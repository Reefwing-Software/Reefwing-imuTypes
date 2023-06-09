/******************************************************************
  @file       Reefwing_imuTypes.cpp
  @brief      Common structs, enums and classes for Reefwing IMU Libraries
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     2.0.1
  Date:        09/06/23

  1.0.0     Original Release.               19/04/23
  1.0.1     Minor documentation changes.    24/04/23
  2.0.0     Modified Quaternion class.      27/05/23
  2.0.1     Added I2C addresses             09/06/23

  Credit - Uses a modified version of the Madgwick Quaternion Class.
           (http://www.x-io.co.uk/quaternions/)

******************************************************************/

#include <Reefwing_imuTypes.h>
#include <math.h>

/******************************************************************
 *
 * Quarternion Implementation - 
 * 
 ******************************************************************/

Quaternion::Quaternion() {
  reset();
}

Quaternion::Quaternion(float w, float x, float y, float z) {
  q0 = w;
  q1 = x;
  q2 = y;
  q3 = z;
}

Quaternion::Quaternion(float yaw, float pitch, float roll) {
  //  Converts Euler Angles,  yaw (Z), pitch (Y), and roll (X) in radians
  //  to a quaternion.
  //  ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q0 = cr * cp * cy + sr * sp * sy;
  q1 = sr * cp * cy - cr * sp * sy;
  q2 = cr * sp * cy + sr * cp * sy;
  q3 = cr * cp * sy - sr * sp * cy;
}

void Quaternion::reset() {
  q0 = 1.0;
  q1 = q2 = q3 = 0.0;
}

Quaternion Quaternion::getConjugate() {
    Quaternion conjugate;

    conjugate.q0 = q0;
    conjugate.q1 = -q1;
    conjugate.q2 = -q2;
    conjugate.q3 = -q3;

    return conjugate;
}

EulerAngles Quaternion::getEulerAngles() {
    //  Madgwick Version
    EulerAngles eulerAngles;

    eulerAngles.roll = radiansToDegrees(atan2(2.0f * (q2 * q3 - q0 * q1), 2.0f * q0 * q0 - 1.0f + 2.0f * q3 * q3));
    eulerAngles.pitch = radiansToDegrees(-atan((2.0f * (q1 * q3 + q0 * q2)) / sqrt(1.0f - pow((2.0f * q1 * q3 + 2.0f * q0 * q2), 2.0f))));
    eulerAngles.yaw = radiansToDegrees(atan2(2.0f * (q1 * q2 - q0 * q3), 2.0f * q0 * q0 - 1.0f + 2.0f * q1 * q1));

    return eulerAngles;
}

EulerAngles Quaternion::toEulerAngles(float declination) {
  //  Converts a quaternion to Euler Angles
  //  ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  EulerAngles angles;

  angles.yawRadians   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);   
  angles.pitchRadians = -asin(2.0f * (q1 * q3 - q0 * q2));
  angles.rollRadians  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

  angles.pitch = angles.pitchRadians * 180.0f / M_PI;
  angles.yaw   = angles.yawRadians * 180.0f / M_PI; 
  angles.roll  = angles.rollRadians * 180.0f / M_PI;

  // Convert yaw to heading (normal compass degrees)   
  if (angles.yaw < 0) angles.yaw = angles.yaw + 360.0;
  if (angles.yaw >= 360.0) angles.yaw = angles.yaw - 360.0;

  angles.heading = angles.yaw - declination; // You need to subtract a positive declination.

  return angles;
}

float Quaternion::radiansToDegrees (float radians) {
    return 57.2957795130823f * radians;     //  180/PI * radians
}
