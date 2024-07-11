#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "sensor.h"

#define FL_MOTOR 16
#define FR_MOTOR 14
#define BL_MOTOR 13
#define BR_MOTOR 12

#define MOTOR_P 0.01f
#define MOTOR_I 0.00f
#define MOTOR_D 0.00f


struct PID {

  float target, proportion, integral, derivative, runningIntegralValue, prevErr;
  
  PID(float p, float i, float d);
  float update(float reading, float delta);
  
};


struct FlightControl {
  
  PID pitchPID;
  PID yawPID;
  PID rollPID;
  float throttle;
  
  FlightControl();
  void update(Quaternion reading, float dtime);
  void setMotors(float pitch, float yaw, float roll, float throttle);
  void setTarget(Quaternion target);
  
};

#endif
