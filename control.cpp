#include "control.h"


PID::PID(float p, float i, float d) {
  
  this->proportion = p;
  this->integral = i;
  this->derivative = d;
  this->runningIntegralValue = 0.0;
  this->prevErr = 0.0;
  
}


float PID::update(float reading, float delta) {
  
  float err = target - reading;//how much reading needs to change to match target
  
  this->runningIntegralValue += delta*err*this->integral;
  float derivativeComponent = derivative*(err - this->prevErr)/delta;//positive if err has increased, negative if err has decreased
  prevErr = err;
  
  return err*proportion + this->runningIntegralValue + derivativeComponent;
  
}


FlightControl::FlightControl() : 
  pitchPID(MOTOR_P, MOTOR_I, MOTOR_D),
  yawPID(MOTOR_P, MOTOR_I, MOTOR_D),
  rollPID(MOTOR_P, MOTOR_I, MOTOR_D) {

  pinMode(FL_MOTOR, OUTPUT);
  pinMode(FR_MOTOR, OUTPUT);
  pinMode(BL_MOTOR, OUTPUT);
  pinMode(BR_MOTOR, OUTPUT);
  
}


void FlightControl::update(Quaternion reading, float dtime) {
/*
  float pitch = pitchPID.update(reading.pitch, dtime);
  float yaw = yawPID.update(reading.yaw, dtime);
  float roll = rollPID.update(reading.roll, dtime);

  
  Serial.print("pitch:");
  Serial.print(pitch);
  Serial.print(" yaw:");
  Serial.print(yaw);
  Serial.print(" roll:");
  Serial.println(roll);
  

  float mFL = 0.0f;
  float mFR = 0.0f;
  float mBL = 0.0f;
  float mBR = 0.0f;

  mFL += pitch;
  mFR += pitch;

  mBL -= pitch;
  mBR -= pitch;

  mFR -= roll;
  mBR -= roll;

  mFL += roll;
  mBL += roll;

  mFR += yaw;
  mBL += yaw;

  mFL -= yaw;
  mBR -= yaw;

  mFL += throttle;
  mFR += throttle;
  mBL += throttle;
  mBR += throttle;

  analogWrite(FL_MOTOR, constrain(mFL, 0.0f, 1.0f)*255);
  analogWrite(FR_MOTOR, constrain(mFR, 0.0f, 1.0f)*255);
  analogWrite(BL_MOTOR, constrain(mBL, 0.0f, 1.0f)*255);
  analogWrite(BR_MOTOR, constrain(mBR, 0.0f, 1.0f)*255);

  */
  
}


void FlightControl::setTarget(Quaternion target) {
}


void FlightControl::setMotors(float pitch, float yaw, float roll, float throttle) {


  
}
