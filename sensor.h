#ifndef SENSOR_H
#define SENSOR_H

#define MPU_I2C_ADDR 0x68
#define GYRO_CONF_ADDR 27
#define ACCEL_CONF_ADDR 28
#define ACCEL_ADDR 59
#define GYRO_ADDR 67
#define PWR_ADDR 107

#define CALIBRATION_SAMPLES 100
#define CALIBRATION_SPACING_MS 10

#define GYRO_SCALE 250 / 32767.0f //131.0f
#define ACCEL_SCALE 4 / 32767.0f

#define SENSOR_TILT -11.3f

#define RADIANS PI/180.0f
#define DEGREES 180/PI

#define GYRO_CORRECTION_RATE 0.5f

#define RENORMALIZATION_THRESHOLD 10

#include <Wire.h>
#include <Arduino.h>

struct IVec3 {
  short x;
  short y;
  short z;
  void print();
};

struct Vec3 {
  float x;
  float y;
  float z;
  void print();
};

struct PYR {
  float pitch;
  float yaw;
  float roll;
  void print();
};

struct Quaternion {//using this now

  float x;
  float y;
  float z;
  float w;

  void print();
  Quaternion add(Quaternion b);
  Quaternion subtract(Quaternion b);
  
  inline Quaternion rotate(Vec3 axis, float theta) {
    Quaternion b = fromAxisAngle(axis, theta);
    return b.multiply(*this);
  }
  
  inline Quaternion multiply(Quaternion b) {
    return Quaternion{
      this->w*b.x + this->x*b.w + this->y*b.z - this->z*b.y,
      this->w*b.y + this->y*b.w + this->z*b.x - this->x*b.z,
      this->w*b.z + this->z*b.w + this->x*b.y - this->y*b.x,
      this->w*b.w - this->x*b.x - this->y*b.y - this->z*b.z
    };
  }
  
  inline Quaternion inverse() {
    Quaternion retval = *this;
    retval.x *= -1.0f;
    retval.y *= -1.0f;
    retval.z *= -1.0f;
    return retval;
  }
  
  static inline Quaternion fromAxisAngle(Vec3 axis, float theta) {
    float sine = sinf(theta/2.0f);
    return Quaternion {
      sine * axis.x,
      sine * axis.y,
      sine * axis.z,
      cosf(theta/2.0f)
    };
  }

  static inline Quaternion fromPYR(PYR p) {
    float cp = cosf(p.pitch/2.0f);
    float cy = cosf(p.yaw/2.0f);
    float cr = cosf(p.roll/2.0f);

    float sp = sinf(p.pitch/2.0f);
    float sy = sinf(p.yaw/2.0f);
    float sr = sinf(p.roll/2.0f);
    
    return Quaternion {
      cy*cp*sr - sy*sp*cr,
      cy*sp*cr + sy*cp*sr,
      sy*cp*cr - cy*sp*sr,
      cy*cp*cr - sy*sp*sr
    };
  }
  
  inline void normalize() {
    float magnitude = sqrtf(this->x*this->x + this->y*this->y + this->z*this->z + this->w*this->w);
    this->x /= magnitude;
    this->y /= magnitude;
    this->z /= magnitude;
    this->w /= magnitude;
  }

  inline PYR toPYR() {
    Quaternion q = *this;
    PYR retval;

        // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    retval.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = std::sqrt(1.0f + 2.0f * (q.w * q.y - q.x * q.z));
    float cosp = std::sqrt(1.0f - 2.0f * (q.w * q.y - q.x * q.z));
    retval.pitch = 2.0f * std::atan2(sinp, cosp) - PI / 2.0f;

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    retval.yaw = std::atan2(siny_cosp, cosy_cosp);

    return retval;
  }
  
};

struct FlightData {
  FlightData();
  Vec3 displacement;
  Quaternion attitude;
  PYR perceivedDown;//direction sensor believes is down
  Vec3 gyroOffset;
  Vec3 accelOffset;

  Vec3 gyro;
  Vec3 accel;

  int lastNormalization;

  void calibrate();
  Vec3 getAccel();
  Vec3 getGyro();
  Vec3 isolatedAccel();
  Quaternion getAttitude();
  void update(float dtime);

  static Quaternion quaternionFromAccel(Vec3 accel);
};

void initSensor();
int8_t readReg(int8_t reg);
void writeReg(int8_t reg, int8_t value);

IVec3 readRawGyro();
IVec3 readRawAccel();

#endif
