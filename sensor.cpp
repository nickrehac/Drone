#include "sensor.h"


void initSensor() {
  
  Wire.begin(5, 4);
  writeReg(PWR_ADDR, 3);
  writeReg(GYRO_CONF_ADDR, 0);
  writeReg(ACCEL_CONF_ADDR, 0b00001000);
  
}


void IVec3::print() {
  
    Serial.print("x:");
    Serial.print(x);
    Serial.print(" y:");
    Serial.print(y);
    Serial.print(" z:");
    Serial.print(z);
    
}


void Vec3::print() {
  
    Serial.print("x:");
    Serial.print(x);
    Serial.print(" y:");
    Serial.print(y);
    Serial.print(" z:");
    Serial.print(z);
    
}


void PYR::print() {
  
    Serial.print("pitch:");
    Serial.print(pitch);
    Serial.print(" yaw:");
    Serial.print(yaw);
    Serial.print(" roll:");
    Serial.print(roll);
    
}


FlightData::FlightData() {
  
  displacement.x = 0.0f;
  displacement.y = 0.0f;
  displacement.z = 0.0f;

  attitude.pitch = SENSOR_TILT;
  attitude.yaw = 0.0f;
  attitude.roll = 0.0f;
  
}


void FlightData::calibrate() {
    
  int accX = 0;
  int accY = 0;
  int accZ = 0;

  int gyroX = 0;
  int gyroY = 0;
  int gyroZ = 0;

  for(int i = 0; i < CALIBRATION_SAMPLES; i++) {
      
    IVec3 rawGyro = readRawGyro();
    IVec3 rawAccel = readRawAccel();

    accX += rawAccel.x;
    accY += rawAccel.y;
    accZ += rawAccel.z;

    gyroX += rawGyro.x;
    gyroY += rawGyro.y;
    gyroZ += rawGyro.z;

    delay(CALIBRATION_SPACING_MS);
      
  }

  gyroOffset.x = gyroX / (float)CALIBRATION_SAMPLES * GYRO_SCALE;
  gyroOffset.y = gyroY / (float)CALIBRATION_SAMPLES * GYRO_SCALE;
  gyroOffset.z = gyroZ / (float)CALIBRATION_SAMPLES * GYRO_SCALE;

  accelOffset.x = accX / (float)CALIBRATION_SAMPLES * ACCEL_SCALE;
  accelOffset.y = accY / (float)CALIBRATION_SAMPLES * ACCEL_SCALE;
  accelOffset.z = accZ / (float)CALIBRATION_SAMPLES * ACCEL_SCALE;

  //account for sensor tilt
  accelOffset.x -= sinf(SENSOR_TILT * RADIANS);
  accelOffset.z -= cosf(SENSOR_TILT * RADIANS);

}


PYR FlightData::pyrFromAccel(Vec3 accel) {

  PYR retval;

  retval.pitch = atan2f(accel.x, accel.z)*DEGREES;
  retval.yaw = 0.0f;
  retval.roll = atan2f(accel.y, accel.z)*DEGREES;

  return retval;
  
}


void FlightData::update(float dtime) {

  gyro = getGyro();
  accel = getAccel();
  PYR accelPYR = pyrFromAccel(accel);

  float pitchCorrection = (accelPYR.pitch - attitude.pitch) * GYRO_CORRECTION_RATE;
  float yawCorrection = (accelPYR.yaw - attitude.yaw) * GYRO_CORRECTION_RATE;
  float rollCorrection = (accelPYR.roll - attitude.roll) * GYRO_CORRECTION_RATE;

  float newPitch

  float pitchChange = gyro.y * cos(attitude.roll*RADIANS) + gyro.z * -sin(attitude.roll * RADIANS);
  float yawChange = gyro.z * cos(attitude.roll*RADIANS) + gyro.y * sin(attitude.roll * RADIANS);
  float rollChange = gyro.x;
  
  attitude.pitch -= (pitchChange - pitchCorrection) * dtime;
  attitude.yaw += (yawChange + yawCorrection) * dtime;
  attitude.roll += (rollChange + rollCorrection) * dtime;
  
}


PYR FlightData::getAttitude() {//calculate attitude from corrected data
  
  PYR retval = attitude;

  retval.pitch -= SENSOR_TILT;
  
  return retval;
  
}


Vec3 FlightData::isolatedAccel() {
  
  Vec3 retval = accel;
    
  float biasX = sinf(attitude.pitch * RADIANS);
  float biasZ = cosf(attitude.pitch * RADIANS) + cosf(attitude.roll * RADIANS) - 1.0f;
  float biasY = sinf(attitude.roll * RADIANS);

  retval.x -= biasX;
  retval.y -= biasY;
  retval.z -= biasZ;

  return retval;
  
}


Vec3 FlightData::getAccel() {
  
  Vec3 retval;
  IVec3 rawAccel = readRawAccel();
       
  retval.x = rawAccel.x * ACCEL_SCALE - accelOffset.x;
  retval.y = rawAccel.y * ACCEL_SCALE - accelOffset.y;
  retval.z = rawAccel.z * ACCEL_SCALE - accelOffset.z;
  
  return retval;
  
}


Vec3 FlightData::getGyro() {
  
  Vec3 retval;
  IVec3 rawGyro = readRawGyro();

  retval.x = rawGyro.x * GYRO_SCALE - gyroOffset.x;
  retval.y = rawGyro.y * GYRO_SCALE - gyroOffset.y;
  retval.z = rawGyro.z * GYRO_SCALE - gyroOffset.z;

  return retval;
  
}


int8_t readReg(int8_t reg) {
  
  int8_t retval;
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.requestFrom(MPU_I2C_ADDR, 1);
  retval = Wire.read();
  Wire.endTransmission();
  
  return retval;
  
}


void writeReg(int8_t reg, int8_t value) {
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  
}


IVec3 readRawAccel() {
  
  IVec3 retval;
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(ACCEL_ADDR);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.requestFrom(MPU_I2C_ADDR, 6);
  
  char temp[2];
  
  temp[1] = Wire.read();
  temp[0] = Wire.read();
  retval.x = *((short*) temp);
  
  temp[1] = Wire.read();
  temp[0] = Wire.read();
  retval.y = *((short*) temp);
  
  temp[1] = Wire.read();
  temp[0] = Wire.read();
  retval.z = *((short*) temp);

  Wire.endTransmission();

  return retval;
  
}


IVec3 readRawGyro() {
  
  IVec3 retval;
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(GYRO_ADDR);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.requestFrom(MPU_I2C_ADDR, 6);
  
  char temp[2];
  
  temp[1] = Wire.read();
  temp[0] = Wire.read();
  retval.x = *((short*) temp);
  
  temp[1] = Wire.read();
  temp[0] = Wire.read();
  retval.y = *((short*) temp);
  
  temp[1] = Wire.read();
  temp[0] = Wire.read();
  retval.z = *((short*) temp);

  Wire.endTransmission();
  

  return retval;
  
}
