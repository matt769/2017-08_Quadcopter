#include "MPU6050.h"
#include "I2CHelper.h"

// Config registers
static const uint8_t WHO_AM_I = 117;
static const uint8_t MPU_ADDRESS = 104; // I2C Address of MPU-6050
static const uint8_t PWR_MGMT_1 = 107;   // Power management
static const uint8_t CONFIG = 26;
static const uint8_t GYRO_CONFIG = 27; //  Gyro config
static const uint8_t ACCEL_CONFIG = 28; //  Accelerometer config
static const uint8_t INT_PIN_CFG = 55; // Interupt pin config
static const uint8_t INT_ENABLE = 56; // Interupt enable
static const uint8_t INT_STATUS = 58; // Interupt status

// Sensor registers
static const uint8_t ACCEL_XOUT_H = 59;   // [15:8]
static const uint8_t ACCEL_XOUT_L = 60;   //[7:0]
static const uint8_t ACCEL_YOUT_H = 61;   // [15:8]
static const uint8_t ACCEL_YOUT_L = 62;   //[7:0]
static const uint8_t ACCEL_ZOUT_H = 63;   // [15:8]
static const uint8_t ACCEL_ZOUT_L = 64;   //[7:0]
static const uint8_t TEMP_OUT_H = 65;
static const uint8_t TEMP_OUT_L = 66;
static const uint8_t GYRO_XOUT_H = 67;  // [15:8]
static const uint8_t GYRO_XOUT_L = 68;   //[7:0]
static const uint8_t GYRO_YOUT_H = 69;   // [15:8]
static const uint8_t GYRO_YOUT_L = 70;   //[7:0]
static const uint8_t GYRO_ZOUT_H = 71;   // [15:8]
static const uint8_t GYRO_ZOUT_L = 72;   //[7:0]

extern I2CHelper I2CH;

MPU6050::MPU6050() {
  gyroResolution = (250.0f * pow(2,0)) / 32768.0f;	// default
  accelResolution = (2.0f * pow(2,0)) / 32768.0f;	// default
}


void MPU6050::on() {
  I2CH.writeRegister(MPU_ADDRESS, PWR_MGMT_1, 0); // wake up the MPU-6050
}

void MPU6050::reset() {
  I2CH.writeBits(MPU_ADDRESS, PWR_MGMT_1, 7, 1, 1); // wake up the MPU-6050
}

void MPU6050::setLowPassFilter(uint8_t f) {
  I2CH.writeBits(MPU_ADDRESS, CONFIG, 0, 3, f);
}

void MPU6050::setGyroFullScaleRange(uint8_t range) {
  I2CH.writeBits(MPU_ADDRESS, GYRO_CONFIG, 3, 2, range);
  gyroResolution = (250.0f * pow(2,range)) / 32768.0f;
}

void MPU6050::setAccelFullScaleRange(uint8_t range) {
  I2CH.writeBits(MPU_ADDRESS, ACCEL_CONFIG, 3, 2, range);
  accelResolution = (2.0f * pow(2,range)) / 32768.0f;
}

void MPU6050::setClockSource(uint8_t s) {
  // don't think I'll need to include the other options // this is clock source X axis gyroscope
  I2CH.writeBits(MPU_ADDRESS, PWR_MGMT_1, 0, 3, s);
}

void MPU6050::defaultConfig(){
  setLowPassFilter(3);
  setGyroFullScaleRange(0);
  setAccelFullScaleRange(2);
  setClockSource(1);
}


uint8_t getGyroFullScaleRange(){
}
uint8_t getAccelFullScaleRange();


bool MPU6050::getAccel(int16_t sensorData[]) {
  I2CH.read(MPU_ADDRESS, ACCEL_XOUT_H, 6);
  if (I2CH.available() == 6) {
    sensorData[0] = I2CH.receive() << 8 | I2CH.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    sensorData[1] = I2CH.receive() << 8 | I2CH.receive(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    sensorData[2] = I2CH.receive() << 8 | I2CH.receive(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    return true;
  }
  else {
    I2CH.flushI2cBuffer();
    return false;
  }
}

bool MPU6050::getAccel(int16_t *x, int16_t *y, int16_t * z) {
  I2CH.read(MPU_ADDRESS, ACCEL_XOUT_H, 6);
  if (I2CH.available() == 6) {
    *x = I2CH.receive() << 8 | I2CH.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    *y = I2CH.receive() << 8 | I2CH.receive(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    *z = I2CH.receive() << 8 | I2CH.receive(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    return true;
  }
  else {
    I2CH.flushI2cBuffer();
    return false;
  }
}

bool MPU6050::getGyro(int16_t sensorData[]) {
  I2CH.read(MPU_ADDRESS, GYRO_XOUT_H, 6);
  if (I2CH.available() == 6) {
    sensorData[0] = I2CH.receive() << 8 | I2CH.receive();
    sensorData[1] = I2CH.receive() << 8 | I2CH.receive();
    sensorData[2] = I2CH.receive() << 8 | I2CH.receive();
    return true;
  }
  else {
    I2CH.flushI2cBuffer();
    return false;
  }
}

bool MPU6050::getGyro(int16_t *x, int16_t *y, int16_t *z) {
  I2CH.read(MPU_ADDRESS, GYRO_XOUT_H, 6);
  if (I2CH.available() == 6) {
    *x = I2CH.receive() << 8 | I2CH.receive();
    *y = I2CH.receive() << 8 | I2CH.receive();
    *z = I2CH.receive() << 8 | I2CH.receive();
    return true;
  }
  else {
    I2CH.flushI2cBuffer();
    return false;
  }
}

bool MPU6050::getAllSensors(int16_t sensorData[]) {
  I2CH.read(MPU_ADDRESS, ACCEL_XOUT_H, 14);
  if (I2CH.available() == 14) {
    sensorData[0] = I2CH.receive() << 8 | I2CH.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    sensorData[1] = I2CH.receive() << 8 | I2CH.receive(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    sensorData[2] = I2CH.receive() << 8 | I2CH.receive(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)#
    sensorData[3] = I2CH.receive() << 8 | I2CH.receive(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    sensorData[4] = I2CH.receive() << 8 | I2CH.receive(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    sensorData[5] = I2CH.receive() << 8 | I2CH.receive(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    sensorData[6] = I2CH.receive() << 8 | I2CH.receive(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    return true;
  }
  else {
    I2CH.flushI2cBuffer();
    return false;
  }
}

bool MPU6050::getAllSensors(int16_t accelData[], int16_t* temp, int16_t gyroData[]) {
  I2CH.read(MPU_ADDRESS, ACCEL_XOUT_H, 14);
  if (I2CH.available() == 14) {
    accelData[0] = I2CH.receive() << 8 | I2CH.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    accelData[1] = I2CH.receive() << 8 | I2CH.receive(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    accelData[2] = I2CH.receive() << 8 | I2CH.receive(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)#
    *temp = I2CH.receive() << 8 | I2CH.receive(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyroData[0] = I2CH.receive() << 8 | I2CH.receive(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyroData[1] = I2CH.receive() << 8 | I2CH.receive(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyroData[2] = I2CH.receive() << 8 | I2CH.receive(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    return true;
  }
  else {
    I2CH.flushI2cBuffer();
    return false;
  }
}

bool MPU6050::getAllSensors(int16_t *ax, int16_t *ay, int16_t *az, int16_t *t, int16_t *gx, int16_t *gy, int16_t *gz) {
  I2CH.read(MPU_ADDRESS, ACCEL_XOUT_H, 14);
  if (I2CH.available() == 14) {
    *ax = I2CH.receive() << 8 | I2CH.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    *ay = I2CH.receive() << 8 | I2CH.receive(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    *az = I2CH.receive() << 8 | I2CH.receive(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    *t = I2CH.receive() << 8 | I2CH.receive(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    *gx = I2CH.receive() << 8 | I2CH.receive(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    *gy = I2CH.receive() << 8 | I2CH.receive(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    *gz = I2CH.receive() << 8 | I2CH.receive(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    return true;
  }
  else {
    I2CH.flushI2cBuffer();
    return false;
  }
}

bool MPU6050::getTemperature(int16_t *temperature) {
  I2CH.read(MPU_ADDRESS, TEMP_OUT_H, 2);
  if (I2CH.available() == 2) {
    *temperature = I2CH.receive() << 8 | I2CH.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    return true;
  }
  else {
    I2CH.flushI2cBuffer();
    return false;
  }
}



