// add a default config
// change setting functions to take the actual values and convert them to the required register setting
//  how to handle incorrect input? apply default

// add definition for     
//	uint8_t getGyroFullScaleRange();
//	uint8_t getAccelFullScaleRange();


#ifndef MPU6050_H
#define MPU6050_H

//#include "I2CHelper.h"
#include "MPU6050.h"
#include <Arduino.h>

class MPU6050 {
  public:
    float gyroResolution;
    float accelResolution;
    MPU6050();
    void on();
    void reset();
    void setLowPassFilter(uint8_t f);
    void setGyroFullScaleRange(uint8_t r);
    void setAccelFullScaleRange(uint8_t r);
    uint8_t getGyroFullScaleRange();
    uint8_t getAccelFullScaleRange();
    void setClockSource(uint8_t s);
    void defaultConfig();
    bool getAccel(int16_t data[]);
    bool getAccel(int16_t*x, int16_t*y, int16_t*z);
    bool getGyro(int16_t data[]);
    bool getGyro(int16_t*x, int16_t*y, int16_t*z);
    bool getAllSensors(int16_t []);
    bool getAllSensors(int16_t accels[], int16_t* temp, int16_t gyros[]);
    bool getAllSensors(int16_t *ax, int16_t *ay, int16_t *az, int16_t *t, int16_t *gx, int16_t *gy, int16_t *gz);
    bool getTemperature(int16_t *t);

};


#endif
