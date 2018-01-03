#ifndef IMU_H
#define IMU_H

// SORT OUT ACCEL/GYRO RESOLUTION HANDLING

#include "MPU6050.h"


class IMU {
  public:
    int16_t accelOffsets[3], gyroOffsets[3];
    float accelAverages[3];
    float accelValues[3], gyroValues[3], tempValue; //accel in g, gyro in deg/s, temp in deg
    unsigned long lastReadingTime;
    unsigned long thisReadingTime;
    bool sensorRead;
    void applyOffsets();
    void gyroReadingsToValues();
    void accelReadingsToValues();
    void accumulateGyroChange();
    void averageAccelReadings();
    void calculateAccelAngles();
    void resetGyroChangeAccumulator();
    void complementaryFilter();
    void calculateOffsetValues();
    MPU6050 mpu;
    int16_t accelReadings[3], gyroReadings[3], tempReading; // raw readings, x, y, z
    float accelAngles[3], gyroAngles[3], gyroChangeAngles[3], currentAngles[3];
    IMU();
    void sensorOn();
    bool getData(); // rename? versions for gyro/accel only?
    void initialiseCurrentAngles();
    void updateRate();
    void updateAttitude();
    void calculateVerticalAccel();

};


#endif
