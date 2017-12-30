#include "IMU.h"
#include "Parameters.h"


IMU::IMU() {
}


bool IMU::getData() {   // for testing, will rename later
  unsigned long tm = micros();
  sensorRead = mpu.getAllSensors(accelReadings, &tempReading, gyroReadings);
  if (sensorRead) {
    lastReadingTime = thisReadingTime;
    thisReadingTime = tm;
  }
  return sensorRead;
}

void IMU::sensorOn() {
  mpu.reset();
  delay(50);
  mpu.defaultConfig();
  mpu.on();
  lastReadingTime = micros();
}

void IMU::applyOffsets() {
  gyroReadings[0] = gyroReadings[0] - gyroOffsets[0];
  gyroReadings[1] = gyroReadings[1] - gyroOffsets[1];
  gyroReadings[2] = gyroReadings[2] - gyroOffsets[2];
  accelReadings[0] = -accelReadings[0] +- accelOffsets[0]; // this probably belongs elsewhere
  accelReadings[1] = accelReadings[1] - accelOffsets[1];
  accelReadings[2] = accelReadings[2] - accelOffsets[2];
}

void IMU::gyroReadingsToValues() {
  gyroValues[0] = gyroReadings[0] * mpu.gyroResolution;
  gyroValues[1] = gyroReadings[1] * mpu.gyroResolution;
  gyroValues[2] = gyroReadings[2] * mpu.gyroResolution;
}

void IMU::accelReadingsToValues() {
  accelValues[0] = accelReadings[0] * mpu.accelResolution;
  accelValues[1] = accelReadings[1] * mpu.accelResolution;
  accelValues[2] = accelReadings[2] * mpu.accelResolution;
}

void IMU::accumulateGyroChange() {
  float interval = (thisReadingTime - lastReadingTime) * MICROS_TO_SECONDS;
  gyroChangeAngles[0] += gyroValues[0] * interval;
  gyroChangeAngles[1] += gyroValues[1] * interval;
  gyroChangeAngles[2] += gyroValues[2] * interval;
  // Note faster alternative to this
  // use the raw reading value and do not convert to seconds
  // then only when actually populating gyroChangeAngle variables, apply gyroRes and convert to seconds
}


void IMU::averageAccelReadings() {
  // don't need to convert to values at all because we only need relative values
  accelAverages[0] = (accelAverages[0] * (1.0f - accelAverageAlpha)) + (accelReadings[0] * accelAverageAlpha);
  accelAverages[1] = (accelAverages[1] * (1.0f - accelAverageAlpha)) + (accelReadings[1] * accelAverageAlpha);
  accelAverages[2] = (accelAverages[2] * (1.0f - accelAverageAlpha)) + (accelReadings[2] * accelAverageAlpha);
}


void IMU::calculateAccelAngles() {
  accelAngles[0] = atan2(accelAverages[1], accelAverages[2]) * RAD_TO_DEG;
  accelAngles[1] = atan2(accelAverages[0], accelAverages[2]) * RAD_TO_DEG;
  //  accelAverages[2] = atan2(accelAverages[0],accelAverages[1]) * RAD_TO_DEG;
}

void IMU::initialiseCurrentAngles() {
  calculateOffsetValues();
  // take a certain number of readings
  getData();  // just to get timing variables filled
  for (int i = 0; i < 1000; i++) {
    getData();
    averageAccelReadings();
    delay(1);
  }
  calculateAccelAngles();
  currentAngles[0] = accelAngles[0];
  currentAngles[1] = accelAngles[1];
  //  currentAngles[2] = accelAngles[2];
  currentAngles[2] = 0;
  gyroAngles[0] = currentAngles[0];
  gyroAngles[1] = currentAngles[1];
  gyroAngles[2] = currentAngles[2];
}

void IMU::resetGyroChangeAccumulator() {
  gyroChangeAngles[0] = 0;
  gyroChangeAngles[1] = 0;
  gyroChangeAngles[2] = 0;
}
void IMU::complementaryFilter() {
  // calc gyro ony angle just for testing
  gyroAngles[0] += gyroChangeAngles[0];
  gyroAngles[1] += gyroChangeAngles[1];
  gyroAngles[2] += gyroChangeAngles[2];

  currentAngles[0] = ((currentAngles[0] + gyroChangeAngles[0]) * compFilterAlpha) + (accelAngles[0] * (1.0f - compFilterAlpha));
  currentAngles[1] = ((currentAngles[1] + gyroChangeAngles[1]) * compFilterAlpha) + (accelAngles[1] * (1.0f - compFilterAlpha));
  //  currentAngles[2] = ((currentAngles[2] + gyroChangeAngles[2]) * compFilterAlpha) + (accelAngles[2] * (1.0f - compFilterAlpha));
  currentAngles[2] = currentAngles[2] + gyroChangeAngles[2];   // add wrap around
}

void IMU::updateRate(){
    getData();
    applyOffsets();
    gyroReadingsToValues();
    accumulateGyroChange();
    averageAccelReadings();
}

void IMU::updateAttitude(){
  // put 'full' routine in here

    calculateAccelAngles();
    complementaryFilter();
    resetGyroChangeAccumulator();
}


void IMU::calculateOffsetValues() {
  // first, get the temperature
  int32_t temperatureSum = 0;
  uint16_t repetitions = 500;
  for (int i = 0; i < 200; i++) {
    getData();
    delay(2);
  }
  for (int i = 0; i < repetitions; i++) {
    getData();
    temperatureSum += tempReading;
    delay(2);
  }
  float temperature = (float)temperatureSum / (float)repetitions;
  accelOffsets[0] = (int)(( temperature * offsetAccelScale[0] ) + offsetAccelIntercept[0]);
  accelOffsets[1] = (int)(( temperature * offsetAccelScale[1] ) + offsetAccelIntercept[1]);
  accelOffsets[2] = (int)(( temperature * offsetAccelScale[2] ) + offsetAccelIntercept[2] - 16384);
  gyroOffsets[0] = (int)(( temperature * offsetGyroScale[0] ) + offsetGyroIntercept[0]);
  gyroOffsets[1] = (int)(( temperature * offsetGyroScale[1] ) + offsetGyroIntercept[1]);
  gyroOffsets[2] = (int)(( temperature * offsetGyroScale[2] ) + offsetGyroIntercept[2]);

  byte accelRangeFactor = pow(2, accelFullScaleRange);
  byte gyroRangeFactor = pow(2, gyroFullScaleRange);

  accelOffsets[0] /= accelRangeFactor;
  accelOffsets[10] /= accelRangeFactor;
  accelOffsets[2] /= accelRangeFactor;
  gyroOffsets[0] /= gyroRangeFactor;
  gyroOffsets[1] /= gyroRangeFactor;
  gyroOffsets[2] /= gyroRangeFactor;

}

void IMU::calculateVerticalAccel() {
  accelValues[2] = accelAverages[2] * mpu.accelResolution;      // AcZAve has already been filtered, although I might wish to have a different filter parameter
}

