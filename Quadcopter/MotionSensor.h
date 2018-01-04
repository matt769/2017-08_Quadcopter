// X = ROLL
// Y = PITCH
// Z = YAW

// Config registers
const byte WHO_AM_I = 117;
const byte MPU_ADDRESS = 104; // I2C Address of MPU-6050
const byte PWR_MGMT_1 = 107;   // Power management
const byte CONFIG = 26;
const byte GYRO_CONFIG = 27; //  Gyro config
const byte ACCEL_CONFIG = 28; //  Accelerometer config
const byte INT_PIN_CFG = 55; // Interupt pin config
const byte INT_ENABLE = 56; // Interupt enable
const byte INT_STATUS = 58; // Interupt status

// Sensor registers
const byte ACCEL_XOUT_H = 59;   // [15:8]
const byte ACCEL_XOUT_L = 60;   //[7:0]
const byte ACCEL_YOUT_H = 61;   // [15:8]
const byte ACCEL_YOUT_L = 62;   //[7:0]
const byte ACCEL_ZOUT_H = 63;   // [15:8]
const byte ACCEL_ZOUT_L = 64;   //[7:0]
const byte TEMP_OUT_H = 65;
const byte TEMP_OUT_L = 66;
const byte GYRO_XOUT_H = 67;  // [15:8]
const byte GYRO_XOUT_L = 68;   //[7:0]
const byte GYRO_YOUT_H = 69;   // [15:8]
const byte GYRO_YOUT_L = 70;   //[7:0]
const byte GYRO_ZOUT_H = 71;   // [15:8]
const byte GYRO_ZOUT_L = 72;   //[7:0]

// DERIVE THESE SETTINGS FROM CALIBRATION & SETUP
// ax,ay,az,gx,gy,gz
const float offsetScale[6] = { 0.01129227174, -0.00323063182, -0.11709311610, -0.02385017929, 0.00375586283, 0.00117846130};
const float offsetIntercept[6] = { 875.974694, 34.84791487, 17830.3859, -557.7712577, 342.0514029, 207.8547826};
// will be used to populate these
int16_t accelXOffset, accelYOffset, accelZOffset, gyXOffset, gyYOffset, gyZOffset;  // to be populated during setup depending on the temperature


// MEASUREMENT
int16_t accX, accY, accZ, tmp, gyX, gyY, gyZ; // raw measurement values
float valAcX, valAcY, valAcZ, valTmp, valGyX, valGyY, valGyZ; // converted to real units
float accXAve = 0, accYAve = 0, accZAve = 0;
unsigned long lastReadingTime; // For calculating angle change from gyros
unsigned long thisReadingTime; // For calculating angle change from gyros
const float MICROS_TO_SECONDS = 0.000001;
bool sensorRead;  // indicates whether valid information was read from the sensor
const float gyroRes = (250.0f * pow(2,FS_SEL)) / 32768.0f; // FS_SEL = 0 -> 250.0f / 32768.0f; // see register map
const float accelRes = (2.0f * pow(2,AFS_SEL)) / 32768.0f;

struct angle {
  float roll;
  float pitch;
  float yaw;
};

struct angle accelAngles;
struct angle gyroAngles;
struct angle gyroChangeAngles;
struct angle currentAngles;

bool readGyrosAccels() {
  I2c.read(MPU_ADDRESS, ACCEL_XOUT_H, 14);
  if (I2c.available() == 14) {
    accX = I2c.receive() << 8 | I2c.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    accY = I2c.receive() << 8 | I2c.receive(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    accZ = I2c.receive() << 8 | I2c.receive(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    tmp = I2c.receive() << 8 | I2c.receive(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyX = I2c.receive() << 8 | I2c.receive(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyY = I2c.receive() << 8 | I2c.receive(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyZ = I2c.receive() << 8 | I2c.receive(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    lastReadingTime = thisReadingTime;
    thisReadingTime = micros();
    return true;
  }
  else {
    flushI2cBuffer();
    return false;
  }
}

bool readGyros() {
  I2c.read(MPU_ADDRESS, GYRO_XOUT_H, 6);
  if (I2c.available() == 6) {
    gyX = I2c.receive() << 8 | I2c.receive(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyY = I2c.receive() << 8 | I2c.receive(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyZ = I2c.receive() << 8 | I2c.receive(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    lastReadingTime = thisReadingTime;
    thisReadingTime = micros();
    return true;
  }
  else {
    flushI2cBuffer();
    return false;
  }
}


// this depends on pre-calculated values of how the output changes with temperature
void calculateOffsets() {
  // first, get the temperature
  long temperatureSum = 0;
  int repetitions = 500;
  for (int i = 0; i < 500; i++) {
    readGyrosAccels();
    delay(2);
  }
  for (int i = 0; i < repetitions; i++) {
    readGyrosAccels();
    temperatureSum += tmp;
    delay(2);
  }
  float temperature = (float)temperatureSum / (float)repetitions;
  accelXOffset = (int)(( temperature * offsetScale[0] ) + offsetIntercept[0]);
  accelYOffset = (int)(( temperature * offsetScale[1] ) + offsetIntercept[1]);
  accelZOffset = (int)(( temperature * offsetScale[2] ) + offsetIntercept[2] - 16384);
  gyXOffset = (int)(( temperature * offsetScale[3] ) + offsetIntercept[3]);
  gyYOffset = (int)(( temperature * offsetScale[4] ) + offsetIntercept[4]);
  gyZOffset = (int)(( temperature * offsetScale[5] ) + offsetIntercept[5]);

  byte accelRangeFactor = pow(2, AFS_SEL);
  byte gyroRangeFactor = pow(2, FS_SEL);

  accelXOffset /= accelRangeFactor;
  accelYOffset /= accelRangeFactor;
  accelZOffset /= accelRangeFactor;
  gyXOffset /= gyroRangeFactor;
  gyYOffset /= gyroRangeFactor;
  gyZOffset /= gyroRangeFactor;
}

void setupMotionSensor() {
  writeBitsNew(MPU_ADDRESS, PWR_MGMT_1, 7, 1, 1); // resets the device
  delay(50);  // delay desirable after reset
  writeRegister(MPU_ADDRESS, PWR_MGMT_1, 0); // wake up the MPU-6050
  writeBitsNew(MPU_ADDRESS, GYRO_CONFIG, 3, 2, FS_SEL); // set gyro full scale range
  writeBitsNew(MPU_ADDRESS, ACCEL_CONFIG, 3, 2, AFS_SEL); // set accel full scale range
  writeBitsNew(MPU_ADDRESS, CONFIG, 0, 3, DPLF_VALUE); // set low pass filter
  writeBitsNew(MPU_ADDRESS, PWR_MGMT_1, 0, 3, 1); // sets clock source to X axis gyro (as recommended in user guide)
  byte MPU_ADDRESS_CHECK = readRegister(MPU_ADDRESS, WHO_AM_I);
  if (MPU_ADDRESS_CHECK == MPU_ADDRESS) {
    Serial.println(F("MPU-6050 available"));
  }
  else {
    Serial.println(F("ERROR: MPU-6050 NOT FOUND"));
    Serial.println(F("Try reseting..."));
    while (1); // CHANGE TO SET SOME STATUS FLAG THAT CAN BE SENT TO TRANSMITTER
  }
}

void convertGyroReadingsToValues() {
  valGyX = (gyX - gyXOffset) * gyroRes;
  valGyY = (gyY - gyYOffset) * gyroRes;
  valGyZ = (gyZ - gyZOffset) * gyroRes;
}

void accumulateGyroChange() {
  float interval = (thisReadingTime - lastReadingTime) * MICROS_TO_SECONDS;  // convert to seconds (from micros)
  gyroChangeAngles.roll += valGyX * interval;
  gyroChangeAngles.pitch += valGyY * interval;
  gyroChangeAngles.yaw += valGyZ * interval;
  // Note faster alternative to this
  // use the reading value (valGy?) and do not convert to seconds
  // then only when actually populating gyroChangeAngle variables, apply gyroRes and convert to seconds
}

void accumulateAccelReadings() {
  // don't need to convert to values at all because we only need relative values

//Serial.print(accX);Serial.print('\t');
//Serial.print(accY);Serial.print('\t');
//Serial.print(accZ);Serial.print('\n');
  
  accX = - accX + accelXOffset;
  accY = accY - accelYOffset;
  accZ = accZ - accelZOffset;
//Serial.print(accX);Serial.print('\t');
//Serial.print(accY);Serial.print('\t');
//Serial.print(accZ);Serial.print('\t');
  accXAve = (accXAve * (1.0f - accelAverageAlpha)) + (accX * accelAverageAlpha);
  accYAve = (accYAve * (1.0f - accelAverageAlpha)) + (accY * accelAverageAlpha);
  accZAve = (accZAve * (1.0f - accelAverageAlpha)) + (accZ * accelAverageAlpha);
//  Serial.print(accXAve);Serial.print('\t');
//Serial.print(accYAve);Serial.print('\t');
//Serial.print(accZAve);Serial.print('\n');
}

void calcAnglesAccel() {
  accelAngles.roll = atan2(accYAve, accZAve) * RAD_TO_DEG;
  accelAngles.pitch = atan2(accXAve, accZAve) * RAD_TO_DEG;
  //  accelAngles.yaw = atan2(AcXAve,AcYAve) * RAD_TO_DEG;
}

// QC must be stationary when this runs
void initialiseCurrentAngles() {
  // take a certain number of readings
  readGyrosAccels();  // just to get timing variables filled
  for (int i = 0; i < 200; i++) {
    readGyrosAccels();
    accumulateAccelReadings();
    delay(5);
  }
  calcAnglesAccel();
  currentAngles.roll = accelAngles.roll;
  currentAngles.pitch = accelAngles.pitch;
  //  currentAngles.yaw = accelAngles.yaw;
  currentAngles.yaw = 0;
  gyroAngles.roll = currentAngles.roll;
  gyroAngles.pitch = currentAngles.pitch;
  gyroAngles.yaw = currentAngles.yaw;
}

void resetGyroChange() {
  gyroChangeAngles.roll = 0;
  gyroChangeAngles.pitch = 0;
  gyroChangeAngles.yaw = 0;
}

void mixAngles() {
  gyroAngles.roll += gyroChangeAngles.roll;   // calc gyro ony angle just for testing
  gyroAngles.pitch += gyroChangeAngles.pitch;
  gyroAngles.yaw += gyroChangeAngles.yaw;

  currentAngles.roll = ((currentAngles.roll + gyroChangeAngles.roll) * compFilterAlpha) + (accelAngles.roll * (1.0f - compFilterAlpha));
  currentAngles.pitch = ((currentAngles.pitch + gyroChangeAngles.pitch) * compFilterAlpha) + (accelAngles.pitch * (1.0f - compFilterAlpha));
  //  currentAngles.yaw = ((currentAngles.yaw + gyroChangeAngles.yaw) * compFilterAlpha) + (accelAngles.yaw * (1.0f - compFilterAlpha));
  currentAngles.yaw = currentAngles.yaw + gyroChangeAngles.yaw;   // add wrap around
}

void calculateVerticalAccel() {
  valAcZ = accZAve * accelRes;      // AcZAve has already been filtered, although I might wish to have a different filter parameter
}

void calibrateGyro(int repetitions) {
  long gyXSum = 0, gyYSum = 0, gyZSum = 0;
  int i = 0;
  // throw away first 100 readings
  for (i = 0; i < 100; i++) {
    readGyrosAccels();
    delay(2);
  }
  for (i = 0; i < repetitions; i++) {
    readGyrosAccels();
    gyXSum += gyX;
    gyYSum += gyY;
    gyZSum += gyZ;
    delay(2);
  }
  gyXOffset = gyXSum / repetitions;
  gyYOffset = gyYSum / repetitions;
  gyZOffset = gyZSum / repetitions;
//  Serial.print(gyXOffset); Serial.print('\t');
//  Serial.print(gyYOffset); Serial.print('\t');
//  Serial.print(gyZOffset); Serial.print('\n');

}


