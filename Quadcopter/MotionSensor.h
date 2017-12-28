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
int16_t AccelXOffset, AccelYOffset, AccelZOffset, GyXOffset, GyYOffset, GyZOffset;
const float gyroRes = 250.0f / 32768.0f;  // or could be (250 * pow(2,FS_SEL)) / 32768.0f
const float accelRes = 8.0f / 32768.0f;

const float MICROS_TO_SECONDS = 0.000001;

// MEASUREMENT
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // raw measurement values
float valAcX, valAcY, valAcZ, valTmp, valGyX, valGyY, valGyZ; // converted to real units
float AcXAve = 0, AcYAve = 0, AcZAve = 0;

unsigned long lastReadingTime; // For calculating angle change from gyros
unsigned long thisReadingTime; // For calculating angle change from gyros
bool sensorRead;  // indicates whether valid information was read from the sensor
float ZAccel;

struct angle {
  float roll;
  float pitch;
  float yaw;
};

struct angle accelAngles;
struct angle gyroAngles;
struct angle gyroChangeAngles;
struct angle currentAngles;


// PARAMETERS
const byte DPLF_VALUE = 3;  // set low pass filter
const byte FS_SEL = 0;  // gyro full scale range +/-250deg/s
const byte AFS_SEL = 2;  // accel full scale range +/-8g
const float compFilterAlpha = 0.99f; // weight applied to gyro angle estimate
const float compFilterAlphaComplement = 1.0f - compFilterAlpha; // remove this, compiler should optimise anyway
const float accelAverageAlpha = 0.1f; // weight applied to new accel angle calculation in complementary filter
const float accelAverageAlphaComplement = 1.0f - accelAverageAlpha; // remove this, compiler should optimise anyway


bool readGyrosAccels() {
  //
  I2c.read(MPU_ADDRESS, ACCEL_XOUT_H, 14);
  // read the most significant bit register into the variable then shift to the left
  // and binary add the least significant
  if (I2c.available() == 14) {
    AcX = I2c.receive() << 8 | I2c.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = I2c.receive() << 8 | I2c.receive(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = I2c.receive() << 8 | I2c.receive(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = I2c.receive() << 8 | I2c.receive(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = I2c.receive() << 8 | I2c.receive(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = I2c.receive() << 8 | I2c.receive(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = I2c.receive() << 8 | I2c.receive(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
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
    temperatureSum += Tmp;
    delay(2);
  }
  float temperature = (float)temperatureSum / (float)repetitions;
  AccelXOffset = (int)(( temperature * offsetScale[0] ) + offsetIntercept[0]);
  AccelYOffset = (int)(( temperature * offsetScale[1] ) + offsetIntercept[1]);
  AccelZOffset = (int)(( temperature * offsetScale[2] ) + offsetIntercept[2] - 16384);
  GyXOffset = (int)(( temperature * offsetScale[3] ) + offsetIntercept[3]);
  GyYOffset = (int)(( temperature * offsetScale[4] ) + offsetIntercept[4]);
  GyZOffset = (int)(( temperature * offsetScale[5] ) + offsetIntercept[5]);

  byte accelRangeFactor = pow(2, AFS_SEL);
  byte gyroRangeFactor = pow(2, FS_SEL);

  AccelXOffset /= accelRangeFactor;
  AccelYOffset /= accelRangeFactor;
  AccelZOffset /= accelRangeFactor;
  GyXOffset /= gyroRangeFactor;
  GyYOffset /= gyroRangeFactor;
  GyZOffset /= gyroRangeFactor;

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
  valGyX = (GyX - GyXOffset) * gyroRes;
  valGyY = (GyY - GyYOffset) * gyroRes;
  valGyZ = (GyZ - GyZOffset) * gyroRes;
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
  AcX = - AcX + AccelXOffset;
  AcY = AcY - AccelYOffset;
  AcZ = AcZ - AccelZOffset;
  AcXAve = (AcXAve * accelAverageAlphaComplement) + (AcX * accelAverageAlpha);
  AcYAve = (AcYAve * accelAverageAlphaComplement) + (AcY * accelAverageAlpha);
  AcZAve = (AcZAve * accelAverageAlphaComplement) + (AcZ * accelAverageAlpha);
}

// each atan operation will take about 600us
void calcAnglesAccel() {
  accelAngles.roll = atan2(AcYAve, AcZAve) * RAD_TO_DEG;
  accelAngles.pitch = atan2(AcXAve, AcZAve) * RAD_TO_DEG;
  //  accelAngles.yaw = atan2(AcXAve,AcYAve) * RAD_TO_DEG;
}

// QC must be sationary when this runs
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
  // calc gyro ony angle just for testing
  gyroAngles.roll += gyroChangeAngles.roll;
  gyroAngles.pitch += gyroChangeAngles.pitch;
  gyroAngles.yaw += gyroChangeAngles.yaw;

  currentAngles.roll = ((currentAngles.roll + gyroChangeAngles.roll) * compFilterAlpha) + (accelAngles.roll * compFilterAlphaComplement);
  currentAngles.pitch = ((currentAngles.pitch + gyroChangeAngles.pitch) * compFilterAlpha) + (accelAngles.pitch * compFilterAlphaComplement);
  //  currentAngles.yaw = ((currentAngles.yaw + gyroChangeAngles.yaw) * compFilterAlpha) + (accelAngles.yaw * compFilterAlphaComplement);
  currentAngles.yaw = currentAngles.yaw + gyroChangeAngles.yaw;   // add wrap around
}

void calculateVerticalAccel() {
  ZAccel = AcZAve * accelRes;      // AcZAve has already been filtered, although I might wish to have a different filter parameter
}

void calibrateGyro(int repetitions) {
  long GyXSum = 0, GyYSum = 0, GyZSum = 0;
  int i = 0;
  // throw away first 100 readings
  for (i = 0; i < 100; i++) {
    readGyrosAccels();
    delay(2);
  }
  for (i = 0; i < repetitions; i++) {
    readGyrosAccels();
    GyXSum += GyX;
    GyYSum += GyY;
    GyZSum += GyZ;
    delay(2);
  }
  GyXOffset = GyXSum / repetitions;
  GyYOffset = GyYSum / repetitions;
  GyZOffset = GyZSum / repetitions;
  Serial.print(GyXOffset); Serial.print('\t');
  Serial.print(GyYOffset); Serial.print('\t');
  Serial.print(GyZOffset); Serial.print('\n');

}


