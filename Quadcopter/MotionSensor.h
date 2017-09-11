// Start without using interupts or DMP

// ADD setting gyro range
// ADD setting accel range

// change to defines
// calibrate accel offsets


// X = ROLL
// Y = PITCH
// Z = YAW

// Config registers
const byte WHO_AM_I = 117;
const byte MPU_ADDRESS = 104; // I2C Address of MPU-6050
const byte PWR_MGMT_1 = 107;   // Power management
const byte CONFIG = 26;
const byte FS_SEL = 27; //  Gyro config
const byte AFS_SEL = 28; //  Accelerometer config
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
// Do I need to handle the offsets myself? (if not using DMP?)
int16_t GyXOffset = -419; 
int16_t GyYOffset = 328;
int16_t GyZOffset = 206;
int16_t AccelXOffset = 705;   // REQUIRES DERIVING FOR NEW MPU
int16_t AccelYOffset = -118;   // REQUIRES DERIVING FOR NEW MPU
int16_t AccelZOffset = 1874;   // REQUIRES DERIVING FOR NEW MPU

// MEASUREMENT
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;   // raw measurement values
float valAcX,valAcY,valAcZ,valTmp,valGyX,valGyY,valGyZ; // converted to real units
float AcXAve = 0, AcYAve = 0, AcZAve = 0;
float accelRes = 2.0f / 32768.0f;
float gyroRes = 250.0f / 32768.0f;
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
//struct angle gyroAngles;
struct angle gyroChangeAngles;
struct angle currentAngles;


// PARAMETERS
const byte DPLF_VALUE = 0;  // set low pass filter 
const float compFilterAlpha = 0.8; // weight applied to gyro angle estimate
const float compFilterAlphaComplement = 1- compFilterAlpha;
const float accelAverageAlpha = 0.2; // weight applied to new accel angle calculation in complementary filter
const float accelAverageAlphaComplement = 1 - accelAverageAlpha;


void setupMotionSensor() {

  // ADD setting gyro range
  // ADD setting accel range
 
  writeBitsNew(MPU_ADDRESS,PWR_MGMT_1,7,1,1);  // resets the device
  delay(50);  // delay desirable after reset
  writeRegister(MPU_ADDRESS,PWR_MGMT_1,0);  // wake up the MPU-6050
//  writeBits(MPU_ADDRESS,INT_PIN_CFG,0,1,0); // Set interupt Active High - this is already default
//  writeBitsNew(MPU_ADDRESS,INT_ENABLE,0,1,1); // Enable Data Ready interupt
  writeBitsNew(MPU_ADDRESS,CONFIG,0,3,DPLF_VALUE); // set low pass filter
  writeBitsNew(MPU_ADDRESS,PWR_MGMT_1,0,3,1); // sets clock source to X axis gyro (as recommended in user guide)


  byte MPU_ADDRESS_CHECK = readRegister(MPU_ADDRESS,WHO_AM_I);
  if(MPU_ADDRESS_CHECK==MPU_ADDRESS){
    Serial.println(F("MPU-6050 available"));
  }
  else {
    Serial.println(F("ERROR: MPU-6050 NOT FOUND"));
    Serial.println(F("Try reseting..."));
    while(1); // CHANGE TO SET SOME STATUS FLAG THAT CAN BE SENT TO TRANSMITTER
  }
}


bool readMainSensors() {
  //

  I2c.read(MPU_ADDRESS, ACCEL_XOUT_H, 14);
  // read the most significant bit register into the variable then shift to the left
  // and binary add the least significant
  if(I2c.available()==14){
  AcX=I2c.receive()<<8|I2c.receive();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)  
  AcY=I2c.receive()<<8|I2c.receive();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=I2c.receive()<<8|I2c.receive();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=I2c.receive()<<8|I2c.receive();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=I2c.receive()<<8|I2c.receive();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=I2c.receive()<<8|I2c.receive();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=I2c.receive()<<8|I2c.receive();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  sensorRead = true;
  lastReadingTime = thisReadingTime;
  thisReadingTime = micros();
  }
  else {
    sensorRead = false;
    flushI2cBuffer();
  }
  return sensorRead;
} 

byte getInteruptStatus(byte address){
  return readRegister(address,INT_STATUS);
}



void convertGyroReadingsToValues(){

  valGyX = (GyX - GyXOffset) * gyroRes;
  valGyY = (GyY - GyYOffset) * gyroRes;
  valGyZ = (GyZ - GyZOffset) * gyroRes;
}

//void convertGyroReadingsToValues(){
//
//  valGyX = (valGyX * 0.7) + (GyX - GyXOffset) * gyroRes * 0.3;
//  valGyY = (valGyY * 0.7) + (GyY - GyYOffset) * gyroRes * 0.3;
//  valGyZ = (valGyZ * 0.7) + (GyZ - GyZOffset) * gyroRes * 0.3;
//}

void accumulateGyroChange(){
  float interval = (thisReadingTime - lastReadingTime) * 0.000001;  // convert to seconds (from micros)
  gyroChangeAngles.roll += valGyX * interval;
  gyroChangeAngles.pitch += valGyY * interval;
  gyroChangeAngles.yaw += valGyZ * interval;
}

void accumulateAccelReadings(){
  // can filter on the accel readings without having to convert to values until we need to calculate an actual angle
  // actually, don't need to convert to values at all because we only need relative values
  // use a cyclic buffer ot just sum and low pass filter as we go?

  AcX = AcX - AccelXOffset;
  AcY = AcY - AccelYOffset;
  AcZ = AcZ - AccelZOffset;
  
  AcXAve = (AcXAve * accelAverageAlphaComplement) + (AcX * accelAverageAlpha);
  AcYAve = (AcYAve * accelAverageAlphaComplement) + (AcY * accelAverageAlpha);
  AcZAve = (AcZAve * accelAverageAlphaComplement) + (AcZ * accelAverageAlpha);

//  Serial.println(AcXAve);Serial.print('\t');
//  Serial.println(AcYAve);Serial.print('\t');
//  Serial.println(AcZAve);Serial.print('\n');
}


// consider splitting to work into parts to fit in when more time available (is that required)
// I think that each atan operation will take about 600us
void calcAnglesAccel(){
  // this will not work well for Z
  accelAngles.roll = atan2(AcYAve,AcZAve) * RAD_TO_DEG;
  accelAngles.pitch = atan2(AcXAve,AcZAve) * RAD_TO_DEG;
  accelAngles.yaw = atan2(AcXAve,AcYAve) * RAD_TO_DEG;
//  Serial.print("**********");Serial.print('\n');
//  Serial.print(accelAngles.roll);Serial.print('\t');
//  Serial.print(accelAngles.pitch);Serial.print('\t');
//  Serial.print(accelAngles.yaw);Serial.print('\t');
//  Serial.print(currentAngles.roll);Serial.print('\t');
//  Serial.print(currentAngles.pitch);Serial.print('\t');
//  Serial.print(currentAngles.yaw);Serial.print('\n');
//  Serial.print("**********");Serial.print('\n');
}

// QC must be sationary when this runs
void initialiseCurrentAngles(){
  // take a certain number of readings
  readMainSensors();  // just to get timing variables filled
  for(int i=0;i<200;i++){
    readMainSensors();
    accumulateAccelReadings();
    delay(5);
//    Serial.println(AcXAve);Serial.print('\t');
//  Serial.println(AcYAve);Serial.print('\t');
//  Serial.println(AcZAve);Serial.print('\n');
  }
//    Serial.print(AcXAve);Serial.print('\t');
//  Serial.print(AcYAve);Serial.print('\t');
//  Serial.print(AcZAve);Serial.print('\n');
  calcAnglesAccel();
  currentAngles.roll = accelAngles.roll;
  currentAngles.pitch = accelAngles.pitch;
  currentAngles.yaw = accelAngles.yaw;
}

void resetGyroChange(){
  gyroChangeAngles.roll = 0;
  gyroChangeAngles.pitch = 0;
  gyroChangeAngles.yaw = 0;
}


void mixAngles(){
  currentAngles.roll = ((currentAngles.roll + gyroChangeAngles.roll) * compFilterAlpha) + (accelAngles.roll * compFilterAlphaComplement);
  currentAngles.pitch = ((currentAngles.pitch + gyroChangeAngles.pitch) * compFilterAlpha) + (accelAngles.pitch * compFilterAlphaComplement);
  currentAngles.yaw = ((currentAngles.yaw + gyroChangeAngles.yaw) * 0.5) + (accelAngles.yaw * 0.5);
}

void calculateVerticalAccel(){
  ZAccel = AcZAve * accelRes;      // AcZAve has already been filtered, although I might wish to have a different filter parameter
}




void outputForProcessing(float a, float b, float c){
  Serial.print(a);Serial.print('\t');
  Serial.print(b);Serial.print('\t');
  Serial.print(c);Serial.print('\t');
  Serial.print('\n');

  
}













