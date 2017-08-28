// Start without using interupts or DMP
// Don't need to use JR library at all in that case
// Check my first version of self-balancing if poss

// if donig by interupt, do you have to reset flat on MPU?


// THERE ARE STILL SOME JR FUNCTIONS IN THE BELOW
// Ned to replace with my own

// change to defines


// Config registers
const byte WHO_AM_I = 117;
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

const byte MPU_ADDRESS = 104; // I2C Address of MPU-6050

// PARAMETERS
// Do I need to handle the offsets myself? (if not using DMP?)
int XGyroOffset = -343;   // REQUIRES DERIVING FOR NEW MPU
int YGyroOffset = -232;   // REQUIRES DERIVING FOR NEW MPU
int ZGyroOffset = 28;     // REQUIRES DERIVING FOR NEW MPU
int ZAccelOffset = 847;   // REQUIRES DERIVING FOR NEW MPU
byte dlpf = 0;

unsigned long lastReadingTime;
unsigned long thisReadingTime;

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;   // measurement values
int16_t GyXOffset, GyYOffset, GyZOffset;

bool sensorRead;

void setupMotionSensor() {

  // ADD setting gyro range
  // ADD setting accel range
 
  writeBitsNew(MPU_ADDRESS,PWR_MGMT_1,7,1,1);  // resets the device
  delay(50);  // delay desirable after reset
  writeRegister(MPU_ADDRESS,PWR_MGMT_1,0);  // wake up the MPU-6050
//  writeBits(MPU_ADDRESS,INT_PIN_CFG,0,1,0); // Set interupt Active High - this is already default
//  writeBitsNew(MPU_ADDRESS,INT_ENABLE,0,1,1); // Enable Data Ready interupt
  writeBitsNew(MPU_ADDRESS,CONFIG,0,3,0); // set low pass filter to 0 - may need to review
  writeBitsNew(MPU_ADDRESS,PWR_MGMT_1,0,3,1); // sets clock source to X axis gyro (as recommended in user guide)


  byte MPU_ADDRESS_CHECK = readRegister(MPU_ADDRESS,WHO_AM_I);
  if(MPU_ADDRESS_CHECK==MPU_ADDRESS){
    Serial.println(F("MPU-6050 available"));
  }
  else {
    Serial.println(F("ERROR: MPU-6050 NOT FOUND"));
    Serial.println(F("Try reseting..."));
    while(1); // CHANGE TO SET SOME STATUS FLAG THAT CA BE SENT TO TRANSMITTER
  }
}


void readMainSensors(byte address) {
  //
  lastReadingTime = thisReadingTime;
  thisReadingTime = millis();
  I2c.read(address, ACCEL_XOUT_H, 14);
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
  }
  else {
    sensorRead = false;
//    Serial.println(I2c.available());
    flushI2cBuffer();
  }
} 

byte getInteruptStatus(byte address){
  return readRegister(address,INT_STATUS);
}

