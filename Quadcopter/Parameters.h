/*
// PARAMETERS
const int THROTTLE_LIMIT = 1500;

const byte receiverFreq = 50;
const int batteryFreq = 1000;


// SETTINGS/CONFIG
const byte pinStatusLed = 8;
const byte PIN_BATTERY_MONITOR = A0;
const byte PIN_BATTERY_INDICATOR = 8;


// Most motion sensor settings, register numbers etc are still kept in MotionSensor.h
// Except these...
const byte DPLF_VALUE = 1;  // set low pass filter 
const float compFilterAlpha = 0.9; // weight applied to gyro angle estimate
const float compFilterAlphaComplement = 1- compFilterAlpha;
const float accelAverageAlpha = 0.2; // weight applied to new accel angle calculation in complementary filter
const float accelAverageAlphaComplement = 1 - accelAverageAlpha;

// DERIVE THESE SETTINGS FROM CALIBRATION & SETUP
const int16_t GyXOffset = -419; 
const int16_t GyYOffset = 328;
const int16_t GyZOffset = 206;
const int16_t AccelXOffset = 705;   // REQUIRES DERIVING FOR NEW MPU
const int16_t AccelYOffset = -118;   // REQUIRES DERIVING FOR NEW MPU
const int16_t AccelZOffset = 1874;   // REQUIRES DERIVING FOR NEW MPU
const float accelRes = 2.0f / 32768.0f;
const float gyroRes = 250.0f / 32768.0f;


// MOTORS
const uint16_t REFRESH_INTERVAL_TICKS = 20000;

const byte pinMotor1 = 3; // front left (CW)
const byte pinMotor2 = 6; // front right (CCW)
const byte pinMotor3 = 4; // back left (CCW)
const byte pinMotor4 = 5; // back right (CW)


//PID
const int pidRateMin = -150;  // MOTOR INPUT (PULSE LENGTH)
const int pidRateMax = 150;  // MOTOR INPUT (PULSE LENGTH)
const int pidAttitudeMin = -10;  // DEG/S
const int pidAttitudeMax = 10;  // DEG/S

const byte rateLoopFreq = 1;  // works out at about 493Hz
const byte attitudeLoopFreq = 20; // works out at about 46Hz

// ADD SECTION OF DEFINES/CONSTS FOR PID


// RADIO
const byte addresses[][6] = {"1Node", "2Node"};
const bool radioNumber = 1; // receiver should be 1 (or anything not 0)

float rateMin = -70;  // DEGREES/SECOND
float rateMax = 70;  // DEGREES/SECOND
float attitudeMin = -20;  // DEGREES
float attitudeMax = 20;  // DEGREES
unsigned long heartbeatTimeout = 500;




*/
