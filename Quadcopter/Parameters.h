#ifndef PARAMETERS_H
#define PARAMETERS_H

// IMU
const float compFilterAlpha = 0.99f;
const float accelAverageAlpha = 0.05f;
const float offsetAccelScale[3] = { 0.01129227174, -0.00323063182, -0.11709311610};
const float offsetGyroScale[3] = { -0.02385017929, 0.00375586283, 0.00117846130};
const float offsetAccelIntercept[3] = { 875.974694, 34.84791487, 17830.3859};
const float offsetGyroIntercept[3] = {  -557.7712577, 342.0514029, 207.8547826};

// ADD THE GYRO/ACCEL SCALE RANGES HERE ? OR WITH MPU LIB?
// Here for now
// THIS NEEDS TO CHANGE, MPU6050 LIB CURRENTLY USES DEFAULT BUT THEY NEED TO BE TIED TOGETHER PROPERLY
const uint8_t accelFullScaleRange = 2;  // note this is the required register value, not the actual range value
const uint8_t gyroFullScaleRange = 0;
const float MICROS_TO_SECONDS = 0.000001; // consider moving somewhere else


// PID
const int pidRateMin = -150;  // MOTOR INPUT (PULSE LENGTH)
const int pidRateMax = 150;  // MOTOR INPUT (PULSE LENGTH)
const int pidAttitudeMin = -100;  // DEG/S
const int pidAttitudeMax = 100;  // DEG/S

const byte rateLoopFreq = 1;  // 1kHz
const byte attitudeLoopFreq = 5; // 200Hz

const byte ratePIDFreq = attitudeLoopFreq;  // 5ms <=> 200Hz 
const byte attitudePIDFreq = attitudeLoopFreq;


const float rateRollKp = 1.2;
const float rateRollKi = 0;
const float rateRollKd = 0.0025; // 0.0025
const float ratePitchKp = 1.2;
const float ratePitchKi = 0;
const float ratePitchKd = 0.0025; // 0.0025
const float rateYawKp = 1.0;
const float rateYawKi = 0;
const float rateYawKd = 0;

const float attitudeRollKp = 4.0;
const float attitudeRollKi = 0.0;
const float attitudeRollKd = 0.001; // 0.001
const float attitudePitchKp = 4.0;
const float attitudePitchKi = 0.0;
const float attitudePitchKd = 0.001; // 0.001
const float attitudeYawKp = 0;
const float attitudeYawKi = 0;
const float attitudeYawKd = 0;

// MOTORS
const uint16_t CYCLE_TICKS = 5000;
const uint16_t PULSE_GAP = 100;  // gap between starting pulses, in ticks
const uint16_t escTicksStart[4] = {PULSE_GAP, PULSE_GAP * 2, PULSE_GAP * 3, PULSE_GAP * 4};
// If changing pins, will also need to change the port and bit masks in generateEscPulses function
const uint8_t motorPins[4] = {3,6,4,5}; // front left (CW) // front right (CCW) // back left (CCW) // back right (CW)
const uint16_t THROTTLE_LIMIT = 1500;
const uint16_t ZERO_THROTTLE = 1000;
const uint16_t THROTTLE_MIN_SPIN = 1125;

// RECEIVER
const byte address[6] = "1Node";
const byte pipeNumber = 1;
const float rateMin = -120;  // DEGREES/SECOND
const float rateMax = 120;  // DEGREES/SECOND
const float attitudeMin = -30;  // DEGREES
const float attitudeMax = 30;  // DEGREES
const unsigned long heartbeatTimeout = 500;
const byte radio_ce_pin = 9;
const byte radio_csn_pin = 10; // SPI slave select
const uint8_t receiverFreq = 50;



// BATTERY MONITOR
const int batteryFreq = 1000;
const byte pinStatusLed = 8;



#endif
