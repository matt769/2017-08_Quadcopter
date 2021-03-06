// CONTROL LOOP FREQUENCY
const unsigned long receiverFreq = 50; // expressed in loop duration in milliseconds
const unsigned long batteryFreq = 1000; // expressed in loop duration in milliseconds
const unsigned long mainLoopFreq = 5000;  // expressed in loop duration in MICROseconds // 1250 -> 800Hz
const unsigned long mainLoopFreqMillis = mainLoopFreq / 1000;  // PID class takes times in millis
const unsigned long gyroLoopFreq = 1250;  // expressed in loop duration in MICROseconds // 1250 -> 800Hz
const unsigned long magLoopFreq = 20; // expressed in loop duration in milliseconds

// PID OUTPUT LIMITS
const int pidRateMin = -150;  // MOTOR INPUT (PULSE LENGTH)
const int pidRateMax = 150;  // MOTOR INPUT (PULSE LENGTH)
const int pidAttitudeMin = -100;  // DEG/S
const int pidAttitudeMax = 100;  // DEG/S

// PID GAINS
const float rateRollKp = 1.2;
const float rateRollKi = 0.0;
const float rateRollKd = 0.0025; // 0.0025
const float ratePitchKp = 1.2;
const float ratePitchKi = 0.0;
const float ratePitchKd = 0.0025; // 0.0025
const float rateYawKp = 1.0;
const float rateYawKi = 0.0;
const float rateYawKd = 0.0;

const float attitudeRollKp = 3.5;
const float attitudeRollKi = 0.5;
const float attitudeRollKd = 0.008; // 0.001
const float attitudePitchKp = 3.5;
const float attitudePitchKi = 0.5;
const float attitudePitchKd = 0.008; // 0.001
const float attitudeYawKp = 1.5;
const float attitudeYawKi = 0.3;
const float attitudeYawKd = 0.0;

// MOTORS
const int THROTTLE_LIMIT = 1600; // currently have no need of more power than this
const int ZERO_THROTTLE = 1000;
const int THROTTLE_MIN_SPIN = 1125;


// RADIO


// MOTION
const byte DPLF_VALUE = 3;  // set low pass filter
const byte FS_SEL = 2;  // 0 = gyro full scale range +/-250deg/s
const byte AFS_SEL = 2;  // 2 = accel full scale range +/-8g
const float compFilterAlpha = 0.998f; // weight applied to gyro angle estimate
const float accelAverageAlpha = 0.2f; // weight given to the new reading over the running average


// BATTERY



// STATUS LED
const byte pinStatusLed = 8;







