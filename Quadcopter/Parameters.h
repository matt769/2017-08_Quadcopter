// CONTROL LOOP FREQUENCY
// all frequencies expressed in loop duration in milliseconds e.g. 100Hz = 1000/100 = 10ms
const uint16_t rateLoopFreq = 1250;  // 1 -> 1kHz
const uint16_t attitudeLoopFreq = 5000; // 5 -> 200Hz
const uint16_t ratePIDFreq = attitudeLoopFreq / 1000;
const uint16_t attitudePIDFreq = attitudeLoopFreq / 1000;
const uint16_t receiverFreq = 50;
const uint16_t batteryFreq = 1000;

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
const int THROTTLE_LIMIT = 1500; // currently have no need of more power than this
const int ZERO_THROTTLE = 1000;
const int THROTTLE_MIN_SPIN = 1125;


// RADIO


// MOTION
const byte DPLF_VALUE = 3;  // set low pass filter
const byte FS_SEL = 0;  // gyro full scale range +/-250deg/s
const byte AFS_SEL = 2;  // accel full scale range +/-8g
const float compFilterAlpha = 0.99f; // weight applied to gyro angle estimate
const float accelAverageAlpha = 0.1f; // weight applied to new accel angle calculation in complementary filter




// BATTERY

