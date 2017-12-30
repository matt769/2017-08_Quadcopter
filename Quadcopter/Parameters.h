



// CONTROL LOOP FREQUENCY
const byte rateLoopFreq = 1;  // 1kHz
const byte attitudeLoopFreq = 5; // 200Hz
const byte ratePIDFreq = attitudeLoopFreq;
const byte attitudePIDFreq = attitudeLoopFreq;



// PID OUTPUT LIMITS
const int pidRateMin = -150;  // MOTOR INPUT (PULSE LENGTH)
const int pidRateMax = 150;  // MOTOR INPUT (PULSE LENGTH)
const int pidAttitudeMin = -100;  // DEG/S
const int pidAttitudeMax = 100;  // DEG/S

// PID GAINS
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



// RADIO




// MOTORS




// MOTION




// BATTERY

