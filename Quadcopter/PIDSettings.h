uint16_t loopCounterPidRate;
uint16_t loopCounterPidAttitude;

struct pid {
  float actual;
  float output;
  float target;
  float kP;
  float kI;
  float kD;
};

struct pid rateRollSettings;
struct pid ratePitchSettings;
struct pid rateYawSettings;
struct pid attitudeRollSettings;
struct pid attitudePitchSettings;
struct pid attitudeYawSettings;

PID pidRateRoll(&rateRollSettings.actual, &rateRollSettings.output, &rateRollSettings.target, rateRollSettings.kP, rateRollSettings.kI, rateRollSettings.kD, DIRECT, mainLoopFreqMillis);
PID pidRatePitch(&ratePitchSettings.actual, &ratePitchSettings.output, &ratePitchSettings.target, ratePitchSettings.kP, ratePitchSettings.kI, ratePitchSettings.kD, DIRECT, mainLoopFreqMillis);
PID pidRateYaw(&rateYawSettings.actual, &rateYawSettings.output, &rateYawSettings.target, rateYawSettings.kP, rateYawSettings.kI, rateYawSettings.kD, DIRECT, mainLoopFreqMillis);
PID pidAttitudeRoll(&attitudeRollSettings.actual, &attitudeRollSettings.output, &attitudeRollSettings.target, attitudeRollSettings.kP, attitudeRollSettings.kI, attitudeRollSettings.kD, DIRECT, mainLoopFreqMillis);
PID pidAttitudePitch(&attitudePitchSettings.actual, &attitudePitchSettings.output, &attitudePitchSettings.target, attitudePitchSettings.kP, attitudePitchSettings.kI, attitudePitchSettings.kD, DIRECT, mainLoopFreqMillis);
PID pidAttitudeYaw(&attitudeYawSettings.actual, &attitudeYawSettings.output, &attitudeYawSettings.target, attitudeYawSettings.kP, attitudeYawSettings.kI, attitudeYawSettings.kD, DIRECT, mainLoopFreqMillis);

void pidRateModeOn() {
  pidRateRoll.SetMode(AUTOMATIC);
  pidRatePitch.SetMode(AUTOMATIC);
  pidRateYaw.SetMode(AUTOMATIC);
}

void pidRateModeOff() {
  pidRateRoll.SetMode(MANUAL);
  pidRatePitch.SetMode(MANUAL);
  pidRateYaw.SetMode(MANUAL);
}

void pidAttitudeModeOn() {
  pidAttitudeRoll.SetMode(AUTOMATIC);
  pidAttitudePitch.SetMode(AUTOMATIC);
  pidAttitudeYaw.SetMode(AUTOMATIC);
}

void pidAttitudeModeOff() {
  pidAttitudeRoll.SetMode(MANUAL);
  pidAttitudePitch.SetMode(MANUAL);
  pidAttitudeYaw.SetMode(MANUAL);
}

void setupPid() {

  pidRateModeOff();
  pidAttitudeModeOff();

  rateRollSettings.kP = rateRollKp;
  rateRollSettings.kI = rateRollKi;
  rateRollSettings.kD = rateRollKd;
  ratePitchSettings.kP = ratePitchKp;
  ratePitchSettings.kI = ratePitchKi;
  ratePitchSettings.kD = ratePitchKd;
  rateYawSettings.kP = rateYawKp;
  rateYawSettings.kI = rateYawKi;
  rateYawSettings.kD = rateYawKd;

  attitudeRollSettings.kP = attitudeRollKp;
  attitudeRollSettings.kI = attitudeRollKi;
  attitudeRollSettings.kD = attitudeRollKd;
  attitudePitchSettings.kP = attitudePitchKp;
  attitudePitchSettings.kI = attitudePitchKi;
  attitudePitchSettings.kD = attitudePitchKd;
  attitudeYawSettings.kP = attitudeYawKp;
  attitudeYawSettings.kI = attitudeYawKi;
  attitudeYawSettings.kD = attitudeYawKd;

  pidRateRoll.SetSampleTime(mainLoopFreqMillis);
  pidRatePitch.SetSampleTime(mainLoopFreqMillis);
  pidRateYaw.SetSampleTime(mainLoopFreqMillis);
  pidAttitudeRoll.SetSampleTime(mainLoopFreqMillis);
  pidAttitudePitch.SetSampleTime(mainLoopFreqMillis);
  pidAttitudeYaw.SetSampleTime(mainLoopFreqMillis);

  pidRateRoll.SetTunings(rateRollSettings.kP, rateRollSettings.kI, rateRollSettings.kD);
  pidRatePitch.SetTunings(ratePitchSettings.kP, ratePitchSettings.kI, ratePitchSettings.kD);
  pidRateYaw.SetTunings(rateYawSettings.kP, rateYawSettings.kI, rateYawSettings.kD);
  pidAttitudeRoll.SetTunings(attitudeRollSettings.kP, attitudeRollSettings.kI, attitudeRollSettings.kD);
  pidAttitudePitch.SetTunings(attitudePitchSettings.kP, attitudePitchSettings.kI, attitudePitchSettings.kD);
  pidAttitudeYaw.SetTunings(attitudeYawSettings.kP, attitudeYawSettings.kI, attitudeYawSettings.kD);

  pidRateRoll.SetOutputLimits(pidRateMin, pidRateMax);
  pidRatePitch.SetOutputLimits(pidRateMin, pidRateMax);
  pidRateYaw.SetOutputLimits(pidRateMin, pidRateMax);
  pidAttitudeRoll.SetOutputLimits(pidAttitudeMin, pidAttitudeMax);
  pidAttitudePitch.SetOutputLimits(pidAttitudeMin, pidAttitudeMax);
  pidAttitudeYaw.SetOutputLimits(pidAttitudeMin, pidAttitudeMax);
}

void pidRateUpdate() {
    pidRateRoll.Compute(false);
    pidRatePitch.Compute(false);
    pidRateYaw.Compute(false);
}

void pidAttitudeUpdate() {
    pidAttitudeRoll.Compute();
    pidAttitudePitch.Compute();
    pidAttitudeYaw.Compute();
}

void setAutoLevelTargets() {
  attitudeRollSettings.target = 0;
  attitudePitchSettings.target = 0;
  attitudeYawSettings.target = 0;
}

// will either be set directly by (mapped) receiver input, or output from the attitude PID
void setRatePidTargets(float roll, float pitch, float yaw) {
  rateRollSettings.target = roll;
  ratePitchSettings.target = pitch;
  rateYawSettings.target = yaw;
}

void setRatePidActual(float roll, float pitch, float yaw) {
  rateRollSettings.actual = roll;
  ratePitchSettings.actual = pitch;
  rateYawSettings.actual = yaw;
}

void setAttitudePidActual(float roll, float pitch, float yaw) {
  attitudeRollSettings.actual = roll;
  attitudePitchSettings.actual = pitch;
  attitudeYawSettings.actual = yaw;
}

void connectionLostDescend(int *throttle, float accelZ) {
//  Serial.println(*throttle);
  if (accelZ > 0.95) {    // accelZ < 1 implies downwards movement, reduce throttle until I get it
    *throttle -= 1;
  }
}

void overrideYawTarget() {
//  rateYawSettings.target = 0;
  // replace with what the rate target would have been
  rateYawSettings.target = (float)map(rcPackage.yaw+1, 0,255, rateMax, rateMin);
}

