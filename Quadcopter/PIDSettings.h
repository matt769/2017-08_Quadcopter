
// PID controller by Brett Beauregard, code can be found here
// https://github.com/br3ttb/Arduino-PID-Library/

// will need to turn off before QC has taken off

// NOTE THAT PID SETTINGS ARE DEFINED IN THE SETUP FUNCTION
// alternatively initialise as shown here: https://arduino.stackexchange.com/questions/14763/assigning-value-inside-structure-array-outside-setup-and-loop-functions

// NEED TO REVIEW THE LIMITS

// Review how PID deals with being turned on and off

int pidRateMin = -100;  // MOTOR INPUT (PULSE LENGTH)
int pidRateMax = 100;  // MOTOR INPUT (PULSE LENGTH)
int pidAttitudeMin = -10;  // DEG/S
int pidAttitudeMax = 10;  // DEG/S

byte rateLoopFreq = 4;  // works out at about 200Hz
byte attitudeLoopFreq = 20; // 20Hz


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

PID pidRateRoll(&rateRollSettings.actual, &rateRollSettings.output, &rateRollSettings.target, rateRollSettings.kP, rateRollSettings.kI, rateRollSettings.kD, DIRECT);
PID pidRatePitch(&ratePitchSettings.actual, &ratePitchSettings.output, &ratePitchSettings.target, ratePitchSettings.kP, ratePitchSettings.kI, ratePitchSettings.kD, DIRECT);
PID pidRateYaw(&rateYawSettings.actual, &rateYawSettings.output, &rateYawSettings.target, rateYawSettings.kP, rateYawSettings.kI, rateYawSettings.kD, DIRECT);
PID pidAttitudeRoll(&attitudeRollSettings.actual, &attitudeRollSettings.output, &attitudeRollSettings.target, attitudeRollSettings.kP, attitudeRollSettings.kI, attitudeRollSettings.kD, DIRECT);
PID pidAttitudePitch(&attitudePitchSettings.actual, &attitudePitchSettings.output, &attitudePitchSettings.target, attitudePitchSettings.kP, attitudePitchSettings.kI, attitudePitchSettings.kD, DIRECT);
PID pidAttitudeYaw(&attitudeYawSettings.actual, &attitudeYawSettings.output, &attitudeYawSettings.target, attitudeYawSettings.kP, attitudeYawSettings.kI, attitudeYawSettings.kD, DIRECT);



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

  rateRollSettings.kP = 10;
  rateRollSettings.kI = 0;
  rateRollSettings.kD = 0;
  ratePitchSettings.kP = 10;
  ratePitchSettings.kI = 0;
  ratePitchSettings.kD = 0;
  rateYawSettings.kP = 10;
  rateYawSettings.kI = 0;
  rateYawSettings.kD = 0;

  attitudeRollSettings.kP = 1;
  attitudeRollSettings.kI = 0;
  attitudeRollSettings.kD = 0;
  attitudePitchSettings.kP = 1;
  attitudePitchSettings.kI = 0;
  attitudePitchSettings.kD = 0;
  attitudeYawSettings.kP = 1;
  attitudeYawSettings.kI = 0;
  attitudeYawSettings.kD = 0;

  pidRateRoll.SetSampleTime(rateLoopFreq);
  pidRatePitch.SetSampleTime(rateLoopFreq);
  pidRateYaw.SetSampleTime(rateLoopFreq);
  pidAttitudeRoll.SetSampleTime(attitudeLoopFreq);
  pidAttitudePitch.SetSampleTime(attitudeLoopFreq);
  pidAttitudeYaw.SetSampleTime(attitudeLoopFreq);

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
  pidRateRoll.Compute();
  pidRatePitch.Compute();
  pidRateYaw.Compute();

  // check that PID is returning 1 (not 0, which implies it's not eady for a new loop yet)

  //  Serial.println(pidRateRoll.Compute());
  //  Serial.println(pidRatePitch.Compute());
  //  Serial.println(pidRateYaw.Compute());
  //  Serial.println("");
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

//void setAttitudePidTargets(float *roll, float *pitch, float *yaw){
//    attitudeRollSettings.target = *roll;
//    attitudePitchSettings.target = *pitch;
//    attitudeYawSettings.target = *yaw;
//}

void setRatePidTargets(float *roll, float *pitch, float *yaw) {
  rateRollSettings.target = *roll;
  ratePitchSettings.target = *pitch;
  rateYawSettings.target = *yaw;
}

void setRatePidActual(float *roll, float *pitch, float *yaw) {
  rateRollSettings.actual = *roll;
  ratePitchSettings.actual = *pitch;
  rateYawSettings.actual = *yaw;
}


void setAttitudePidActual(float *roll, float *pitch, float *yaw) {
  attitudeRollSettings.actual = *roll;
  attitudePitchSettings.actual = *pitch;
  attitudeYawSettings.actual = *yaw;
}


// really this should only act if there's no user input
// and if we regain user input and there's a difference, perhaps should try and move more slowly towards it
// also review these thresholds and increments
// change this to be full PID
//void connectionLostDescend(int *throttle, float *ZAccel) {
//  if (*ZAccel > 1.05) {
//    *throttle -= 5;
//  }
//  else if (*ZAccel < 0.95) {
//    *throttle += 5;
//  }
//}

void connectionLostDescend(int *throttle, float *ZAccel) {
  if (*ZAccel < 1.05) {    // ZAccel > 1 implies downwards movement, reduce throttle until I get it
    *throttle -= 1;
  }
}


void overrideYawTarget(){
  rateYawSettings.target = 0;
}



