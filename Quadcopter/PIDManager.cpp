#include "PIDManager.h"
#include <Arduino.h>
#include "PID.h"
#include "Parameters.h"

struct pid rateRollSettings;
struct pid ratePitchSettings;
struct pid rateYawSettings;
struct pid attitudeRollSettings;
struct pid attitudePitchSettings;
struct pid attitudeYawSettings;

PID pidRateRoll(&rateRollSettings.actual, &rateRollSettings.output, &rateRollSettings.target, rateRollSettings.kP, rateRollSettings.kI, rateRollSettings.kD, DIRECT);
PID pidRatePitch(&ratePitchSettings.actual, &ratePitchSettings.output, &ratePitchSettings.target, ratePitchSettings.kP, ratePitchSettings.kI, ratePitchSettings.kD, DIRECT);
PID pidRateYaw(&rateYawSettings.actual, &rateYawSettings.output, &rateYawSettings.target, rateYawSettings.kP, rateYawSettings.kI, rateYawSettings.kD, REVERSE);
PID pidAttitudeRoll(&attitudeRollSettings.actual, &attitudeRollSettings.output, &attitudeRollSettings.target, attitudeRollSettings.kP, attitudeRollSettings.kI, attitudeRollSettings.kD, DIRECT);
PID pidAttitudePitch(&attitudePitchSettings.actual, &attitudePitchSettings.output, &attitudePitchSettings.target, attitudePitchSettings.kP, attitudePitchSettings.kI, attitudePitchSettings.kD, DIRECT);
PID pidAttitudeYaw(&attitudeYawSettings.actual, &attitudeYawSettings.output, &attitudeYawSettings.target, attitudeYawSettings.kP, attitudeYawSettings.kI, attitudeYawSettings.kD, REVERSE);

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
  rateRollSettings.kD = rateRollKd; // 0.0025
  ratePitchSettings.kP = ratePitchKp;
  ratePitchSettings.kI = ratePitchKi;
  ratePitchSettings.kD = ratePitchKd; // 0.0025
  rateYawSettings.kP = rateYawKp;
  rateYawSettings.kI = rateYawKi;
  rateYawSettings.kD = rateYawKd;

  attitudeRollSettings.kP = attitudeRollKp;
  attitudeRollSettings.kI = attitudeRollKi;
  attitudeRollSettings.kD = attitudeRollKd; // 0.001
  attitudePitchSettings.kP = attitudePitchKp;
  attitudePitchSettings.kI = attitudePitchKi;
  attitudePitchSettings.kD = attitudePitchKd; // 0.001
  attitudeYawSettings.kP = attitudeYawKp;
  attitudeYawSettings.kI = attitudeYawKi;
  attitudeYawSettings.kD = attitudeYawKd;

  pidRateRoll.SetSampleTime(ratePIDFreq);
  pidRatePitch.SetSampleTime(ratePIDFreq);
  pidRateYaw.SetSampleTime(ratePIDFreq);
  pidAttitudeRoll.SetSampleTime(attitudePIDFreq);
  pidAttitudePitch.SetSampleTime(attitudePIDFreq);
  pidAttitudeYaw.SetSampleTime(attitudePIDFreq);

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

// return true if all PIDs are run
bool pidRateUpdate() {
  static unsigned long thisTime;
  static unsigned long lastTime;
  static unsigned long interval;
  thisTime = millis();
  interval = thisTime - lastTime; // saving value of interval in case I want to send to PID calculation
  if (interval >= ratePIDFreq) {
    lastTime += ratePIDFreq;
    pidRateRoll.Compute();
    pidRatePitch.Compute();
    pidRateYaw.Compute();
    return true;
  }
  return false;
}

// return true if all PIDs are run
bool pidAttitudeUpdate() {
  static unsigned long thisTime;
  static unsigned long lastTime;
  static unsigned long interval;
  thisTime = millis();
  interval = thisTime - lastTime;
  if (interval >= attitudePIDFreq) {
    lastTime += attitudePIDFreq;
    pidAttitudeRoll.Compute();
    pidAttitudePitch.Compute();
    pidAttitudeYaw.Compute();
    return true;
  }
  return false;
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

void connectionLostDescend(int *throttle, float ZAccel) {
//  Serial.println(*throttle);
  if (ZAccel > 0.95) {    // ZAccel < 1 implies downwards movement, reduce throttle until I get it
    *throttle -= 1;
  }
  if (*throttle < 1050) {
//    setMotorsLow(); // change to modify global KILL flag
    digitalWrite(8, HIGH);
    while (1);
  }
}


void overrideYawTarget() {
  rateYawSettings.target = 0;
}

