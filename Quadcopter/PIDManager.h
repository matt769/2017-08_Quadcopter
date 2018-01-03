#ifndef PIDMANAGER_H
#define PIDMANAGER_H

struct pid {
  float actual;
  float output;
  float target;
  float kP;
  float kI;
  float kD;
};

void pidRateModeOn();
void pidRateModeOff();
void pidAttitudeModeOn();
void pidAttitudeModeOff();
void setupPid();
bool pidRateUpdate();
bool pidAttitudeUpdate();
void setAutoLevelTargets(); // should this actually be elsewhere?
void setRatePidTargets(float roll, float pitch, float yaw);
void setRatePidActual(float roll, float pitch, float yaw);
void setAttitudePidActual(float roll, float pitch, float yaw);
void connectionLostDescend(int *throttle, float ZAccel);
void overrideYawTarget();

#endif
