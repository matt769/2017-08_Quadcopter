#include "PID.h"
#include <Arduino.h>


PID::PID(float* Input, float* Output, float* Setpoint,
         float Kp, float Ki, float Kd, int ControllerDirection)
{
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;
  SetControllerDirection(ControllerDirection);
  SetTunings(Kp, Ki, Kd);
  lastTime = millis() - SampleTime;
}

void PID::SetMode(int Mode)
{
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto)
  { /*we just went from manual to auto*/
    PID::Initialize();
  }
  inAuto = newAuto;
}

bool PID::Compute()
{
  if (!inAuto) return false;

  float input = *myInput;
  float error = *mySetpoint - input;
  ITerm += (ki * error);
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
  float dInput = (input - lastInput);

  float output = kp * error + ITerm - kd * dInput;

  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;
  *myOutput = output;

  lastInput = input;
  return true;
}

void PID::SetOutputLimits(float Min, float Max)
{
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (inAuto)
  {
    if (*myOutput > outMax) *myOutput = outMax;
    else if (*myOutput < outMin) *myOutput = outMin;

    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;
  }
}

void PID::SetTunings(float Kp, float Ki, float Kd)
{
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  dispKp = Kp; dispKi = Ki; dispKd = Kd;

  float SampleTimeInSec = ((float)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;

  if (controllerDirection == REVERSE)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

void PID::SetControllerDirection(int Direction)
{
  if (inAuto && Direction != controllerDirection)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
  controllerDirection = Direction;
}

void PID::SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    float ratio  = (float)NewSampleTime / (float)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

void PID::Initialize()
{
  ITerm = *myOutput;
  lastInput = *myInput;
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
}


