// ****************************************************************************************
// Based on PID Library by Brett Beauregard
// https://github.com/br3ttb/Arduino-PID-Library
// ****************************************************************************************

#ifndef PID_H
#define PID_H

class PID
{
  public:
#define AUTOMATIC  1
#define MANUAL  0
#define DIRECT  0
#define REVERSE  1
    PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection);
    void SetMode(int Mode);
    bool Compute();
    void SetOutputLimits(float Min, float Max);
    void SetTunings(float Kp, float Ki, float Kd);
    void SetControllerDirection(int Direction);
    void SetSampleTime(int NewSampleTime);

  private:
    void Initialize();
    float dispKp;       // * we'll hold on to the tuning parameters in user-entered
    float dispKi;       //   format for display purposes
    float dispKd;       //
    float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter
    int controllerDirection;
    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
    unsigned long lastTime;
    float ITerm, lastInput;
    unsigned long SampleTime;
    float outMin, outMax;
    bool inAuto;
};

#endif
