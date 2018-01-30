// ****************************************************************************************
// PID Library by Brett Beauregard
//    with some modifications - mainly that the time interval is checked outside the functions here
// https://github.com/br3ttb/Arduino-PID-Library
// ****************************************************************************************


class PID
{
  public:
    //Constants used in some of the functions below
#define AUTOMATIC  1
#define MANUAL  0
#define DIRECT  0
#define REVERSE  1

    //commonly used functions **************************************************************************
    PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection)
    {
      myOutput = Output;
      myInput = Input;
      mySetpoint = Setpoint;
      inAuto = false;
      SetControllerDirection(ControllerDirection);
      SetTunings(Kp, Ki, Kd);
    }

    void SetMode(int Mode)
    {
      bool newAuto = (Mode == AUTOMATIC);
      if (newAuto && !inAuto)
      { /*we just went from manual to auto*/
        PID::Initialize();
      }
      inAuto = newAuto;
    }

    void Compute(bool allTerms=true)
    {
      float input = *myInput;
      float error = *mySetpoint - input;
      if (allTerms) {
        ITerm += (ki * error);
        if (ITerm > outMax) ITerm = outMax;
        else if (ITerm < outMin) ITerm = outMin;
      }
      float dInput = (input - lastInput);
      /*Compute PID Output*/
      float output;
      if (allTerms) output = kp * error + ITerm - kd * dInput;
      else output = kp * error - kd * dInput;
      
      if (output > outMax) output = outMax;
      else if (output < outMin) output = outMin;
      *myOutput = output;
      /*Remember some variables for next time*/
      lastInput = input;
    }

    void SetOutputLimits(float Min, float Max)
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

    void SetTunings(float Kp, float Ki, float Kd)
    {
      if (Kp < 0 || Ki < 0 || Kd < 0) return;

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

    void SetControllerDirection(int Direction)
    {
      if (inAuto && Direction != controllerDirection)
      {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
      }
      controllerDirection = Direction;
    }

    void SetSampleTime(int NewSampleTime)
    {
      if (NewSampleTime > 0)
      {
        float ratio  = (float)NewSampleTime
                       / (float)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
      }
    }


  private:
    void Initialize()
    {
      ITerm = *myOutput;
      lastInput = *myInput;
      if (ITerm > outMax) ITerm = outMax;
      else if (ITerm < outMin) ITerm = outMin;
    }
    float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter
    int controllerDirection;
    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
    //   what these values are.  with pointers we'll just know.
    float ITerm, lastInput;
    unsigned long SampleTime;
    float outMin, outMax;
    bool inAuto;
};
