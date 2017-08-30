
// PID controller by Brett Beauregard, code can be found here
// https://github.com/br3ttb/Arduino-PID-Library/

// will need to turn off before QC has taken off

// NOTE THAT PID SETTINGS ARE DEFINED IN THE SETUP FUNCTION
// alternatively initialise as shown here: https://arduino.stackexchange.com/questions/14763/assigning-value-inside-structure-array-outside-setup-and-loop-functions

// NEED TO REVIEW THE LIMITS

// Review how PID deals with being turned on and off

int pidRateMin = -50;  // MOTOR INPUT
int pidRateMax = 50;  // MOTOR INPUT
int pidBalanceMin = -10;  // DEG/S
int pidBalanceMax = 10;  // DEG/S

byte rateLoopFreq = 9;   // remove ** ~1ms **  from desired loop time to compensate to time to run code
byte balanceLoopFreq = 45;   // remove ** ~5ms **  from desired loop time to compensate to time to run code


struct pid {
  double actual;  // these all need to be double since that's what the PID onstructor requires (although could change just for thi?)
  double output;
  double target;
  double kP;
  double kI;
  double kD;
};

struct pid rateRollSettings;
struct pid ratePitchSettings;
struct pid rateYawSettings;
struct pid balanceRollSettings;
struct pid balancePitchSettings;
struct pid balanceYawSettings;

PID pidRateRoll(&rateRollSettings.actual, &rateRollSettings.output, &rateRollSettings.target, rateRollSettings.kP, rateRollSettings.kI, rateRollSettings.kD, DIRECT);
PID pidRatePitch(&ratePitchSettings.actual, &ratePitchSettings.output, &ratePitchSettings.target, ratePitchSettings.kP, ratePitchSettings.kI, ratePitchSettings.kD, DIRECT);
PID pidRateYaw(&rateYawSettings.actual, &rateYawSettings.output, &rateYawSettings.target, rateYawSettings.kP, rateYawSettings.kI, rateYawSettings.kD, DIRECT);
PID pidBalanceRoll(&balanceRollSettings.actual, &balanceRollSettings.output, &balanceRollSettings.target, balanceRollSettings.kP, balanceRollSettings.kI, balanceRollSettings.kD, DIRECT);
PID pidBalancePitch(&balancePitchSettings.actual, &balancePitchSettings.output, &balancePitchSettings.target, balancePitchSettings.kP, balancePitchSettings.kI, balancePitchSettings.kD, DIRECT);
PID pidBalanceYaw(&balanceYawSettings.actual, &balanceYawSettings.output, &balanceYawSettings.target, balanceYawSettings.kP, balanceYawSettings.kI, balanceYawSettings.kD, DIRECT);



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

void pidBalanceModeOn() {
  pidBalanceRoll.SetMode(AUTOMATIC);
  pidBalancePitch.SetMode(AUTOMATIC);
  pidBalanceYaw.SetMode(AUTOMATIC);
}

void pidBalanceModeOff() {
  pidBalanceRoll.SetMode(MANUAL);
  pidBalancePitch.SetMode(MANUAL);
  pidBalanceYaw.SetMode(MANUAL);
}

void setupPid() {

  pidRateModeOff();
  pidBalanceModeOff();

  rateRollSettings.kP = 1;
  rateRollSettings.kI = 0;
  rateRollSettings.kD = 0;
  ratePitchSettings.kP = 1;
  ratePitchSettings.kI = 0;
  ratePitchSettings.kD = 0;
  rateYawSettings.kP = 1;
  rateYawSettings.kI = 0;
  rateYawSettings.kD = 0;

  balanceRollSettings.kP = 1;
  balanceRollSettings.kI = 0;
  balanceRollSettings.kD = 0;
  balancePitchSettings.kP = 1;
  balancePitchSettings.kI = 0;
  balancePitchSettings.kD = 0;
  balanceYawSettings.kP = 1;
  balanceYawSettings.kI = 0;
  balanceYawSettings.kD = 0;

  pidRateRoll.SetSampleTime(rateLoopFreq);
  pidRatePitch.SetSampleTime(rateLoopFreq);
  pidRateYaw.SetSampleTime(rateLoopFreq);
  pidBalanceRoll.SetSampleTime(balanceLoopFreq);
  pidBalancePitch.SetSampleTime(balanceLoopFreq);
  pidBalanceYaw.SetSampleTime(balanceLoopFreq);

  pidRateRoll.SetTunings(rateRollSettings.kP,rateRollSettings.kI,rateRollSettings.kD);
  pidRatePitch.SetTunings(ratePitchSettings.kP,ratePitchSettings.kI,ratePitchSettings.kD);
  pidRateYaw.SetTunings(rateYawSettings.kP,rateYawSettings.kI,rateYawSettings.kD);
  pidBalanceRoll.SetTunings(balanceRollSettings.kP,balanceRollSettings.kI,balanceRollSettings.kD);
  pidBalancePitch.SetTunings(balancePitchSettings.kP,balancePitchSettings.kI,balancePitchSettings.kD);
  pidBalanceYaw.SetTunings(balanceYawSettings.kP,balanceYawSettings.kI,balanceYawSettings.kD);

  pidRateRoll.SetOutputLimits(pidRateMin,pidRateMax);
  pidRatePitch.SetOutputLimits(pidRateMin,pidRateMax);
  pidRateYaw.SetOutputLimits(pidRateMin,pidRateMax);
  pidBalanceRoll.SetOutputLimits(pidBalanceMin,pidBalanceMax);
  pidBalancePitch.SetOutputLimits(pidBalanceMin,pidBalanceMax);
  pidBalanceYaw.SetOutputLimits(pidBalanceMin,pidBalanceMax);

}


void pidRateUpdate() {
//  pidRateRoll.Compute();
//  pidRatePitch.Compute();
//  pidRateYaw.Compute();

    // check that PID is returning 1 (not 0, which implies it's not eady for a new loop yet)

//  Serial.println(pidRateRoll.Compute());
//  Serial.println(pidRatePitch.Compute());
//  Serial.println(pidRateYaw.Compute());
//  Serial.println("");
}

void pidBalanceUpdate() {
  pidBalanceRoll.Compute();
  pidBalancePitch.Compute();
  pidBalanceYaw.Compute();
}


void setAutoLevelTargets(){
    balanceRollSettings.target = 0;
    balancePitchSettings.target = 0;
    balanceYawSettings.target = 0;
}





