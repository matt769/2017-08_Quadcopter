
// PID controller by Brett Beauregard, code can be found here
// https://github.com/br3ttb/Arduino-PID-Library/

// will need to turn off before QC has taken off


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

PID pidRateRoll(&rateRollSettings.actual, &rateRollSettings.output, &rateRollSettings.target, rateRollSettings.kP, rateRollSettings.kI, rateRollSettings.kD, REVERSE);
PID pidRatePitch(&ratePitchSettings.actual, &ratePitchSettings.output, &ratePitchSettings.target, ratePitchSettings.kP, ratePitchSettings.kI, ratePitchSettings.kD, REVERSE);
PID pidRateYaw(&rateYawSettings.actual, &rateYawSettings.output, &rateYawSettings.target, rateYawSettings.kP, rateYawSettings.kI, rateYawSettings.kD, REVERSE);
PID pidBalanceRoll(&balanceRollSettings.actual, &balanceRollSettings.output, &balanceRollSettings.target, balanceRollSettings.kP, balanceRollSettings.kI, balanceRollSettings.kD, REVERSE);
PID pidBalancePitch(&balancePitchSettings.actual, &balancePitchSettings.output, &balancePitchSettings.target, balancePitchSettings.kP, balancePitchSettings.kI, balancePitchSettings.kD, REVERSE);
PID pidBalanceYaw(&balanceYawSettings.actual, &balanceYawSettings.output, &balanceYawSettings.target, balanceYawSettings.kP, balanceYawSettings.kI, balanceYawSettings.kD, REVERSE);


void pidSetup() {

  rateRollSettings.kP = 1;
  rateRollSettings.kI = 0;
  rateRollSettings.kD = 0;
  ratePitchSettings.kP = 1;
  ratePitchSettings.kI = 0;
  ratePitchSettings.kD = 0;
  rateYawSettings.kP = 1;
  rateYawSettings.kI = 0;
  rateYawSettings.kD = 0;

  rateRollSettings.kP = 1;
  rateRollSettings.kI = 0;
  rateRollSettings.kD = 0;
  ratePitchSettings.kP = 1;
  ratePitchSettings.kI = 0;
  ratePitchSettings.kD = 0;
  rateYawSettings.kP = 1;
  rateYawSettings.kI = 0;
  rateYawSettings.kD = 0;

  // for now, deal with limits elsewhere  // PID.SetOutputLimits(a, b);
  // also handle sample time elsewhere, but this can stay here for now
  pidRateRoll.SetSampleTime(20);
  pidRatePitch.SetSampleTime(20);
  pidRateYaw.SetSampleTime(20);
  pidBalanceRoll.SetSampleTime(50);
  pidBalancePitch.SetSampleTime(50);
  pidBalanceYaw.SetSampleTime(50);

}

void pidRateModeOn() {
  pidRateRoll.SetMode(AUTOMATIC);
  pidRatePitch.SetMode(AUTOMATIC);
  pidRateYaw.SetMode(AUTOMATIC);
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

void pidRateUpdate() {
  pidRateRoll.Compute();
  pidRatePitch.Compute();
  pidRateYaw.Compute();
}

void pidBalanceUpdate() {
  pidBalanceRoll.Compute();
  pidBalancePitch.Compute();
  pidBalanceYaw.Compute();
}

