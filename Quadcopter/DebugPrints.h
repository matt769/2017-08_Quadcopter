
void printAnglesAllSourcesRoll() {
  Serial.print(currentAngles.roll); Serial.print('\t');
  Serial.print(accelAngles.roll); Serial.print('\t');
  Serial.print(gyroAngles.roll); Serial.print('\n');
}

void printAnglesAllSourcesPitch() {
  Serial.print(currentAngles.pitch); Serial.print('\t');
  Serial.print(accelAngles.pitch); Serial.print('\t');
  Serial.print(gyroAngles.pitch); Serial.print('\n');
}

void printMotorPulsesBlock() {
  Serial.print(motor1pulse); Serial.print('\t');
  Serial.print(motor2pulse); Serial.print('\n');
  Serial.print(motor3pulse); Serial.print('\t');
  Serial.print(motor4pulse); Serial.print('\n');
  Serial.print('\n');
}
void printMotorPulsesLine() {
  Serial.print(motor1pulse); Serial.print('\t');
  Serial.print(motor2pulse); Serial.print('\t');
  Serial.print(motor3pulse); Serial.print('\t');
  Serial.print(motor4pulse); Serial.print('\n');
}

void printRatePIDOutputs() {
  Serial.print(rateRollSettings.output); Serial.print('\t');
  Serial.print(ratePitchSettings.output); Serial.print('\t');
  Serial.print(rateYawSettings.output); Serial.print('\n');
}


int loopCounter = 0;
int loopCounterRx = 0;
int loopCounterRate = 0;
int loopCounterAttitude = 0;

void printLoopTimes() {
  Serial.print(loopCounter); Serial.print('\t');
  loopCounter = 0;
  Serial.print(loopCounterRx); Serial.print('\t');
  loopCounterRx = 0;
  Serial.print(loopCounterRate); Serial.print('\t');
  loopCounterRate = 0;
  Serial.print(loopCounterAttitude); Serial.print('\t');
  loopCounterAttitude = 0;
  Serial.print(loopCounterPidRate); Serial.print('\t');
  loopCounterPidRate = 0;
  Serial.print(loopCounterPidAttitude); Serial.print('\t');
  loopCounterPidAttitude = 0;
  Serial.print(loopCounterMotorRecalc); Serial.print('\t');
  loopCounterMotorRecalc = 0;
  Serial.print(loopCounterMotorUpdate); Serial.print('\t');
  loopCounterMotorUpdate = 0;
  Serial.print(loopCounterMotorPulse); Serial.print('\n');
  loopCounterMotorPulse = 0;
}

void printPidInfoPitch() {
  // plot the actual error as well?
  Serial.print(attitudePitchSettings.actual); Serial.print('\t');
  Serial.print(attitudePitchSettings.target); Serial.print('\t');
  Serial.print(attitudePitchSettings.output); Serial.print('\n');
}






