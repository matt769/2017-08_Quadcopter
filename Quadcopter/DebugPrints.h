void printAllAngles() {
  Serial.print(currentAngles.roll); Serial.print('\t');
  Serial.print(currentAngles.pitch); Serial.print('\t');
  Serial.print(currentAngles.yaw); Serial.print('\n');
}

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


void printPidInfoPitch() {
  // plot the actual error as well?
  Serial.print(attitudePitchSettings.actual); Serial.print('\t');
  Serial.print(attitudePitchSettings.target); Serial.print('\t');
  Serial.print(attitudePitchSettings.output); Serial.print('\n');
}






