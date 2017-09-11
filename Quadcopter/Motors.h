// actually the ESCs really

Servo motor1; // front left (CW)
Servo motor2; // front right (CCW)
Servo motor3; // back left (CCW)
Servo motor4; // back right (CW)

byte pinMotor1 = 31; // UPDATE
byte pinMotor2 = 33; // UPDATE
byte pinMotor3 = 35; // UPDATE
byte pinMotor4 = 37; // UPDATE

int motor1pulse;
int motor2pulse;
int motor3pulse;
int motor4pulse;

const int ZERO_THROTTLE = 1000;

void setupMotors() {
  motor1.writeMicroseconds(1500);
  motor1.attach(pinMotor1);
  motor2.writeMicroseconds(1500);
  motor2.attach(pinMotor2);
  motor3.writeMicroseconds(1500);
  motor3.attach(pinMotor3);
  motor4.writeMicroseconds(1500);
  motor4.attach(pinMotor4);
}

void calculateMotorInput(int *throttle, float *rollOffset, float *pitchOffset, float *yawOffset) {
  motor1pulse = *throttle + *rollOffset - *pitchOffset + *yawOffset;
  motor2pulse = *throttle - *rollOffset - *pitchOffset - *yawOffset;
  motor3pulse = *throttle + *rollOffset + *pitchOffset - *yawOffset;
  motor4pulse = *throttle - *rollOffset + *pitchOffset + *yawOffset;
}

// alternatively I could just cap throttle by pidRateMax*3
void capMotorInputNearMaxThrottle() {
  int maxMotorValue = max(motor1pulse, max(motor2pulse, max(motor3pulse, motor4pulse)));
  int adj = maxMotorValue - 2000;
  if (adj > 0) {
    motor1pulse -= adj;
    motor2pulse -= adj;
    motor3pulse -= adj;
    motor4pulse -= adj;
  }

}

void capMotorInputNearMinThrottle(){
  //at the very least, stop pulse going below 1000
  if(motor1pulse<ZERO_THROTTLE) motor1pulse = ZERO_THROTTLE;
  if(motor2pulse<ZERO_THROTTLE) motor2pulse = ZERO_THROTTLE;
  if(motor3pulse<ZERO_THROTTLE) motor3pulse = ZERO_THROTTLE;
  if(motor4pulse<ZERO_THROTTLE) motor4pulse = ZERO_THROTTLE;
}




void updateMotors() {
  motor1.writeMicroseconds(motor1pulse);
  motor2.writeMicroseconds(motor2pulse);
  motor3.writeMicroseconds(motor3pulse);
  motor4.writeMicroseconds(motor4pulse);
}




// FOR TESTING ONLY
void setMotorsLow() {
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}
void setMotorsHigh() {
  motor1.writeMicroseconds(2000);
  motor2.writeMicroseconds(2000);
  motor3.writeMicroseconds(2000);
  motor4.writeMicroseconds(2000);
}
void setMotorsCustom(int input) {
  motor1.writeMicroseconds(input);
  motor2.writeMicroseconds(input);
  motor3.writeMicroseconds(input);
  motor4.writeMicroseconds(input);
}


