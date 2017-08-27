// actually the ESCs really

Servo motor1; // front left (CW)
Servo motor2; // front right (CCW)
Servo motor3; // back left (CCW)
Servo motor4; // back right (CW)

byte pinMotor1 = 4; // UPDATE
byte pinMotor2 = 5; // UPDATE
byte pinMotor3 = 6; // UPDATE
byte pinMotor4 = 7; // UPDATE

int motor1pulse;
int motor2pulse;
int motor3pulse;
int motor4pulse;


void setupMotors(){
  motor1.writeMicroseconds(1000);
  motor1.attach(pinMotor1);
  motor2.writeMicroseconds(1000);
  motor2.attach(pinMotor2);
  motor3.writeMicroseconds(1000);
  motor3.attach(pinMotor3);
  motor4.writeMicroseconds(1000);
  motor4.attach(pinMotor4);
}



void calculateMotorInput(){
  motor1pulse = throttle + rateRollSettings.ouput - ratePitchSettings.ouput - rateYawSettings.ouput
  motor2pulse = throttle + rateRollSettings.ouput + ratePitchSettings.ouput + rateYawSettings.ouput
  motor3pulse = throttle - rateRollSettings.ouput - ratePitchSettings.ouput + rateYawSettings.ouput
  motor4pulse = throttle - rateRollSettings.ouput + ratePitchSettings.ouput - rateYawSettings.ouput  
}



void updateMotors(){
  motor1.writeMicroseconds(motor1pulse);
  motor2.writeMicroseconds(motor2pulse);
  motor3.writeMicroseconds(motor3pulse);
  motor4.writeMicroseconds(motor4pulse);
}

