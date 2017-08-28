// RECEIVER (ignore for now - use direct control?)
// Start work on motor control

// inner loop for rate mode - constant rate of change of angle
// outer loop for balance mode

// remove debug statements later

// should I control ESC pulses myself or use servo.write - USE SERVO.WRITE for now

// use interupts from radio/MPU? or just poll

// Need to rearrange the order of some bits, esp. around balance/rate mode


#include <I2C.h>
#include <Servo.h>
#include <SPI.h>
#include <RF24.h>
#include <PID_v1.h>

#include "debug_and_misc.h"
#include "BatteryMonitor.h"
#include "I2cFunctions.h"
#include "MotionSensor.h"
#include "PID.h"
#include "Receiver.h"
#include "Motors.h"


int rcInputThrottle;
//double rcInputRatePitch;
//double rcInputRateRoll;
//double rcInputRateYaw;

bool balance_mode;  




void setup() {
  Serial.begin(115200);

  setupI2C();
  setupMotionSensor();
  //  setupRadio();   ignore for now
  setupPid();
  setupMotors();


  Serial.println(F("Setup complete"));
  // Indicate readiness somehow. LED?

  pidRateModeOn(); //normally this would only come after arming
  balance_mode = false;

}

unsigned long lastPrint = 0;  // for debug only



void loop() {
  // at what frequency should everything run?
  // gyro 8kHz, accel 1kZ - but depends on LPF settings too I think
  // receiver 10Hz?
  // read sensor 250Hz?
  // inner loop 250Hz?
  // outer 100Hz?

  // read receiver
  // read gyros/accel
  // calculate inner PID (gyro info only)
  // update motors
  // update current angle estimation (gyro + accel info)
  // update inner loop setpoint
  // update outer loop setpoint (if balance mode)

//  Serial.println("1");
  checkRadioForInputPLACEHOLDER(); // currently contains placeholder values
//  Serial.println("2");
  if(balance_mode){
    mapRcToPidInput(&rcInputThrottle, &balanceRollSettings.target, &balancePitchSettings.target, &balanceYawSettings.target, &balance_mode);
    // NEED TO CALCUALTE ACTUAL ANGLE
    pidBalanceUpdate();
    // set rate setpoints
    ratePitchSettings.target = balancePitchSettings.output;
    rateRollSettings.target = balanceRollSettings.output;
    rateYawSettings.target = balanceYawSettings.output;
  }
  else {
    mapRcToPidInput(&rcInputThrottle, &rateRollSettings.target, &ratePitchSettings.target, &rateYawSettings.target, &balance_mode);
  }
  
  readMainSensors(MPU_ADDRESS);
  convertReadingsToValues();

  rateRollSettings.actual = valGyX;
  ratePitchSettings.actual = valGyY;
  rateYawSettings.actual = valGyZ;

  pidRateUpdate();

  calculateMotorInput(&rcInputThrottle, &rateRollSettings.output, &ratePitchSettings.output, &rateYawSettings.output);  // change to use pointers
  updateMotors();

  if (millis() - lastPrint > 1000) {
//    Serial.print(valGyX); Serial.print('\n');
    Serial.print(rateRollSettings.target); Serial.print('\t');
    Serial.print(ratePitchSettings.target); Serial.print('\t');
    Serial.print(rateYawSettings.target); Serial.print('\n');
    Serial.print(rateRollSettings.output); Serial.print('\t');
    Serial.print(ratePitchSettings.output); Serial.print('\t');
    Serial.print(rateYawSettings.output); Serial.print('\n');
    Serial.print(motor1pulse); Serial.print('\t');
    Serial.print(motor2pulse); Serial.print('\n');
    Serial.print(motor3pulse); Serial.print('\t');
    Serial.print(motor4pulse); Serial.print('\n');
    Serial.print('\n');
    lastPrint = millis();
  }
  


  



} // loop
