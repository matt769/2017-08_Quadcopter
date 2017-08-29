// RECEIVER (ignore for now - use direct control?)
// Start work on motor control

// after main framework done, test timings and try and speed up if required
// rename all X/Y/Z as roll/pitch/yaw
// add rx heartbeat
// Need timeout on radio


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
bool balance_mode;

// all frequencies expressed in loop duration in milliseconds e.g. 100Hz = 1000/100 = 10ms
int rateLoopFreq = 9;   // remove ** 1ms **  from desired loop time to compensate to time to run code
unsigned long rateLoopLast = 0;
int balanceLoopFreq = 45;   // remove ** 5ms **  from desired loop time to compensate to time to run code
unsigned long balanceLoopLast = 0;
int receiverFreq = 20;  // although this can also be controlled on the transmitter side
unsigned long receiverLast = 0;



void setup() {
  Serial.begin(115200);

  setupI2C();
  setupMotionSensor();
  //  setupRadio();   ignore for now
  setupPid();
  setupMotors();

  initialiseCurrentAngles();

  Serial.println(AcXAve);Serial.print('\t');
  Serial.println(AcYAve);Serial.print('\t');
  Serial.println(AcZAve);Serial.print('\n');
  Serial.print(accelAngles.roll * RAD_TO_DEG);Serial.print('\t');
  Serial.print(accelAngles.pitch * RAD_TO_DEG);Serial.print('\t');
  Serial.print(accelAngles.yaw * RAD_TO_DEG);Serial.print('\n');
  Serial.print(currentAngles.roll * RAD_TO_DEG);Serial.print('\t');
  Serial.print(currentAngles.pitch * RAD_TO_DEG);Serial.print('\t');
  Serial.print(currentAngles.yaw * RAD_TO_DEG);Serial.print('\n');
  
  

  Serial.println(F("Setup complete"));
  // Indicate readiness somehow. LED?

  pidRateModeOn(); //normally this would only come after arming
  balance_mode = true;

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

  if (millis() - receiverLast > receiverFreq) {
    if (checkRadioForInputPLACEHOLDER()) { // currently contains placeholder values
      // only run if received new command
      if (balance_mode) {
        mapRcToPidInput(&rcInputThrottle, &balanceRollSettings.target, &balancePitchSettings.target, &balanceYawSettings.target, &balance_mode);
      }
      else {
        mapRcToPidInput(&rcInputThrottle, &rateRollSettings.target, &ratePitchSettings.target, &rateYawSettings.target, &balance_mode);
      }
    }
  }
  // ADD - only do if received new input


  if (millis() - rateLoopLast > rateLoopFreq) {
//        Serial.println(millis());
    rateLoopLast = millis();
    readMainSensors();
    convertGyroReadingsToValues();
    rateRollSettings.actual = valGyX;
    ratePitchSettings.actual = valGyY;
    rateYawSettings.actual = valGyZ;
    pidRateUpdate();
    calculateMotorInput(&rcInputThrottle, &rateRollSettings.output, &ratePitchSettings.output, &rateYawSettings.output);
    updateMotors();

    accumulateGyroChange();
    accumulateAccelReadings();  // or should I just take these in balance loop?


  }

  // should this be calculated regardless of whether balance loop is on or not? Yes, ideally
  // but it won't feed anything
  if (millis() - balanceLoopLast > balanceLoopFreq) {
//    Serial.println(millis());
    balanceLoopLast = millis();

  Serial.print(currentAngles.roll * RAD_TO_DEG);Serial.print('\t');
  Serial.print(currentAngles.pitch * RAD_TO_DEG);Serial.print('\t');
  Serial.print(currentAngles.yaw * RAD_TO_DEG);Serial.print('\n');

    calcAnglesAccel();
          Serial.print(accelAngles.roll * RAD_TO_DEG);Serial.print('\t');
  Serial.print(accelAngles.pitch * RAD_TO_DEG);Serial.print('\t');
  Serial.print(accelAngles.yaw * RAD_TO_DEG);Serial.print('\n');
    mixAngles();
      Serial.print(currentAngles.roll * RAD_TO_DEG);Serial.print('\t');
  Serial.print(currentAngles.pitch * RAD_TO_DEG);Serial.print('\t');
  Serial.print(currentAngles.yaw * RAD_TO_DEG);Serial.print('\n');
    balanceRollSettings.actual = currentAngles.roll * RAD_TO_DEG;
    balancePitchSettings.actual = currentAngles.pitch * RAD_TO_DEG;
    balanceYawSettings.actual = currentAngles.yaw * RAD_TO_DEG;
            Serial.print(balanceRollSettings.actual); Serial.print('\t');
      Serial.print(balancePitchSettings.actual); Serial.print('\t');
      Serial.print(balanceYawSettings.actual); Serial.print('\n');
      Serial.print('\n');

    if (balance_mode) {
      //      Serial.println(millis());
      pidBalanceUpdate();
      // set rate setpoints
      ratePitchSettings.target = balancePitchSettings.output;
      rateRollSettings.target = balanceRollSettings.output;
      rateYawSettings.target = balanceYawSettings.output;
    }
  }



    if (millis() - lastPrint > 1000) {
      //    Serial.print(valGyX); Serial.print('\n');
//      Serial.print(rateRollSettings.target); Serial.print('\t');
//      Serial.print(ratePitchSettings.target); Serial.print('\t');
//      Serial.print(rateYawSettings.target); Serial.print('\n');
//      Serial.print(rateRollSettings.output); Serial.print('\t');
//      Serial.print(ratePitchSettings.output); Serial.print('\t');
//      Serial.print(rateYawSettings.output); Serial.print('\n');
//      Serial.print(motor1pulse); Serial.print('\t');
//      Serial.print(motor2pulse); Serial.print('\n');
//      Serial.print(motor3pulse); Serial.print('\t');
//      Serial.print(motor4pulse); Serial.print('\n');

      Serial.print(balanceRollSettings.actual); Serial.print('\t');
      Serial.print(balancePitchSettings.actual); Serial.print('\t');
      Serial.print(balanceYawSettings.actual); Serial.print('\n');
//
//      Serial.print('\n');
      lastPrint = millis();
    }







} // loop
