// RECEIVER (ignore for now - use direct control?)
// Start work on motor control

// after main framework done, test timings and speed up (ONLY IF REQUIRED)
// rename all X/Y/Z as roll/pitch/yaw
// add rx heartbeat
// Need timeout on radio
// Handle changes in mode
// Add autolevel (in absence of input, change to balance mode and set targets to 0)

// decide how to handle PID timing - as done below or just using in-built PID timing?
//  probably the latter but will requires some minor changes to structure/order of main loop

// CHECK TIMINGS ON ATMEGA chip, not Arduino Mega (as it is currently on)

// where to handle change of state between rate and balance
// I think I need to re-think a bit

// I've included local copy of Servo library so that I can modify REFRESH_INTERVAL
// but default (50Hz) should be ok. ESCs support 30-450Hz

// What is in RAD and what in DEG ???


#include <I2C.h>
#include "Servo.h"
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
bool balance_mode = false;
bool auto_level_mode = false;  // ADD SOMETHING TO CHANGE THIS WHEN NO INPUT

byte RATE = 0;
byte BALANCE = 1;
byte STATE = RATE;
byte PREV_STATE = RATE;

// all frequencies expressed in loop duration in milliseconds e.g. 100Hz = 1000/100 = 10ms
int rateLoopFreq = 4;   // remove ** ~1ms **  from desired loop time to compensate to time to run code
unsigned long rateLoopLast = 0;
int balanceLoopFreq = 45;   // remove ** ~5ms **  from desired loop time to compensate to time to run code
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

  Serial.println(F("Setup complete"));
  // Indicate readiness somehow. LED?

  pidRateModeOn(); //normally this would only come after arming
  pidBalanceModeOn(); //normally this would only come on/off after entering/leaving balance mode
//  balance_mode = true;  // FOR DEBUGGING ONLY

  // ARM MOTORS AUTOMATICALLY - DEBUGGING ONLY
    unsigned long timeNow = millis();
    setMotorsHigh();
    while(millis() - timeNow > 1000){
      
    }
    timeNow = millis();
    setMotorsLow();
    while(millis() - timeNow > 1000){
      
    }

}

unsigned long lastPrint = 0;  // for debug only
int counter = 0;  // for debug only

void loop() {


  

//  balance_mode = getMode();
//  if (balance_mode && STATE == RATE) {
//    STATE = BALANCE;
//    PREV_STATE = RATE;
//  }
//  else if (!balance_mode && STATE == BALANCE) {
//    STATE = RATE;
//    PREV_STATE = BALANCE;
//  }



  if (millis() - receiverLast > receiverFreq) {
    if (checkRadioForInputPLACEHOLDER()) { // currently contains placeholder values
      balance_mode = getMode();
      if (balance_mode) {
        mapRcToPidInput(&rcInputThrottle, &balanceRollSettings.target, &balancePitchSettings.target, &balanceYawSettings.target, &balance_mode);
      }
      else {
        mapRcToPidInput(&rcInputThrottle, &rateRollSettings.target, &ratePitchSettings.target, &rateYawSettings.target, &balance_mode);
      }
    }
  }




  // overwrite user input (actually there won't be any if using auto level)
  // need to determine what throttle level is appropriate
  // maybe remove this for now
  if (auto_level_mode) {
    rateRollSettings.target = 0;
    ratePitchSettings.target = 0;
    rateYawSettings.target = 0;
  }

  if (millis() - rateLoopLast > rateLoopFreq) {
    //        Serial.println(millis());
    rateLoopLast = millis();
    counter += readMainSensors();
    convertGyroReadingsToValues();
    rateRollSettings.actual = valGyX;
    ratePitchSettings.actual = valGyY;
    rateYawSettings.actual = valGyZ;
    pidRateUpdate();
    calculateMotorInput(&rcInputThrottle, &rateRollSettings.output, &ratePitchSettings.output, &rateYawSettings.output);
    updateMotors();

    accumulateGyroChange();

    //    Serial.print("GyroChange:"); Serial.print('\t');
    //    Serial.print(gyroChangeAngles.roll); Serial.print('\t');
    //    Serial.print(gyroChangeAngles.pitch); Serial.print('\t');
    //    Serial.print(gyroChangeAngles.yaw); Serial.print('\n');
    accumulateAccelReadings();  // or should I just take these in balance loop?
  }

  // should this be calculated regardless of whether balance loop is on or not? Yes, ideally
  // but it won't feed anything
  if (millis() - balanceLoopLast > balanceLoopFreq) {
    balanceLoopLast = millis();

    calcAnglesAccel();
    mixAngles();
    resetGyroChange();

    balanceRollSettings.actual = currentAngles.roll;
    balancePitchSettings.actual = currentAngles.pitch;
    balanceYawSettings.actual = currentAngles.yaw;

    //    Serial.print("InDegrees: "); Serial.print('\t');
    //    Serial.print(balanceRollSettings.actual); Serial.print('\t');
    //    Serial.print(balancePitchSettings.actual); Serial.print('\t');
    //    Serial.print(balanceYawSettings.actual); Serial.print('\t');
    //    Serial.print(counter); Serial.print('\n');
    counter = 0;
    //    Serial.print('\n');

    if (balance_mode) {
      pidBalanceUpdate();
      // set rate setpoints
      rateRollSettings.target = balanceRollSettings.output;
      ratePitchSettings.target = balancePitchSettings.output;
      rateYawSettings.target = balanceYawSettings.output;

      rateYawSettings.target = 0;   // OVERIDE THE YAW BALANCE PID OUTPUT
      
//      Serial.print("Actual: "); Serial.print('\t');
//      Serial.print(balanceRollSettings.actual); Serial.print('\t');
//      Serial.print(balancePitchSettings.actual); Serial.print('\t');
//      Serial.print(balanceYawSettings.actual); Serial.print('\n');
//      Serial.print("Target: "); Serial.print('\t');
//      Serial.print(balanceRollSettings.target); Serial.print('\t');
//      Serial.print(balancePitchSettings.target); Serial.print('\t');
//      Serial.print(balanceYawSettings.target); Serial.print('\n');
//      Serial.print("Output: "); Serial.print('\t');
//      Serial.print(balanceRollSettings.output); Serial.print('\t');
//      Serial.print(balancePitchSettings.output); Serial.print('\t');
//      Serial.print(balanceYawSettings.output); Serial.print('\n');
//      Serial.print('\n');
      
    }
  }
//  Serial.println(motor1pulse);

  if (millis() - lastPrint > 1000) {
    //    Serial.print(valGyX); Serial.print('\n');
    //      Serial.print(rateRollSettings.target); Serial.print('\t');
    //      Serial.print(ratePitchSettings.target); Serial.print('\t');
    //      Serial.print(rateYawSettings.target); Serial.print('\n');
    //      Serial.print(rateRollSettings.output); Serial.print('\t');
    //      Serial.print(ratePitchSettings.output); Serial.print('\t');
    //      Serial.print(rateYawSettings.output); Serial.print('\n');
    Serial.print(motor1pulse); Serial.print('\t');
    Serial.print(motor2pulse); Serial.print('\n');
    Serial.print(motor3pulse); Serial.print('\t');
    Serial.print(motor4pulse); Serial.print('\n');

        Serial.print(balanceRollSettings.actual); Serial.print('\t');
        Serial.print(balancePitchSettings.actual); Serial.print('\t');
        Serial.print(balanceYawSettings.actual); Serial.print('\n');
        Serial.print(balanceRollSettings.target); Serial.print('\t');
        Serial.print(balancePitchSettings.target); Serial.print('\t');
        Serial.print(balanceYawSettings.target); Serial.print('\n');
        Serial.print(balanceRollSettings.output); Serial.print('\t');
        Serial.print(balancePitchSettings.output); Serial.print('\t');
        Serial.print(balanceYawSettings.output); Serial.print('\n');

        Serial.print(rateRollSettings.actual); Serial.print('\t');
        Serial.print(ratePitchSettings.actual); Serial.print('\t');
        Serial.print(rateYawSettings.actual); Serial.print('\n');
        Serial.print(rateRollSettings.target); Serial.print('\t');
        Serial.print(ratePitchSettings.target); Serial.print('\t');
        Serial.print(rateYawSettings.target); Serial.print('\n');
        Serial.print(rateRollSettings.output); Serial.print('\t');
        Serial.print(ratePitchSettings.output); Serial.print('\t');
        Serial.print(rateYawSettings.output); Serial.print('\n');
    //
    Serial.print('\n');
    lastPrint = millis();
  }



} // loop
