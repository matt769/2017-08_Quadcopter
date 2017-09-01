// renamed balance to attitude
// added a function for dealing with throttle in autolevel mode
//    not actually being run in main loop
//    and need to add some logic around when this is actually used (i.e. user may want to retain control of throttle)
// added/removed comments



// RECEIVER (ignore for now - use direct control?)

// after main framework done, test timings and speed up (ONLY IF REQUIRED)
// rename all X/Y/Z as roll/pitch/yaw?
// Need timeout on radio in case of 'freeze'

// CHECK TIMINGS ON ATMEGA chip, not Arduino Mega (as it is currently on)

// May need to re-think how handling change of state between rate and balance

// scaling according to voltage

// turn on auto level when there are 5 missed transmissions (based on elapsed time since last)
// OR user turns in on in control byte
// need to address what is happening with throttle in case of lost input
// also need to keep overriding user input for the other controls
// I could put a filter on Z axis accel value to see if QC is going up or down, and modify throttle accordingly
// could add another PID for this
// really need barometer too though

// what else is required in SOFTWARE order for it to actually fly
// (aside from testing existing stuff)
//  - LOW THROTTLE 'LIMIT'

// and in HARDWARE
// make frame to hold/test it

// print some lined/squared paper

// order some thinner/more flexible wire? mabye can get from maplins?

// starting values for PID
// in YMFC example (which is overall very similar)
// for roll and pitch, P = 1.4, I = 0.05, D = 15
// for yaw, P = 4.0, I = 0.02, D = 0
// P seems really low!
// and max PID output is 400 - this seems really high!

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
bool attitude_mode = false;  // REMOVE THIS AND JUST USE STATE (actually change STATE to MODE)
bool auto_level = false;  // ADD SOMETHING TO CHANGE THIS WHEN NO INPUT

byte RATE = 0;
byte BALANCE = 1;
byte MODE = RATE; // MODE IS ONLY FOR RATE or ATTITUDE
byte PREV_MODE = RATE;

// all frequencies expressed in loop duration in milliseconds e.g. 100Hz = 1000/100 = 10ms
//rateLoopFreq defined in PID.h
unsigned long rateLoopLast = 0;
//attitudeLoopFreq defined in PID.h
unsigned long attitudeLoopLast = 0;
byte receiverFreq = 20;  // although this can also be controlled on the transmitter side
unsigned long receiverLast = 0;

byte statusLed = A5;



// DEBUG
unsigned long lastPrint = 0;  // for debug only
//int counter = 0;  // for debug only

// for testing
unsigned long offTimer = 15000;
unsigned long timeOn;



void setup() {
  Serial.begin(115200);

  pinMode(statusLed,OUTPUT);
  
  setupBatteryMonitor();
  setupI2C();
  setupMotionSensor();
  //  setupRadio();   ignore for now
  setupPid();
  setupMotors();

  initialiseCurrentAngles();

  Serial.println(F("Setup complete"));
  digitalWrite(statusLed,HIGH);

  pidRateModeOn(); // ideally this would only come after arming

  timeOn = millis();

//  setMotorsLow();
//  delay(5000);
  
}



void loop() {


  if(millis() - timeOn > offTimer){
    setMotorsLow();
//    Serial.println("Stopping");
    while(1){
      
    }
  }


  // CHECK FOR USER INPUT
  if (millis() - receiverLast > receiverFreq) {
    if (checkRadioForInputPLACEHOLDER()) { // currently contains placeholder values
      attitude_mode = getMode();
      auto_level = !checkHeartbeat() || getAutolevel();
      if (attitude_mode || auto_level) {
        mapRcToPidInput(&rcInputThrottle, &attitudeRollSettings.target, &attitudePitchSettings.target, &attitudeYawSettings.target, &attitude_mode);
        MODE = BALANCE;
      }
      else {
        mapRcToPidInput(&rcInputThrottle, &rateRollSettings.target, &ratePitchSettings.target, &rateYawSettings.target, &attitude_mode);
        MODE = RATE;
      }
    }
  }

  // OVERRIDE PID SETTINGS IF TRYING TO AUTO-LEVEL
  if(auto_level){  // if no communication receiver, OR user has specified auto-level
    // need to address what is happening with throttle in case of lost input
    // also need to keep overriding user input for the other controls
    setAutoLevelTargets();
  }


  rcInputThrottle = map(analogRead(A12),1023,0,1000,1300); // OVERRIDE THROTTLE WITH MANUAL INPUT  *DEBUGGING*
//  Serial.println(rcInputThrottle);

  // HANDLE STATE CHANGES
  if(MODE != PREV_MODE){
    if(MODE == BALANCE){
      pidAttitudeModeOn();
    }
    else {
      pidAttitudeModeOff();
    }
  }


  // RUN RATE LOOP
  if (millis() - rateLoopLast > rateLoopFreq) {
    //        Serial.println(millis());
    rateLoopLast = millis();
//    counter += readMainSensors(); // DEBUGGING
    readMainSensors();
    convertGyroReadingsToValues();
    rateRollSettings.actual = valGyX; // tidy up these into a function?
    ratePitchSettings.actual = valGyY;
    rateYawSettings.actual = valGyZ;
    pidRateUpdate();
    calculateMotorInput(&rcInputThrottle, &rateRollSettings.output, &ratePitchSettings.output, &rateYawSettings.output);
    //capMotorInputNearMaxThrottle(); //alternative would be a general cap on throttle
    updateMotors();

    accumulateGyroChange();

    //    Serial.print("GyroChange:"); Serial.print('\t');
    //    Serial.print(gyroChangeAngles.roll); Serial.print('\t');
    //    Serial.print(gyroChangeAngles.pitch); Serial.print('\t');
    //    Serial.print(gyroChangeAngles.yaw); Serial.print('\n');
    accumulateAccelReadings();  // or should I just take these in attitude loop?
  }

  // RUN ATTITUDE LOOP
  // Note that the attitude PID itself will not run unless QC is in ATTITUDE mode
  if (millis() - attitudeLoopLast > attitudeLoopFreq) {
    attitudeLoopLast = millis();

    calcAnglesAccel();
    mixAngles();
    resetGyroChange();

    attitudeRollSettings.actual = currentAngles.roll;
    attitudePitchSettings.actual = currentAngles.pitch;
    attitudeYawSettings.actual = currentAngles.yaw;

    //    Serial.print("InDegrees: "); Serial.print('\t');
    //    Serial.print(attitudeRollSettings.actual); Serial.print('\t');
    //    Serial.print(attitudePitchSettings.actual); Serial.print('\t');
    //    Serial.print(attitudeYawSettings.actual); Serial.print('\t');
    //    Serial.print(counter); Serial.print('\n');
//    counter = 0;
    //    Serial.print('\n');

    if (attitude_mode) {
      pidAttitudeUpdate();
      // set rate setpoints
      rateRollSettings.target = attitudeRollSettings.output;
      ratePitchSettings.target = attitudePitchSettings.output;
      rateYawSettings.target = attitudeYawSettings.output;

      rateYawSettings.target = 0;   // OVERIDE THE YAW BALANCE PID OUTPUT
      
//      Serial.print("Actual: "); Serial.print('\t');
//      Serial.print(attitudeRollSettings.actual); Serial.print('\t');
//      Serial.print(attitudePitchSettings.actual); Serial.print('\t');
//      Serial.print(attitudeYawSettings.actual); Serial.print('\n');
//      Serial.print("Target: "); Serial.print('\t');
//      Serial.print(attitudeRollSettings.target); Serial.print('\t');
//      Serial.print(attitudePitchSettings.target); Serial.print('\t');
//      Serial.print(attitudeYawSettings.target); Serial.print('\n');
//      Serial.print("Output: "); Serial.print('\t');
//      Serial.print(attitudeRollSettings.output); Serial.print('\t');
//      Serial.print(attitudePitchSettings.output); Serial.print('\t');
//      Serial.print(attitudeYawSettings.output); Serial.print('\n');
//      Serial.print('\n');
      
    }
  }

//    updateBatteryIndicator();
  
//  Serial.println(motor1pulse);

  if (millis() - lastPrint > 1000) {
    //    Serial.print(valGyX); Serial.print('\n');

//    Serial.print(motor1pulse); Serial.print('\t');
//    Serial.print(motor2pulse); Serial.print('\n');
//    Serial.print(motor3pulse); Serial.print('\t');
//    Serial.print(motor4pulse); Serial.print('\n');
//    Serial.print('\n');
//
//        Serial.print(attitudeRollSettings.actual); Serial.print('\t');
//        Serial.print(attitudePitchSettings.actual); Serial.print('\t');
//        Serial.print(attitudeYawSettings.actual); Serial.print('\n');
//        Serial.print(attitudeRollSettings.target); Serial.print('\t');
//        Serial.print(attitudePitchSettings.target); Serial.print('\t');
//        Serial.print(attitudeYawSettings.target); Serial.print('\n');
//        Serial.print(attitudeRollSettings.output); Serial.print('\t');
//        Serial.print(attitudePitchSettings.output); Serial.print('\t');
//        Serial.print(attitudeYawSettings.output); Serial.print('\n');
//
//        Serial.print(rateRollSettings.actual); Serial.print('\t');
//        Serial.print(ratePitchSettings.actual); Serial.print('\t');
//        Serial.print(rateYawSettings.actual); Serial.print('\n');
//        Serial.print(rateRollSettings.target); Serial.print('\t');
//        Serial.print(ratePitchSettings.target); Serial.print('\t');
//        Serial.print(rateYawSettings.target); Serial.print('\n');
//        Serial.print(rateRollSettings.output); Serial.print('\t');
//        Serial.print(ratePitchSettings.output); Serial.print('\t');
//        Serial.print(rateYawSettings.output); Serial.print('\n');
    //
//    Serial.print('\n');
    lastPrint = millis();
  }



} // loop
