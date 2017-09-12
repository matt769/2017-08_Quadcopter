
// output gyro/accel readings that can be used by processing
// chek with motors running

//hope that motion sensor response is fast enought that I don't need to use JR DMP code



// SOFTWARE
// starting values for PID
// in YMFC example (which is overall very similar)
// for roll and pitch, P = 1.4, I = 0.05, D = 15
// for yaw, P = 4.0, I = 0.02, D = 0
// P seems really low!
// and max PID output is 400 - this seems really high!


// GENERAL
// CONNECT ESC signal grounds! may help with the stutter
// add voltage divider and battery monitor to breadboard
// Balance propellors!
// CHeck for capacitors connected to power system
// buy safety goggles
// make frame to hold/test it
// after main framework done, test timings and speed up (ONLY IF REQUIRED)

// LATER
// change 'mode' to just be attitude on/off rather than rate/attiture, because rate will always happen in the background
// possible to not update the motor pulse until after the current one (if active) has finished.
// CHECK TIMINGS ON ATMEGA chip, not Arduino Mega (as it is currently on)
// test changing Servo refresh rate
// strip out most of Servo code, just implement what I need (good learning experience)


#include <I2C.h>
#include "Servo.h"
#include <SPI.h>
#include <RF24.h>
#include "PID_v1.h" // try changing timing to micros()?

#include "I2cFunctions.h"
#include "MotionSensor.h"
#include "PIDSettings.h"
#include "Receiver.h"
#include "Motors.h"
#include "BatteryMonitor.h"


int throttle;  // distinct from the user input because I may need to modify
bool attitude_mode = false;  // REMOVE THIS AND JUST USE STATE (actually change STATE to MODE)
bool auto_level = false;  // ADD SOMETHING TO CHANGE THIS WHEN NO INPUT

byte RATE = 0;
byte BALANCE = 1;
byte MODE = RATE; // MODE IS ONLY FOR RATE or ATTITUDE
byte PREV_MODE = RATE;

bool KILL = 0;
static byte countKillCommand = 0;

// all frequencies expressed in loop duration in milliseconds e.g. 100Hz = 1000/100 = 10ms
//rateLoopFreq defined in PID.h
unsigned long rateLoopLast = 0;
//attitudeLoopFreq defined in PID.h
unsigned long attitudeLoopLast = 0;
byte receiverFreq = 50;  // although this can also be controlled on the transmitter side
unsigned long receiverLast = 0;

byte pinStatusLed = 4;

const int MIN_THROTTLE = 1100;  // CHECK THIS
//const int MAX_THROTTLE = 1800;  // CHECK THIS


// DEBUG
unsigned long lastPrint = 0;  // for debug only

// for testing
unsigned long offTimer = 15000;
unsigned long timeOn;



void setup() {
  Serial.begin(115200);

  pinMode(pinStatusLed, OUTPUT);

  setupBatteryMonitor();
  setupI2C();
  setupMotionSensor();
  setupRadio();
  setupPid();
  setupMotors();

  initialiseCurrentAngles();

  // wait for radio connection
      while (!checkRadioForInput());
      while (rcPackage.throttle < 200) checkRadioForInput();
      while (rcPackage.throttle >50) checkRadioForInput();
 
  Serial.println(F("Setup complete"));
  digitalWrite(pinStatusLed, HIGH);

  pidRateModeOn();

  timeOn = millis();  // DEBUGGING

}

int loopCounter = 0;

void loop() {

  //   FOR TESTING
//  if (millis() - timeOn > offTimer) {
//    setMotorsLow();
//    //    Serial.println("Stopping");
//    while (1) {
//    }
//  }


  // CHECK FOR USER INPUT
  if (millis() - receiverLast > receiverFreq) {
    // we don't need to bother doing any of this stuff if there's no actual input
    if (checkRadioForInput()) {
      receiverLast = millis();
      // CHECK MODES
      attitude_mode = getMode();
      auto_level = getAutolevel() || !checkHeartbeat();
      if (getKill() && (throttle < 1050)) {
        setMotorsLow();
        Serial.println("KILL");
        while (1);  //
      }
      // MAP CONTROL VALUES
      mapThrottle(&throttle);
      if (attitude_mode || auto_level) {
        mapRcToPidInput(&attitudeRollSettings.target, &attitudePitchSettings.target, &attitudeYawSettings.target, &attitude_mode);
        MODE = BALANCE;
      }
      else {
        mapRcToPidInput(&rateRollSettings.target, &ratePitchSettings.target, &rateYawSettings.target, &attitude_mode);
        MODE = RATE;
      }
    }

    // update battery info
    calculateBatteryVoltage();
    calculateBatteryLevel();
    updateBatteryIndicator();  // could go in its own loop
  }







  // OVERRIDE PID SETTINGS IF TRYING TO AUTO-LEVEL
  if (auto_level) { // if no communication received, OR user has specified auto-level
    setAutoLevelTargets();
    // If connection lost then also change throttle so that QC is descending slowly
    if (!rxHeartbeat) {
      calculateVerticalAccel();
      connectionLostDescend(&throttle, &ZAccel);
    }
  }

  // HANDLE STATE CHANGES
  if (MODE != PREV_MODE) {
    if (MODE == BALANCE) {
      pidAttitudeModeOn();
//      Serial.println(F("Entering attitude mode"));
      PREV_MODE = BALANCE;
    }
    else {
      pidAttitudeModeOff();
//      Serial.println(F("Leaving attitude mode"));
      PREV_MODE = RATE;
    }
  }

  // RUN RATE LOOP
  if (millis() - rateLoopLast > rateLoopFreq) {
    rateLoopLast = millis();
    //    loopCounter ++;
    readMainSensors();
    convertGyroReadingsToValues();
    pidRateUpdate();
    setRatePidActual(&valGyX, &valGyY, &valGyZ);
    calculateMotorInput(&throttle, &rateRollSettings.output, &ratePitchSettings.output, &rateYawSettings.output);
    capMotorInputNearMaxThrottle();
    capMotorInputNearMinThrottle();
    updateMotors();
    // required for attitude calculations
    accumulateGyroChange();
    accumulateAccelReadings();
  }

  // RUN ATTITUDE LOOP
  // Note that the attitude PID itself will not run unless QC is in ATTITUDE mode
  if (millis() - attitudeLoopLast > attitudeLoopFreq) {
    attitudeLoopLast = millis();
    calcAnglesAccel();
    mixAngles();
    resetGyroChange();
    if (attitude_mode) {
      setAttitudePidActual(&currentAngles.roll, &currentAngles.pitch, &currentAngles.yaw);
      pidAttitudeUpdate();
      // set rate setpoints
      setRatePidTargets(&attitudeRollSettings.output, &attitudePitchSettings.output, &attitudeYawSettings.output);
      overrideYawTarget();  // OVERIDE THE YAW BALANCE PID OUTPUT
    }
  }


  // DEBUGGONG
  if (millis() - lastPrint > 1000) {

//  Serial.print(dividerReading);Serial.print('\t');
//  Serial.print(dividerVoltage);Serial.print('\t');
//  Serial.print(batteryVoltage);Serial.print('\t');
//  Serial.print(batteryLevel);Serial.print('\n');

    Serial.println(statusForAck);

//    Serial.print(rxHeartbeat);Serial.print('\t');
//    Serial.print(auto_level);Serial.print('\t');
//    Serial.print(lastRxReceived);Serial.print('\t');
//    
//    Serial.print(throttle);Serial.print('\n');

    //    Serial.print(functionTimeSum);Serial.print('\t');
    //    Serial.print(functionTimeCounter);Serial.print('\t');
    //    Serial.print((float)functionTimeSum / functionTimeCounter);Serial.print('\n');
    //    functionTimeSum = 0;
    //    functionTimeCounter = 0;

    //    Serial.println(loopCounter);
    //    loopCounter = 0;

    //    Serial.print(throttle); Serial.print('\t');
    //    Serial.print(valGyX); Serial.print('\n');
    //    printPackage();
    //    Serial.print(motor1pulse); Serial.print('\t');
    //    Serial.print(motor2pulse); Serial.print('\n');
    //    Serial.print(motor3pulse); Serial.print('\t');
    //    Serial.print(motor4pulse); Serial.print('\n');
    //    Serial.print('\n');
    //
    //    Serial.println(maxLoopDuration);
    //    Serial.print(F("Outer loop: ")); Serial.print('\t');
    //        Serial.print(attitudeRollSettings.actual); Serial.print('\t');
    //        Serial.print(attitudePitchSettings.actual); Serial.print('\t');
    //        Serial.print(attitudeYawSettings.actual); Serial.print('\t');
    //        Serial.print(attitudeRollSettings.target); Serial.print('\t');
    //        Serial.print(attitudePitchSettings.target); Serial.print('\t');
    //        Serial.print(attitudeYawSettings.target); Serial.print('\t');
    //        Serial.print(attitudeRollSettings.output); Serial.print('\t');
    //        Serial.print(attitudePitchSettings.output); Serial.print('\t');
    //        Serial.print(attitudeYawSettings.output); Serial.print('\n');
//        Serial.print(F("Inner loop: ")); Serial.print('\t');
//            Serial.print(rateRollSettings.actual); Serial.print('\t');
//            Serial.print(ratePitchSettings.actual); Serial.print('\t');
//            Serial.print(rateYawSettings.actual); Serial.print('\t');
//            Serial.print(rateRollSettings.target); Serial.print('\t');
//            Serial.print(ratePitchSettings.target); Serial.print('\t');
//            Serial.print(rateYawSettings.target); Serial.print('\t');
//            Serial.print(rateRollSettings.output); Serial.print('\t');
//            Serial.print(ratePitchSettings.output); Serial.print('\t');
//            Serial.print(rateYawSettings.output); Serial.print('\n');
    //    //
    //        Serial.print('\n');
    lastPrint = millis();
  }


} // loop
