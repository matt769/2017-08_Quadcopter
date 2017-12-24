
#include <I2C.h>  // http://dsscircuits.com/articles/86-articles/66-arduino-i2c-master-library
#include <SPI.h>  // standard Arduino DPI library
#include <RF24.h> // https://github.com/nRF24/RF24


// These variables need to be available for the additional tabs to use
const int THROTTLE_LIMIT = 1500;
const int ZERO_THROTTLE = 1000;
const int THROTTLE_MIN_SPIN = 1125;
int throttle;  // distinct from the user input because it may be modified
bool error = false; // will be used in acknowledgement byte to indicate some error


#include "PID.h"
#include "Parameters.h"   // currently all commented out
#include "BatteryMonitor.h"
#include "I2cFunctions.h"
#include "MotionSensor.h"
#include "Receiver.h"
#include "Motors.h"
#include "PIDSettings.h"

bool attitude_mode = false;  // REMOVE THIS AND JUST USE STATE (actually change STATE to MODE)
bool auto_level = false;

byte RATE = 0;
byte BALANCE = 1;
byte MODE = RATE; // MODE IS ONLY FOR RATE or ATTITUDE
byte PREV_MODE = RATE;

bool KILL = 0;

// all frequencies expressed in loop duration in milliseconds e.g. 100Hz = 1000/100 = 10ms
//rateLoopFreq defined in PID.h
unsigned long rateLoopLast = 0;
//attitudeLoopFreq defined in PID.h
unsigned long attitudeLoopLast = 0;
const byte receiverFreq = 50;  // although this can also be controlled on the transmitter side
unsigned long receiverLast = 0;
unsigned long batteryLoopLast = 0;
const int batteryFreq = 1000;
const byte pinStatusLed = 8;

unsigned long lastPrint = 0;  // FOR DEBUGGING
//int loopCounterRx = 0;  // DEBUGGING
int loopCounterRate = 0;  // DEBUGGING
int loopCounterAttitude = 0;  // DEBUGGING

void setup() {
  Serial.begin(115200);
  pinMode(pinStatusLed, OUTPUT);
  digitalWrite(pinStatusLed, HIGH);
  setupBatteryMonitor();
  setupI2C();
  setupMotionSensor();
  setupRadio();
  setupPid();

  calculateOffsets();
//  calibrateGyro(500);
  initialiseCurrentAngles();

  // ARMING PROCEDURE
  // wait for radio connection and specific user input (stick up, stick down)
  while (!checkRadioForInput()){
  }
  while (rcPackage.throttle < 200) {
    checkRadioForInput();
  }
  while (rcPackage.throttle > 50) {
    checkRadioForInput();
  }
  // END ARMING
  setupMotors();
  Serial.println(F("Setup complete"));
  digitalWrite(pinStatusLed, LOW);
  pidRateModeOn();
  unsigned long startTime = millis();
  rateLoopLast = startTime;
  attitudeLoopLast = startTime;
  receiverLast = startTime;
  batteryLoopLast = startTime;
} // END SETUP


void loop() {

// ****************************************************************************************
// CHECK FOR USER INPUT
// ****************************************************************************************
  if (millis() - receiverLast >= receiverFreq) {
    receiverLast += receiverFreq;
    //    loopCounterRx++;
    checkHeartbeat();  // must be done outside if(radio.available) loop
    // we don't need to bother doing the rest of this stuff if there's no actual input
    if (checkRadioForInput()) {
      // CHECK MODES
      attitude_mode = getMode();
      auto_level = getAutolevel();
      if (getKill() && (throttle < 1050)) {
        setMotorsLow();
        digitalWrite(pinStatusLed, HIGH);
        while (1);

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
    auto_level = auto_level || !rxHeartbeat;
    if (auto_level) {
      MODE = BALANCE;
    }
  }


// ****************************************************************************************
// HANDLE STATE CHANGES
// ****************************************************************************************
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

// ****************************************************************************************
// RUN RATE LOOP
// includes sensor read
// ****************************************************************************************
  if (millis() - rateLoopLast >= rateLoopFreq) {
    rateLoopLast += rateLoopFreq;
        loopCounterRate++;
    readGyrosAccels();
    convertGyroReadingsToValues();
    setRatePidActual(&valGyX, &valGyY, &valGyZ);
    // if PID has updated the outputs then recalculate the required motor pulses
    if (pidRateUpdate()) {
      calculateMotorInput(&throttle, &rateRollSettings.output, &ratePitchSettings.output, &rateYawSettings.output);
      capMotorInputNearMaxThrottle();
      capMotorInputNearMinThrottle();
      updateMotors();
    }
    // required for attitude calculations
    accumulateGyroChange();
    accumulateAccelReadings();
 
  }

// ****************************************************************************************
// RUN ATTITUDE CALCULATIONS
// ****************************************************************************************
  if (millis() - attitudeLoopLast >= attitudeLoopFreq) {
    attitudeLoopLast += attitudeLoopFreq;
        loopCounterAttitude++;
    calcAnglesAccel();
    mixAngles();
    resetGyroChange();
    // OVERRIDE PID SETTINGS IF TRYING TO AUTO-LEVEL
    if (auto_level) { // if no communication received, OR user has specified auto-level
      setAutoLevelTargets();
      // If connection lost then also change throttle so that QC is descending slowly
      if (!rxHeartbeat) {
        calculateVerticalAccel();
        connectionLostDescend(&throttle, &ZAccel);
      }
    }
    // The attitude PID itself will not run unless QC is in ATTITUDE mode
    if (attitude_mode) {
      setAttitudePidActual(&currentAngles.roll, &currentAngles.pitch, &currentAngles.yaw);
      // if PID has updated the outputs then recalculate the required motor pulses
      if (pidAttitudeUpdate()) {
        // set rate setpoints
        setRatePidTargets(&attitudeRollSettings.output, &attitudePitchSettings.output, &attitudeYawSettings.output);
        overrideYawTarget();  // OVERIDE THE YAW BALANCE PID OUTPUT
      }
    }
  }

// ****************************************************************************************
// CHECK BATTERY
// ****************************************************************************************
  if (millis() - batteryLoopLast >= batteryFreq) {
    batteryLoopLast += batteryFreq;
    calculateBatteryLevel();
  }

// ****************************************************************************************
// DEBUGGING
// ****************************************************************************************

//    if (millis() - lastPrint >= 50) {
//        lastPrint += 50;
        
  //    Serial.print(AcX); Serial.print('\t');
  //    Serial.print(AcY); Serial.print('\t');
//      Serial.print(AcZ);
//      Serial.print('\n');
//        Serial.print(AcZAve);
  //
//      Serial.println(statusForAck);
  //    Serial.print(dividerReading); Serial.print('\t');
  //    Serial.print(dividerVoltage); Serial.print('\t');
  //    Serial.print(batteryVoltage); Serial.print('\t');
//      Serial.print(batteryLevel); 
//      Serial.print('\n');
  //
//      Serial.print(rxHeartbeat); Serial.print('\t');
//      Serial.print(auto_level); Serial.print('\t');
//      Serial.print(lastRxReceived); Serial.print('\t');
//      Serial.print(MODE); Serial.print('\t');
//      Serial.print(throttle); Serial.print('\n');
  //
  //    Serial.print(functionTimeSum); Serial.print('\t');
  //    Serial.print(functionTimeCounter); Serial.print('\t');
  //    Serial.print((float)functionTimeSum / functionTimeCounter); Serial.print('\n');
  //    functionTimeSum = 0;
  //    functionTimeCounter = 0;
  //
  //    Serial.println(loopCounterRx); Serial.print('\t');
//      Serial.print(loopCounterRate); Serial.print('\t');
//      loopCounterRate = 0;
//      Serial.print(loopCounterAttitude); Serial.print('\n');
//      loopCounterAttitude = 0;
  //    loopCounter = 0;
  //
  //    Serial.print(throttle); Serial.print('\t');
  //    Serial.print(valGyX); Serial.print('\n');
//      printPackage();
//      Serial.print(motor1pulse); Serial.print('\t');
//      Serial.print(motor2pulse); Serial.print('\n');
//      Serial.print(motor3pulse); Serial.print('\t');
//      Serial.print(motor4pulse); Serial.print('\n');
//      Serial.print('\n');
  //
  //    Serial.println(maxLoopDuration);
  //    Serial.print(F("Outer loop: ")); Serial.print('\t');
  //    Serial.print(attitudeRollSettings.actual); Serial.print('\t');
//      Serial.print(attitudePitchSettings.actual); Serial.print('\t');
  //    Serial.print(attitudeYawSettings.actual); Serial.print('\t');
  //    Serial.print(attitudeRollSettings.target); Serial.print('\t');
//      Serial.print(attitudePitchSettings.target); Serial.print('\t');
  //    Serial.print(attitudeYawSettings.target); Serial.print('\t');
  //    Serial.print(attitudeRollSettings.output); Serial.print('\t');
  //    Serial.print(attitudePitchSettings.output); Serial.print('\t');
  //    Serial.print(attitudeYawSettings.output); Serial.print('\n');
  //    Serial.print('\n');
  //    Serial.print(F("Inner loop: ")); Serial.print('\t');
//      Serial.print(rateRollSettings.actual); Serial.print('\t');
//      Serial.print(ratePitchSettings.actual); Serial.print('\t');
//      Serial.print(rateYawSettings.actual); Serial.print('\t');
  //    Serial.print(rateRollSettings.target); Serial.print('\t');
  //    Serial.print(ratePitchSettings.target); Serial.print('\t');
//      Serial.print(rateYawSettings.target); Serial.print('\t');
  //    Serial.print(rateRollSettings.output); Serial.print('\t');
  //    Serial.print(ratePitchSettings.output); Serial.print('\t');
//      Serial.print(rateYawSettings.output); Serial.print('\n');
//      Serial.print(currentAngles.roll); Serial.print('\t');
//      Serial.print(currentAngles.pitch); Serial.print('\t');
//      Serial.print(currentAngles.yaw); Serial.print('\t');
  //    Serial.print(gyroChangeAngles.roll); Serial.print('\t');
  //    Serial.print(gyroChangeAngles.pitch); Serial.print('\t');
  //    Serial.print(gyroChangeAngles.yaw); Serial.print('\t');
  //

//      Serial.print(currentAngles.pitch); Serial.print('\t');
//      Serial.print(accelAngles.pitch); Serial.print('\t');
//      Serial.print(gyroAngles.pitch); Serial.print('\t');
//      Serial.print('\n');
//    }


} // END LOOP
