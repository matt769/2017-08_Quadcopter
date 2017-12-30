#include <I2C.h>  // http://dsscircuits.com/articles/86-articles/66-arduino-i2c-master-library
#include <SPI.h>  // standard Arduino DPI library
#include <RF24.h> // https://github.com/nRF24/RF24

#include "Parameters.h"
#include "PID.h"
#include "BatteryMonitor.h"
#include "I2cFunctions.h"
#include "MotionSensor.h"
#include "Receiver.h"
#include "Motors.h"
#include "PIDSettings.h"

// THROTTLE
int throttle;  // distinct from the user input because it may be modified

// STATE
const bool RATE = false;
const bool ATTITUDE = true;
bool mode = RATE; // MODE IS ONLY FOR RATE or ATTITUDE
bool previousMode = RATE;
bool autoLevel = false;
bool kill = 0;

// CONTROL LOOPS
unsigned long rateLoopLast = 0;
unsigned long attitudeLoopLast = 0;
unsigned long receiverLast = 0;
unsigned long batteryLoopLast = 0;

// DEBUGGING // PERFORMANCE CHECKING
unsigned long lastPrint = 0;
int loopCounter = 0;
int loopCounterRate = 0;
int loopCounterAttitude = 0;
unsigned long functionTimeSum = 0;
int functionTimeCounter = 0;
unsigned long tStart;
unsigned long tEnd;


// LED
const byte pinStatusLed = 8;

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
  initialiseCurrentAngles();
  // ARMING PROCEDURE
  // wait for radio connection and specific user input (stick up, stick down)
  while (!checkRadioForInput()) {
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
  checkHeartbeat(); // refresh
  pidRateModeOn();
  unsigned long startTime = millis();
  lastPrint = startTime;
  rateLoopLast = startTime;
  attitudeLoopLast = startTime;
  receiverLast = startTime;
  batteryLoopLast = startTime;
} // END SETUP



void loop() {

  loopCounter ++;

  // ****************************************************************************************
  // CHECK FOR USER INPUT AND SET MODE
  // ****************************************************************************************
  if (millis() - receiverLast >= receiverFreq) {
    receiverLast += receiverFreq;
    checkHeartbeat();  // must be done outside if(radio.available) loop
    if (checkRadioForInput()) {
      // CHECK MODES
      mode = getMode();
      autoLevel = getAutolevel();
      if (getKill() && (throttle < 1050)) {
        setMotorsLow();
        digitalWrite(pinStatusLed, HIGH);
        while (1);
      }
      // MAP CONTROL VALUES
      mapThrottle(&throttle);
      if (mode || autoLevel) {
        mapRcToPidInput(&attitudeRollSettings.target, &attitudePitchSettings.target, &attitudeYawSettings.target, mode);
        mode = ATTITUDE;
      }
      else {
        mapRcToPidInput(&rateRollSettings.target, &ratePitchSettings.target, &rateYawSettings.target, mode);
        mode = RATE;
      }
    }
    autoLevel = autoLevel || !rxHeartbeat;
    if (autoLevel) {
      mode = ATTITUDE;
    }
  }

  // ****************************************************************************************
  // HANDLE STATE CHANGES
  // ****************************************************************************************
  if (mode != previousMode) {
    if (mode == ATTITUDE) {
      pidAttitudeModeOn();
      previousMode = ATTITUDE;
    }
    else {
      pidAttitudeModeOff();
      previousMode = RATE;
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
    setRatePidActual(valGyX, valGyY, valGyZ);
    // if PID has updated the outputs then recalculate the required motor pulses
    if (pidRateUpdate()) {
      // calculate required pulse length
      calculateMotorInput(throttle, rateRollSettings.output, ratePitchSettings.output, rateYawSettings.output);
      capMotorInputNearMaxThrottle();
      capMotorInputNearMinThrottle(throttle);
      recalculateMotorPulses();
    }
    // required for attitude calculations
    accumulateGyroChange();
    accumulateAccelReadings(); 
  }

  updateMotorPulseISR(); // keep trying to update the actual esc pulses in the ISR in case it was locked previously

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
    if (autoLevel) { // if no communication received, OR user has specified auto-level
      setAutoLevelTargets();
      // If connection lost then also modify throttle so that QC is descending slowly
      if (!rxHeartbeat) {
        calculateVerticalAccel();
        connectionLostDescend(&throttle, valAcZ);
      }
    }
    // The attitude PID itself will not run unless QC is in ATTITUDE mode
    if (mode) {
      setAttitudePidActual(currentAngles.roll, currentAngles.pitch, currentAngles.yaw);
      // if PID has updated the outputs then recalculate the required motor pulses
      if (pidAttitudeUpdate()) {
        // set rate setpoints
        setRatePidTargets(attitudeRollSettings.output, attitudePitchSettings.output, attitudeYawSettings.output);
        overrideYawTarget();  // OVERIDE THE YAW BALANCE PID OUTPUT
      }
    }
  }


  //  updateMotors(); // update the actual esc pulses

  // ****************************************************************************************
  // CHECK BATTERY
  // ****************************************************************************************
  if (millis() - batteryLoopLast >= batteryFreq) {
    batteryLoopLast += batteryFreq;
    calculateBatteryLevel();
  }

  //  updateMotors(); // update the actual esc pulses

  // ****************************************************************************************
  // DEBUGGING
  // ****************************************************************************************

//  if (millis() - lastPrint >= 1000) {
//    lastPrint += 1000;

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
    //      Serial.print(autoLevel); Serial.print('\t');
    //      Serial.print(lastRxReceived); Serial.print('\t');
    //      Serial.print(mode); Serial.print('\t');
    //      Serial.print(throttle); Serial.print('\n');
    //
    
//    Serial.print(functionTimeSum); Serial.print('\t');
//    Serial.print(functionTimeCounter); Serial.print('\t');
//    Serial.print((float)functionTimeSum / functionTimeCounter); Serial.print('\n');
//    functionTimeSum = 0;
//    functionTimeCounter = 0;
    //
    //      Serial.println(loopCounterRx); Serial.print('\t');

    //          Serial.print(loopCounterRate); Serial.print('\t');
    //          loopCounterRate = 0;
    //          Serial.print(loopCounterAttitude); Serial.print('\n');
    //          loopCounterAttitude = 0;
    //    loopCounter = 0;
    //
    //    Serial.print(throttle); Serial.print('\t');
    //    Serial.print(valGyX); Serial.print('\n');
    //      printPackage();
    //        Serial.print(motor1pulse); Serial.print('\t');
    //        Serial.print(motor2pulse); Serial.print('\n');
    //        Serial.print(motor3pulse); Serial.print('\t');
    //        Serial.print(motor4pulse); Serial.print('\n');
    //        Serial.print('\n');
    //
    //        Serial.print(motor1pulse); Serial.print('\t');
    //        Serial.print(motor2pulse); Serial.print('\t');
    //        Serial.print(motor3pulse); Serial.print('\t');
    //        Serial.print(motor4pulse); Serial.print('\n');
    //        Serial.print(escOrderMain[0]); Serial.print('\t');
    //        Serial.print(escTicks[0]); Serial.print('\n');
    //        Serial.print(escOrderMain[1]); Serial.print('\t');
    //        Serial.print(escTicks[1]); Serial.print('\n');
    //        Serial.print(escOrderMain[2]); Serial.print('\t');
    //        Serial.print(escTicks[2]); Serial.print('\n');
    //        Serial.print(escOrderMain[3]); Serial.print('\t');
    //        Serial.print(escTicks[3]); Serial.print('\n');
    //        Serial.print('\n');

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

    //          Serial.print(GyY);Serial.print('\n');

    //          Serial.print(currentAngles.pitch); Serial.print('\t');
    //          Serial.print(accelAngles.pitch); Serial.print('\t');
    //          Serial.print(gyroAngles.pitch); Serial.print('\t');
    //          Serial.print('\n');
//  }


} // END LOOP
