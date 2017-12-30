#include "IMU.h"
#include "PIDManager.h"
#include "QuadMotors.h"
#include "Parameters.h"
#include "Receiver.h"

uint16_t throttle;

uint8_t attitude_mode = false;  // REMOVE THIS AND JUST USE STATE (actually change STATE to MODE)
uint8_t auto_level = false;

uint8_t RATE = 0;
uint8_t BALANCE = 1;
uint8_t MODE = RATE; // MODE IS ONLY FOR RATE or ATTITUDE
uint8_t PREV_MODE = RATE;

bool KILL = 0;

uint32_t rateLoopLast = 0;
uint32_t attitudeLoopLast = 0;
uint32_t receiverLast = 0;
uint32_t batteryLoopLast = 0;

unsigned long lastPrint = 0;  // FOR DEBUGGING
uint16_t loopCounterRate = 0;  // DEBUGGING
uint16_t loopCounterAttitude = 0;  // DEBUGGING

IMU imu;
Receiver rx(radio_ce_pin, radio_csn_pin);

// forward declaration
extern struct pid rateRollSettings;
extern struct pid ratePitchSettings;
extern struct pid rateYawSettings;
extern struct pid attitudeRollSettings;
extern struct pid attitudePitchSettings;
extern struct pid attitudeYawSettings;


void setup() {
  Serial.begin(115200);
  Serial.println(F("Setup"));
  pinMode(pinStatusLed, OUTPUT);
  digitalWrite(pinStatusLed, HIGH);
  imu.sensorOn();
  rx.setupRadio();
  setupPid();
  imu.initialiseCurrentAngles();
  rx.ArmingProcedure(); // requires radio connection and stick high, then low
  digitalWrite(pinStatusLed, LOW);
  QuadMotors::setupMotors();
  rx.checkHeartbeat();
  pidRateModeOn();
  unsigned long startTime = millis();
  rateLoopLast = startTime;
  attitudeLoopLast = startTime;
  receiverLast = startTime;
  batteryLoopLast = startTime;
}


void loop() {

  // ****************************************************************************************
  // CHECK FOR USER INPUT
  // ****************************************************************************************
  if (millis() - receiverLast >= receiverFreq) {
    receiverLast += receiverFreq;
    //    loopCounterRx++;
    rx.checkHeartbeat();  // must be done outside if(radio.available) loop
    // we don't need to bother doing the rest of this stuff if there's no actual input
    if (rx.checkRadioForInput()) {
      // CHECK MODES
      attitude_mode = rx.getMode();
      auto_level = rx.getAutolevel();
      if (rx.getKill() && (throttle < 1050)) {
        QuadMotors::setMotorsLow();
        digitalWrite(pinStatusLed, HIGH);
        while (1);

      }
      // MAP CONTROL VALUES
      rx.mapThrottle(&throttle);
      if (attitude_mode || auto_level) {
        rx.mapRcToPidInput(&attitudeRollSettings.target, &attitudePitchSettings.target, &attitudeYawSettings.target, attitude_mode);
        MODE = BALANCE;
      }
      else {
        rx.mapRcToPidInput(&rateRollSettings.target, &ratePitchSettings.target, &rateYawSettings.target, attitude_mode);
        MODE = RATE;
      }
    }
    auto_level = auto_level || !rx.checkHeartbeat();;
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
    imu.updateRate();
    setRatePidActual(imu.gyroValues[0], imu.gyroValues[1], imu.gyroValues[2]);
    if (pidRateUpdate()) {
      QuadMotors::calculateMotorInput(throttle, rateRollSettings.output, ratePitchSettings.output, rateYawSettings.output);
      QuadMotors::capMotorInputNearMaxThrottle();
      QuadMotors::capMotorInputNearMinThrottle(throttle);
      QuadMotors::updateMotors(true);
    }
  }

  // ****************************************************************************************
  // UPDATE MOTORS
  // ****************************************************************************************
  QuadMotors::updateMotors(false); // try and update the esc pulses only in case it was locked previously


  // ****************************************************************************************
  // RUN ATTITUDE CALCULATIONS
  // ****************************************************************************************
  if (millis() - attitudeLoopLast >= attitudeLoopFreq) {
    attitudeLoopLast += attitudeLoopFreq;
    loopCounterAttitude++;
    imu.updateAttitude();
    // OVERRIDE PID SETTINGS IF TRYING TO AUTO-LEVEL
    if (auto_level) { // if no communication received, OR user has specified auto-level
      setAutoLevelTargets();
      // If connection lost then also change throttle so that QC is descending slowly
      if (!rx.checkHeartbeat()) {
        imu.calculateVerticalAccel();
        connectionLostDescend(&throttle, imu.accelValues[2]);
      }
    }
    // The attitude PID itself will not run unless QC is in ATTITUDE mode
    if (attitude_mode) {
      setAttitudePidActual(imu.currentAngles[0], imu.currentAngles[1], imu.currentAngles[2]);
      // if PID has updated the outputs then recalculate the required motor pulses
      if (pidAttitudeUpdate()) {
        // set rate setpoints
        setRatePidTargets(attitudeRollSettings.output, attitudePitchSettings.output, attitudeYawSettings.output);
        overrideYawTarget();  // OVERIDE THE YAW BALANCE PID OUTPUT
      }
    }
  }

  // ****************************************************************************************
  // DEBUGGING
  // ****************************************************************************************
//  if (millis() - lastPrint >= 50) {
//    lastPrint += 50;
    //    Serial.print(imu.currentAngles[1]); Serial.print('\t');
    //    Serial.print(imu.accelAngles[1]); Serial.print('\t');
    //    Serial.print(imu.gyroAngles[1]); Serial.print('\t');
    //    Serial.print('\n');
//    Serial.print(QuadMotors::escTicks[0]); Serial.print('\t');
//    Serial.print(QuadMotors::escTicks[1]); Serial.print('\t');
//    Serial.print(QuadMotors::escTicks[2]); Serial.print('\t');
//    Serial.print(QuadMotors::escTicks[3]); Serial.print('\n');
//  }

}

