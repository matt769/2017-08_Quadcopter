#include "QuadMotors.h"
#include "Parameters.h"

static uint16_t QuadMotors::escTicks[4] = {0, 0, 0, 0};
static uint16_t QuadMotors::escTicksEndMain[4] = {0, 0, 0, 0};
static uint16_t volatile QuadMotors::escTicksEndIsr[4] = {0, 0, 0, 0};
static uint8_t QuadMotors::escOrderMain[4] = {0, 0, 0, 0};
static uint8_t volatile QuadMotors::escOrderIsr[4] = {0, 0, 0, 0};
static uint8_t QuadMotors::escIndex = 0;
static bool volatile QuadMotors::lockPulses = false;
static bool QuadMotors::needUpdatePulses = false;
static uint8_t QuadMotors::escPulseGenerationCycle = 2;  // 0 = start, 1 = stop, 2 = reset
static uint16_t QuadMotors::motorPulseLengthMs[4] = {0, 0, 0, 0};


QuadMotors::QuadMotors() {
}

static void QuadMotors::calculateMotorInput(uint16_t throttle, float rollOffset, float pitchOffset, float yawOffset) {
  motorPulseLengthMs[0] = throttle + (uint16_t) rollOffset - (uint16_t) pitchOffset - (uint16_t) yawOffset;
  motorPulseLengthMs[1] = throttle - (uint16_t) rollOffset - (uint16_t) pitchOffset + (uint16_t) yawOffset;
  motorPulseLengthMs[2] = throttle + (uint16_t) rollOffset + (uint16_t) pitchOffset + (uint16_t) yawOffset;
  motorPulseLengthMs[3] = throttle - (uint16_t) rollOffset + (uint16_t) pitchOffset - (uint16_t) yawOffset;

}

static void QuadMotors::capMotorInputNearMaxThrottle() {
  uint16_t maxMotorValue = max(motorPulseLengthMs[0], max(motorPulseLengthMs[1], max(motorPulseLengthMs[2], motorPulseLengthMs[3])));
  int16_t adj = maxMotorValue - THROTTLE_LIMIT;
  if (adj > 0) {
    motorPulseLengthMs[0] -= adj;
    motorPulseLengthMs[1] -= adj;
    motorPulseLengthMs[2] -= adj;
    motorPulseLengthMs[3] -= adj;
  }
}

static void QuadMotors::capMotorInputNearMinThrottle(uint16_t throttle) {
  //at the very least, stop pulse going below 1000
  // this could be a problem if trying to control at lowish throttle (if higher than 1000 set as min)
  if (throttle < THROTTLE_MIN_SPIN) { //this is the base throttle i.e. if stick is low, no motor should move even if there's a big movements
    motorPulseLengthMs[0] = ZERO_THROTTLE;
    motorPulseLengthMs[1] = ZERO_THROTTLE;
    motorPulseLengthMs[2] = ZERO_THROTTLE;
    motorPulseLengthMs[3] = ZERO_THROTTLE;
  }
  else {
    if (motorPulseLengthMs[0] < THROTTLE_MIN_SPIN) motorPulseLengthMs[0] = THROTTLE_MIN_SPIN;
    if (motorPulseLengthMs[1] < THROTTLE_MIN_SPIN) motorPulseLengthMs[1] = THROTTLE_MIN_SPIN;
    if (motorPulseLengthMs[2] < THROTTLE_MIN_SPIN) motorPulseLengthMs[2] = THROTTLE_MIN_SPIN;
    if (motorPulseLengthMs[3] < THROTTLE_MIN_SPIN) motorPulseLengthMs[3] = THROTTLE_MIN_SPIN;
  }
}


// ****************************************************************************************
//        FUNCTIONS FOR ESC CREATION
// ****************************************************************************************

ISR(TIMER1_COMPA_vect) {
  QuadMotors::generateEscPulses();
}

// configure timer1 for generating pulses for the ESCs
static void QuadMotors::setupPulseTimer() {
  cli();
  TCCR1A = 0;             // normal counting mode
  TCCR1B = _BV(CS11);     // set prescaler of 8 - 2 ticks per microsecond
  TCNT1 = 0;              // clear the timer count
  TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
  TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt
  OCR1A = CYCLE_TICKS;
  sei(); // enable interrupts
}

static void QuadMotors::endPulseTimer() {
  cli();
  TIMSK1 =  0 ; // disable the output compare interrupt
  TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
  sei();
}

static void QuadMotors::generateEscPulses() {

  if (escPulseGenerationCycle == 0) {  // interupt has fired and we're starting the pulses
    uint8_t currentEsc = escOrderIsr[escIndex];
    if (currentEsc == 1) PORTD |= B00001000; // set bit/pin 3 HIGH
    if (currentEsc == 2) PORTD |= B01000000; // set bit/pin 6 HIGH
    if (currentEsc == 3) PORTD |= B00010000; // set bit/pin 4 HIGH
    if (currentEsc == 4) PORTD |= B00100000; // set bit/pin 5 HIGH
    escIndex++;
    if (escIndex > 3) {
      escIndex = 0;
      OCR1A = escTicksEndIsr[0]; // set to end time of first pulse
      escPulseGenerationCycle = 1;
    }
    else {
      OCR1A = escTicksStart[escIndex];  // next interrupt when the next pulse needs to start
    }
  }

  else if (escPulseGenerationCycle == 1) {  // interupt has fired and we're ending the pulses
    uint8_t currentEsc = escOrderIsr[escIndex];
    if (currentEsc == 1) PORTD &= B11110111;
    if (currentEsc == 2) PORTD &= B10111111;
    if (currentEsc == 3) PORTD &= B11101111;
    if (currentEsc == 4) PORTD &= B11011111;
    escIndex++;
    if (escIndex > 3) {
      OCR1A = CYCLE_TICKS;
      escIndex = 0;
      escPulseGenerationCycle = 2; // reset back to beginning
      lockPulses = false;   // it's now ok to update the pulse length
    }
    else {
      OCR1A = escTicksEndIsr[escIndex];
    }
  }

  else {  // i.e. escPulseGenerationCycle = 2;
    TCNT1 = 0;  // reset the timer
    OCR1A = PULSE_GAP;  // start again after the standard gap
    escPulseGenerationCycle = 0; // next time interupt fire we want to start the pulses
    lockPulses = true;  // this is the beginning of the pulse train, so don't allow the pulse lengths to be updated until this cycle is 'finished'
  }
}

static void QuadMotors::copyPulseInfoToIsrVariables() {
  cli();
  escOrderIsr[0] = escOrderMain[0];
  escOrderIsr[1] = escOrderMain[1];
  escOrderIsr[2] = escOrderMain[2];
  escOrderIsr[3] = escOrderMain[3];
  escTicksEndIsr[0] = escTicksEndMain[0];
  escTicksEndIsr[1] = escTicksEndMain[1];
  escTicksEndIsr[2] = escTicksEndMain[2];
  escTicksEndIsr[3] = escTicksEndMain[3];
  sei();
}

static void QuadMotors::calculateRequiredTicks() {
  escTicks[0] = motorPulseLengthMs[0] << 1;
  escTicks[1] = motorPulseLengthMs[1] << 1;
  escTicks[2] = motorPulseLengthMs[2] << 1;
  escTicks[3] = motorPulseLengthMs[3] << 1;
}

static void QuadMotors::resetOrder() {
  escOrderMain[0] = 1;
  escOrderMain[1] = 2;
  escOrderMain[2] = 3;
  escOrderMain[3] = 4;
}

static void QuadMotors::sortPulses() {
  uint16_t j;
  uint8_t jIdx;
  int16_t k;
  for (uint8_t i = 1; i < 4; ++i)
  {
    j = escTicks[i];
    jIdx = escOrderMain[i];
    k;
    for (k = i - 1; (k >= 0) && (j < escTicks[k]); k--)
    {
      escTicks[k + 1] = escTicks[k];
      escOrderMain[k + 1] = escOrderMain[k];
    }
    escTicks[k + 1] = j;
    escOrderMain[k + 1] = jIdx;
  }
}

static void QuadMotors::calcEndTimes() {
  escTicksEndMain[0] = escTicks[0] + PULSE_GAP;
  escTicksEndMain[1] = escTicks[1] + (2 * PULSE_GAP);
  escTicksEndMain[2] = escTicks[2] + (3 * PULSE_GAP);
  escTicksEndMain[3] = escTicks[3] + (4 * PULSE_GAP);
}

static void QuadMotors::updateMotors(bool needRecalcPulses) {
  if (needRecalcPulses) { // allow the calculation to start as soon as new values are available
    // calculate required counter ticks
    resetOrder(); // reset escOrderMain
    calculateRequiredTicks(); // populate escTicks
    sortPulses(); // reorder escOrderMain and escTicks
    calcEndTimes(); // what times should these finish - populate escTicksEndMain
    needUpdatePulses = true;
  }
  if (needUpdatePulses) { // but will only update them when variables are 'unlocked'
    cli();  // need to turn off interupts here or there is a risk that lockPulses changes state immediately after being checked
    if (!lockPulses) {
      copyPulseInfoToIsrVariables();
      needUpdatePulses = false;
    }
    sei();
  }
}


// ****************************************************************************************
//        SETUP FOR MAIN FILE
// ****************************************************************************************

static void QuadMotors::setupMotors() {
  escPulseGenerationCycle = 2;
  escIndex = 0;
  lockPulses = false;
  needUpdatePulses = false;
  //  needRecalcPulses = false;
  pinMode(motorPins[0], OUTPUT);
  pinMode(motorPins[1], OUTPUT);
  pinMode(motorPins[2], OUTPUT);
  pinMode(motorPins[3], OUTPUT);
  setMotorsLow();
  setupPulseTimer();
}

// ****************************************************************************************
//        OTHER
// ****************************************************************************************

static void QuadMotors::setMotorsLow() {
  motorPulseLengthMs[0] = 1000;
  motorPulseLengthMs[1] = 1000;
  motorPulseLengthMs[2] = 1000;
  motorPulseLengthMs[3] = 1000;
  calculateRequiredTicks();
  calcEndTimes();
  copyPulseInfoToIsrVariables();
}
