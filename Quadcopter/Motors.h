// ****************************************************************************************
// Interupt routine originally based on the standard Servo library
// ****************************************************************************************

#include <avr/interrupt.h>
#include <Arduino.h>

const uint16_t REFRESH_INTERVAL_TICKS = 20000;  // how often ESC pulses will be sent //20000 ticks = 10000us = 10ms <=> 100Hz (assumes prescaler of 8)
int volatile escTicks[5]; // not certain why needs to be volatile but pulses don't work without this
byte escIndex = 0;

const byte pinMotor1 = 3; // front left (CW)
const byte pinMotor2 = 6; // front right (CCW)
const byte pinMotor3 = 4; // back left (CCW)
const byte pinMotor4 = 5; // back right (CW)

int motor1pulse;
int motor2pulse;
int motor3pulse;
int motor4pulse;

const int ZERO_THROTTLE = 1000;
const int THROTTLE_MIN_SPIN = 1125;



// ****************************************************************************************
//        FUNCTIONS FOR ESC PULSE CREATION
// ****************************************************************************************

// configure timer1 for generating pulses for the ESCs
static void setupPulseTimer() {
  TCCR1A = 0;             // normal counting mode
  TCCR1B = _BV(CS11);     // set prescaler of 8 - 2 ticks per microsecond
  TCNT1 = 0;              // clear the timer count
  TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
  TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt
  sei(); // enable interrupts
  OCR1A = REFRESH_INTERVAL_TICKS;
}

static void endPulseTimer() {
  cli();
  TIMSK1 =  0 ; // disable the output compare interrupt
  TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
  sei();
}

static inline void generate_esc_pulses() {
  if (escIndex == 0) {
    TCNT1 = 0; // index set to 0 to indicate that refresh interval completed so reset the timer
  }
  else {  // interupt has fired and we're not yet at end of pulse sequence
    if (escIndex == 1) {
      PORTD &= B11110111; // set bit/pin 3 LOW
    }
    if (escIndex == 2) {
      PORTD &= B10111111; // set bit/pin 6 LOW
    }
    if (escIndex == 3) {
      PORTD &= B11101111; // set bit/pin 4 LOW
    }
    if (escIndex == 4) {
      PORTD &= B11011111; // set bit/pin 5 LOW
    }
  }

  escIndex++;    // increment to the next esc
  if (escIndex < 5) { // still in range of our 4 escs
    OCR1A = TCNT1 + escTicks[escIndex]; // set the compare register to the pulse length for the next ESC plus the current time
    if (escIndex == 1) {
      PORTD |= B00001000;
    }
    if (escIndex == 2) {
      PORTD |= B01000000;
    }
    if (escIndex == 3) {
      PORTD |= B00010000;
    }
    if (escIndex == 4) {
      PORTD |= B00100000;
    }
  }
  else {
    // finished all channels so wait for the refresh period to expire before starting over
    // if there's any chance we could have gone over this value already then would need to check if so
    // but here the max time the pulses will take is 4*2000 micros (plus some small overhead)
    // and if our refresh interval is 10000 micros then we have approx ~2000 micros of spare time
    // if I want to reduce refresh interval below 10000micros (10ms<=>100Hz) then may need to do this check
    OCR1A = REFRESH_INTERVAL_TICKS;
    escIndex = 0; // reset back to beginning
  }
}

ISR(TIMER1_COMPA_vect) {
  generate_esc_pulses();
}



// ****************************************************************************************
//        FUNCTIONS FOR CALCULATING ESC PULSE LENGTH
// ****************************************************************************************

void calculateMotorInput(int *throttle, float *rollOffset, float *pitchOffset, float *yawOffset) {
  motor1pulse = *throttle + (int) * rollOffset - (int) * pitchOffset - (int) * yawOffset;
  motor2pulse = *throttle - (int) * rollOffset - (int) * pitchOffset + (int) * yawOffset;
  motor3pulse = *throttle + (int) * rollOffset + (int) * pitchOffset + (int) * yawOffset;
  motor4pulse = *throttle - (int) * rollOffset + (int) * pitchOffset - (int) * yawOffset;
}

void capMotorInputNearMaxThrottle() {
  int maxMotorValue = max(motor1pulse, max(motor2pulse, max(motor3pulse, motor4pulse)));
  int adj = maxMotorValue - THROTTLE_LIMIT;
  if (adj > 0) {
    motor1pulse -= adj;
    motor2pulse -= adj;
    motor3pulse -= adj;
    motor4pulse -= adj;
  }
}

void capMotorInputNearMinThrottle() {
  //at the very least, stop pulse going below 1000
  // this could be a problem if trying to control at lowish throttle (if higher than 1000 set as min)
  if (throttle < THROTTLE_MIN_SPIN) { //this is the base throttle i.e. if stick is low, no motor should move even if there's a big movements
    motor1pulse = ZERO_THROTTLE;
    motor2pulse = ZERO_THROTTLE;
    motor3pulse = ZERO_THROTTLE;
    motor4pulse = ZERO_THROTTLE;
  }
  else {
    if (motor1pulse < THROTTLE_MIN_SPIN) motor1pulse = THROTTLE_MIN_SPIN;
    if (motor2pulse < THROTTLE_MIN_SPIN) motor2pulse = THROTTLE_MIN_SPIN;
    if (motor3pulse < THROTTLE_MIN_SPIN) motor3pulse = THROTTLE_MIN_SPIN;
    if (motor4pulse < THROTTLE_MIN_SPIN) motor4pulse = THROTTLE_MIN_SPIN;
  }
}

void updateMotors() {
  escTicks[1] = motor1pulse << 1;  // multiply by 2 // 2 ticks per microsecond // compiler probably does bitshift anyway
  escTicks[2] = motor2pulse << 1;
  escTicks[3] = motor3pulse << 1;
  escTicks[4] = motor4pulse << 1;
}

// ****************************************************************************************
//        SETUP FOR MAIN FILE
// ****************************************************************************************

void setupMotors() {
  pinMode(pinMotor1, OUTPUT);
  pinMode(pinMotor2, OUTPUT);
  pinMode(pinMotor3, OUTPUT);
  pinMode(pinMotor4, OUTPUT);
  escTicks[1] = 2000;  // set starting pulse to 0.4micros (out of ESC range)
  escTicks[2] = 2000;
  escTicks[3] = 2000;
  escTicks[4] = 2000;
  setupPulseTimer();
}

// ****************************************************************************************
//        FOR TESTING
// ****************************************************************************************


void setMotorsLow() {
  escTicks[1] = 2000;  // = 1000us
  escTicks[2] = 2000;
  escTicks[3] = 2000;
  escTicks[4] = 2000;
}
void setMotorsHigh() {
  escTicks[1] = 4000;  // = 1000us
  escTicks[2] = 4000;
  escTicks[3] = 4000;
  escTicks[4] = 4000;
}
void setMotorsCustom(int input) {
  escTicks[1] = input << 1;  // input * 2
  escTicks[2] = input << 1;
  escTicks[3] = input << 1;
  escTicks[4] = input << 1;
}

