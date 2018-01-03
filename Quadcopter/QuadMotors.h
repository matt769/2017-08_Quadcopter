// Static class

#ifndef QUADMOTORS_H
#define QUADMOTORS_H

#include <Arduino.h> // required for some datatypes e.g. uint16_t

class QuadMotors {
  public:
    static uint16_t escTicks[4];
    static uint16_t escTicksEndMain[4];
    static uint16_t volatile escTicksEndIsr[4];  // copy to use in IRS
    static uint8_t escOrderMain[4];
    static uint8_t volatile escOrderIsr[4];  // copy to use in IRS
    static uint8_t escIndex;
    static bool volatile lockPulses;
    static bool needUpdatePulses; // this needs to be set to true after the rate PID runs
    static bool needRecalcPulses;  // this needs to be set to true after the new pulse information is calulated
    static uint8_t escPulseGenerationCycle;  // 0 = start, 1 = stop, 2 = reset
    static uint16_t motorPulseLengthMs[4];

    QuadMotors();
    static void setupPulseTimer();
    static void endPulseTimer();
    static inline void generateEscPulses();
    static void calculateMotorInput(uint16_t throttle, float rollOffset, float pitchOffset, float yawOffset);
    static void capMotorInputNearMaxThrottle();
    static void capMotorInputNearMinThrottle(uint16_t throttle);
    static void copyPulseInfoToIsrVariables();
    static void calculateRequiredTicks();
    static void resetOrder();
    static void sortPulses();
    static void calcEndTimes();
    static void setMotorsLow();
    static void updateMotors(bool needRecalcPulses);
    static void setupMotors();

};
#endif
