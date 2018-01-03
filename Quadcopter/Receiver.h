#ifndef RECEIVER_H
#define RECEIVER_H

#include <RF24.h>

// consider moving some functions out of this class e.g. the mapping of inputs

class Receiver {
  public:
    uint8_t statusForAckExternal;
    Receiver(uint16_t ce, uint16_t csn);
    void printPackage();
    void setupRadio();
    void updateAckStatusExternal(uint8_t value);
    bool checkRadioForInput();
    bool checkHeartbeat();
    void mapThrottle(int *throttle);
    void mapRcToPidInput(float *roll, float *pitch, float *yaw, bool mode);
    bool getMode();
    bool getAutolevel();
    bool getKill();
    void ArmingProcedure();

  private:
    uint8_t statusForAck;
    bool rxHeartbeat;
    RF24 radio;
    uint8_t calculateCheckSum();
    void updateAckStatusForTx();
    unsigned long lastRxReceived;
    struct RCPackage {
      uint16_t throttle; // number 0 to 255  // TODO change to byte ON BOTH TX and RX
      uint16_t roll;     // number 0 to 255
      uint16_t pitch;    // number 0 to 255
      uint16_t yaw;     // number 0 to 255
      uint8_t control; // for some control bits
      uint8_t alive; //this will increment every time the data is sent
      uint8_t checksum;
    } rcPackage;

};

#endif
