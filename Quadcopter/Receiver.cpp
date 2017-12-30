#include "Receiver.h"
#include "Parameters.h"


static const uint8_t OK = 1;

Receiver::Receiver(uint16_t ce, uint16_t csn): radio(ce, csn) {
}

uint8_t Receiver::calculateCheckSum() {
  uint8_t sum = 0;
  sum += rcPackage.throttle;
  sum += rcPackage.pitch;
  sum += rcPackage.roll;
  sum += rcPackage.yaw;
  sum += rcPackage.control;
  sum += rcPackage.alive;
  sum = 1 - sum;
  return sum;
}

void Receiver::printPackage() {
  Serial.print(rcPackage.throttle); Serial.print('\t');
  Serial.print(rcPackage.roll); Serial.print('\t');
  Serial.print(rcPackage.pitch); Serial.print('\t');
  Serial.print(rcPackage.yaw); Serial.print('\t');
  Serial.print(rcPackage.control); Serial.print('\t');
  Serial.print(rcPackage.alive); Serial.print('\t');
  Serial.print(rcPackage.checksum); Serial.print('\t');
  Serial.print("CHKSUM_DIFF: "); Serial.println(rcPackage.checksum - calculateCheckSum());
}

// move the parameters here into the parameters file
void Receiver::setupRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);  // MIN, LOW, HIGH, MAX
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS); // slower is more reliable and gives longer range
  radio.openReadingPipe(pipeNumber, address);
  radio.startListening();
}

void Receiver::updateAckStatusForTx() {
  statusForAck = 0;
  statusForAck |= statusForAckExternal; // add information from oter sources
  statusForAck |= OK << 1; // obviously need to change if not ok // NOT YET IMPLEMENTED
  statusForAck |= 1; // set low bit to 1 always // just to distinguish from a value of zero
}

// make statusForAck modifyable by outside functions
// actually provide a separate variable for them to modify, then combine them
// currently this only support setting bits 5/6/7 all together
void Receiver::updateAckStatusExternal(uint8_t value) {
  //   statusForAckExternal &= 0b00011111;     // first clear existing bits
  statusForAckExternal = value << 5;
}

bool Receiver::checkRadioForInput() {
  if ( radio.available()) {
    while (radio.available()) {
      radio.read( &rcPackage, sizeof(rcPackage) );
    }
    // load acknowledgement payload for the next transmission (first transmission will not get any ack payload (but will get normal ack))
    radio.writeAckPayload(1, &statusForAck, sizeof(statusForAck));
    if (rcPackage.checksum != calculateCheckSum()) {
      radio.flush_rx();
      return false;
    }
    //    Serial.println(statusForAck);
    lastRxReceived = millis();
    radio.flush_rx(); // probably remove
    updateAckStatusForTx(); // for next time
    return true;
  }
  return false;
}

bool Receiver::checkHeartbeat() {
  if (millis() - lastRxReceived > heartbeatTimeout) {
    rxHeartbeat = false;
  }
  else {
    rxHeartbeat = true;
  }
  return rxHeartbeat;
}

////bool Receiver::checkRadioForInput() {
////  // PLACEHOLDER VALUES
////  rcPackage.throttle = 127;
////  rcPackage.roll = 127;
////  rcPackage.pitch = 127;
////  rcPackage.yaw = 127;
////  rcPackage.control = B00000000;  // B00000100; attitude mode
////  rxHeartbeat = true;
////  lastRxReceived = millis();
////  updateAckStatusForTx(); // for next time
////  return true;
////}

void Receiver::mapThrottle(int *throttle) {
  if (rcPackage.throttle < 12) {
    *throttle = 0;
  }
  else {
    *throttle = map(rcPackage.throttle, 0, 255, THROTTLE_MIN_SPIN, THROTTLE_LIMIT);
  }
}

// CONSIDER SPLITTING TO SEPARATE FUNCTIONS? NOT SURE MUCH ADVANTAGE LIKE THIS
// UNLESS I PASS THE min and max as well?
void Receiver::mapRcToPidInput(float *roll, float *pitch, float *yaw, bool mode) {
  if (!mode) {
    *roll = (float)map(rcPackage.roll + 1, 0, 255, rateMin, rateMax);
    *pitch = (float)map(rcPackage.pitch + 1, 0, 255, rateMin, rateMax);
    *yaw = (float)map(rcPackage.yaw + 1, 0, 255, rateMax, rateMin);
  }
  else {
    *roll = (float)map(rcPackage.roll + 1, 0, 255, attitudeMin, attitudeMax);
    *pitch = (float)map(rcPackage.pitch + 1, 0, 255, attitudeMin, attitudeMax);
    *yaw = (float)map(rcPackage.yaw + 1, 0, 255, attitudeMin, attitudeMax);
  }
}

bool Receiver::getMode() {
  return bitRead(rcPackage.control, 2);
}
bool Receiver::getAutolevel() {
  return bitRead(rcPackage.control, 3);
}
bool Receiver::getKill() {
  return bitRead(rcPackage.control, 7);
}


void Receiver::ArmingProcedure() {
  while (!checkRadioForInput()) {
  }
  while (rcPackage.throttle < 200) {
    checkRadioForInput();
  }
  while (rcPackage.throttle > 50) {
    checkRadioForInput();
  }
}







