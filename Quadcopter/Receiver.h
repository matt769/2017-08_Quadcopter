// Control byte
// bit 0;
// bit 1:
// bit 2: 1 = Attitude mode, 0 = rate mode
// bit 3: 1 = auto-level on, 0 = auto-level off
// bit 4:
// bit 5:
// bit 6:
// bit 7; 1 = TURN OFF MOTORS - reset required for re-enable // not implemented on current Tx

// Acknowledgement byte
// bit 0: non-zero // for Tx to easily distinguish from no acknowledgement
// bit 1: 1 = OK
// bit 2: 1 = some error
// bit 3:
// bit 4:
// bits 5/6/7: battery indicator (0-7)

const byte address[6] = "1Node";
const byte pipeNumber = 1;
RF24 radio(9, 10); // CE, CSN (SPI SS)

byte statusForAck = 0; // send this back to transmitter as acknowledgement package
const byte OK = 1;

struct dataStruct {
  byte throttle;
  byte roll;
  byte pitch;
  byte yaw;
  byte control; // for some control bits
  byte alive; //this will increment every time the data is sent
  byte checksum;
} rcPackage;

const float rateMin = -120;  // DEGREES/SECOND
const float rateMax = 120;  // DEGREES/SECOND
const float attitudeMin = -30;  // DEGREES
const float attitudeMax = 30;  // DEGREES

bool rxHeartbeat = false;
unsigned long lastRxReceived = 0;
const unsigned long heartbeatTimeout = 500;

byte calculateCheckSum() {
  byte sum = 0;
  sum += rcPackage.throttle;
  sum += rcPackage.pitch;
  sum += rcPackage.roll;
  sum += rcPackage.yaw;
  sum += rcPackage.control;
  sum += rcPackage.alive;
  sum = 1 - sum;
  return sum;
}

void printPackage() {
  Serial.print(rcPackage.throttle); Serial.print('\t');
  Serial.print(rcPackage.roll); Serial.print('\t');
  Serial.print(rcPackage.pitch); Serial.print('\t');
  Serial.print(rcPackage.yaw); Serial.print('\t');
  Serial.print(rcPackage.control); Serial.print('\t');
  Serial.print(rcPackage.alive); Serial.print('\t');
  Serial.print(rcPackage.checksum); Serial.print('\t');
  Serial.print("CHKSUM_DIFF: "); Serial.println(rcPackage.checksum - calculateCheckSum());
}

void setupRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);  // MIN, LOW, HIGH, MAX
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  // RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps // slower is more reliable and gives longer range
  radio.setDataRate(RF24_250KBPS);
  //   * @param delay How long to wait between each retry, in multiples of 250us,
  //   * max is 15.  0 means 250us, 15 means 4000us.
  //   * @param count How many retries before giving up, max 15
  //  radio.setRetries();   // default is setRetries(5,15) // note restrictions due to ack payload
  radio.openReadingPipe(pipeNumber, address);
  radio.startListening();
}

void updateAckStatusForTx() {
  statusForAck = 0;
  statusForAck |= batteryLevel << 5;
  statusForAck |= OK << 1; // obviously need to change if not ok
  statusForAck |= 1; // set low bit to 1 always
}

bool checkRadioForInput() {
  if ( radio.available()) {
    radio.read( &rcPackage, sizeof(rcPackage) );
    // load acknowledgement payload for the next transmission (first transmission will not get any ack payload (but will get normal ack))
    radio.writeAckPayload(1, &statusForAck, sizeof(statusForAck));
    if (rcPackage.checksum != calculateCheckSum()) {
      radio.flush_rx();
      return false;
    }
    lastRxReceived = millis();
    radio.flush_rx();
    updateAckStatusForTx(); // for next time
    return true;
  }
  return false;
}

bool checkHeartbeat() {
  if (millis() - lastRxReceived > heartbeatTimeout) {
    rxHeartbeat = false;
  }
  else {
    rxHeartbeat = true;
  }
  return rxHeartbeat;
}

void mapThrottle(int *throttle) {
  if (rcPackage.throttle < 12) {
    *throttle = 0;
  }
  else {
    *throttle = map(rcPackage.throttle, 0, 255, THROTTLE_MIN_SPIN, THROTTLE_LIMIT);
  }
}

void mapRcToPidInput(float *roll, float *pitch, float *yaw, bool mode) {
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

bool getMode() {
  return bitRead(rcPackage.control, 2);
}
bool getAutolevel() {
  return bitRead(rcPackage.control, 3);
}
bool getKill() {
  return bitRead(rcPackage.control, 7);
}

