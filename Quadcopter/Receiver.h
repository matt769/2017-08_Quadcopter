// ADD CONDITION ON MODE TO mapToPidInput()
// change mapping depending on mode?

// Control byte
// bit 0; 1 = Stick 1 button pressed
// bit 1: 1 = Stick 2 button pressed
// bit 2: 1 = Attitude mode, 0 = rate mode
// bit 3; 1 = auto-level on, 0 = auto-level off


// radio parameters - retransmits etc



byte addresses[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // receiver should be 1 (or anything not 0)
RF24 radio(8, 7); // CE, CSN (SPI SS) *** WILL NEED TO BE UPDATED***

byte statusForAcknowledgement; // send this back to transmitter as acknowledgement package
const byte OK = 1;
const byte GENERAL_ERROR = 2;
const byte BATTERY_LOW = 3;

// Bits 5,6,7 show battery level (that gives 8 segments)


struct dataStruct {
  int throttle; // number 1000 to 2000
  int roll;     // number 1000 to 2000
  int pitch;    // number 1000 to 2000
  int yaw;     // number 1000 to 2000
  byte control; // for some control bits
  byte checksum;
};

struct dataStruct rcPackage;

float rateMin = -10;  // DEGREES/SECOND
float rateMax = 10;  // DEGREES/SECOND
float attitudeMin = -20;  // DEGREES
float attitudeMax = 20;  // DEGREES

bool rxHeartbeat = false;
unsigned long lastRxReceived = 0;
unsigned long heartbeatTimeout = 1000;

void setupRadio() {
  // RADIO
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);  // MIN, LOW, HIGH, MAX
  // Open a writing and reading pipe on each radio, MUST BE OPPOSITE addresses to the receiver
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  radio.startListening();
}

bool checkRadioForInput() {
  if ( radio.available()) {
    while (radio.available()) {
      radio.read( &rcPackage, sizeof(rcPackage) );
    }
    lastRxReceived = millis();
    return true;
  }
  return false;
}

bool checkHeartbeat(){
  if(millis()-lastRxReceived > heartbeatTimeout){
    rxHeartbeat = false;
  }
  else {
    rxHeartbeat = true;
  }
  return rxHeartbeat;
}




bool checkRadioForInputPLACEHOLDER() {
  // ADD PLACEHOLDER VALUES
  rcPackage.throttle = 200;
  rcPackage.roll = 500;
  rcPackage.pitch = 500;
  rcPackage.yaw = 500;
  rcPackage.control = B00000000;
  rxHeartbeat = true;
  lastRxReceived = millis();
  return true;
}


void mapRcToPidInput(int *throttle, double *roll, double *pitch, double *yaw, bool *mode) {
  *throttle = map(rcPackage.throttle,0,1000,1000,2000); // should probably just provide it as the correct range

  if (!*mode) {
    *roll = (double)map(rcPackage.roll, 0, 1000, rateMin, rateMax);
    *pitch = (double)map(rcPackage.pitch, 0, 1000, rateMin, rateMax);
    *yaw = (double)map(rcPackage.yaw, 0, 1000, rateMin, rateMax);
  }
  else {
    *roll = (double)map(rcPackage.roll, 0, 1000, attitudeMin, attitudeMax);
    *pitch = (double)map(rcPackage.pitch, 0, 1000, attitudeMin, attitudeMax);
    *yaw = (double)map(rcPackage.yaw, 0, 1000, attitudeMin, attitudeMax);
  }

}

bool getSomething() {
  return bitRead(rcPackage.control, 0);
}

bool getSomethingElse() {
  return bitRead(rcPackage.control, 1);
}

bool getMode() {
  return bitRead(rcPackage.control, 2);
}

bool getAutolevel() {
  return bitRead(rcPackage.control, 3);
}

