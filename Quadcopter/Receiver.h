// ADD CONDITION ON MODE TO mapToPidInput()
// change mapping depending on mode?


byte addresses[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // receiver should be 1 (or anything not 0)
RF24 radio(8, 7); // CE, CSN (SPI SS) *** WILL NEED TO BE UPDATED***

byte statusForAcknowledgement; // send this back to transmitter as acknowledgement package

struct dataStruct {
  int throttle; // number 1000 to 2000
  int roll;     // number 1000 to 2000
  int pitch;    // number 1000 to 2000
  int yaw;     // number 1000 to 2000
  byte control; // for some control bits
  byte checksum;
};

struct dataStruct rcPackage;

float rateMin = -10;
float rateMax = 10;
float balanceMin = -20;
float balanceMax = 20;



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
   return true;
  }
  return false;
}

bool checkRadioForInputPLACEHOLDER() {
  // ADD PLACEHOLDER VALUES
  rcPackage.throttle = 500;
  rcPackage.roll = 500;
  rcPackage.pitch = 500;
  rcPackage.yaw = 500;
  return true;
}


void mapRcToPidInput(int *throttle, double *roll, double *pitch, double *yaw, bool *mode){
    *throttle = rcPackage.throttle;

  if(!*mode){
    *roll = (double)map(rcPackage.roll,0,1000,rateMin,rateMax);
    *pitch = (double)map(rcPackage.pitch,0,1000,rateMin,rateMax);
    *yaw = (double)map(rcPackage.yaw,0,1000,rateMin,rateMax);
  }
  else {
    // REMEMBER THIS IS DEG/S
    *roll = (double)map(rcPackage.roll,0,1000,balanceMin,balanceMax);
    *pitch = (double)map(rcPackage.pitch,0,1000,balanceMin,balanceMax);
    *yaw = (double)map(rcPackage.yaw,0,1000,balanceMin,balanceMax);
  }

}


