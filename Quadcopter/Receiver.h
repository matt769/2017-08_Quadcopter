

byte addresses[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // receiver should be 1 (or anything not 0)
RF24 radio(8, 7); // CE, CSN (SPI SS) *** WILL NEED TO BE UPDATED***

byte statusForAcknowledgement; // send this back to transmitter as acknowledgement package

struct dataStruct {
  int throttle; // number 1000 to 2000
  int pitch;    // number 1000 to 2000
  int roll;     // number 1000 to 2000
  int yaw;     // number 1000 to 2000
  byte control; // for some control bits
  byte checksum;
};

struct dataStruct rcPackage;

void setupRadio() {
  // RADIO
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);  // MIN, LOW, HIGH, MAX
  // Open a writing and reading pipe on each radio, MUST BE OPPOSITE addresses to the receiver
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  radio.startListening();
}

void checkRadioForInput() {
  if ( radio.available()) {
    while (radio.available()) {
      radio.read( &rcPackage, sizeof(rcPackage) );
    }
  }
}
