

// THIS NEEDS TO GO IN MAIN FILE (or does it?)
byte addresses[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // receiver should be 1 (or anythin not 0)
RF24 radio(8, 7); // CE, CSN (SPI SS) *** WILL NEED TO BE UPDATED***



void setupRadio() {
  // RADIO
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);  // MIN, LOW, HIGH, MAX
  // Open a writing and reading pipe on each radio, MUST BE OPPOSITE addresses to the receiver
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
}





// acknowledgements
