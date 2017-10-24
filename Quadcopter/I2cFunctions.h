// some of these functions aren't technically related to I2C, but that's what I'm using them for here
// review timeout

boolean i2cTimeout;

void setupI2C(){
  I2c.begin();
  I2c.timeOut(50);
  I2c.setSpeed(1);  // fast (400Hz)
}

// stardard write plus records if there was a timeout
void writeRegister(byte address, byte sensorRegister, byte data) {
 i2cTimeout = I2c.write(address,sensorRegister,data); // start transmission to device
} 

// stardard read plus records if there was a timeout
byte readRegister(byte address, byte sensorRegister) {
 i2cTimeout = I2c.read(address, sensorRegister, 1);
 return I2c.receive();
} 

byte modifyBits(byte originalByte, byte startingReplacementBit, byte noReplacementBits, byte replacementValue){
  byte clearMask;
  byte replaceMask;
  byte newByte;
  clearMask = (1<< noReplacementBits)-1;  // set required number of 1s
  clearMask = clearMask << startingReplacementBit; // move to required position
  clearMask = ~clearMask; // invert so that bits to overwrite are 0 in the mask
  newByte = originalByte & clearMask; // apply mask and clear bits of interest
  replaceMask = replacementValue << startingReplacementBit; // move replacement bits to correct position in mask
  newByte = newByte | replaceMask; // apply mask
  return newByte;
}

// there's already a 'writeBits' function which has been used instead of my own???
void writeBitsNew(byte address, byte registerToWrite,byte startingReplacementBit, byte noReplacementBits, byte replacementValue){
  byte originalRegisterValue = readRegister(address,registerToWrite);
  byte newRegisterValue = modifyBits(originalRegisterValue,startingReplacementBit,noReplacementBits,replacementValue);
  writeRegister(address,registerToWrite,newRegisterValue);
}

void writeBitsNew2(byte address, byte registerToWrite, byte originalRegisterValue, byte startingReplacementBit, byte noReplacementBits, byte replacementValue){
  byte newRegisterValue = modifyBits(originalRegisterValue,startingReplacementBit,noReplacementBits,replacementValue);
  writeRegister(address,registerToWrite,newRegisterValue);
}

// NEEDS TESTING
byte readBitsNew(byte address, byte registerToRead,byte startingBit, byte noOfBits){
  byte result = readRegister(address,registerToRead);
  byte readMask = (1<< noOfBits)-1;  // set required number of 1s
  readMask = readMask << startingBit; // move to required position
  result &= readMask;
  result = result >> startingBit; // move bits of interest to beginning
  return result;
}

void flushI2cBuffer(){
  while(I2c.available()){
    I2c.receive();
  }
}
