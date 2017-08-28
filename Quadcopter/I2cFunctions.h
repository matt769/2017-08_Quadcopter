
boolean i2cTimeout;

void setupI2C(){
  I2c.begin();
  I2c.timeOut(50);
  I2c.setSpeed(1);  // fast (400Hz)
}



// ADD GENERAL FUNCTION TO READ MULTIPLE REGISTERS


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



void writeRegister(byte address, byte sensorRegister, byte data) {
 i2cTimeout = I2c.write(address,sensorRegister,data); // start transmission to device
} 

byte readRegister(byte address, byte sensorRegister) {
 i2cTimeout = I2c.read(address, sensorRegister, 1);
 return I2c.receive();
} 


// there's already a write bits function which has been used instead of my own???
void writeBitsNew(byte address, byte registerToWrite,byte startingReplacementBit, byte noReplacementBits, byte replacementValue){
  byte originalregisterValue = readRegister(address,registerToWrite);
  byte newRegisterValue = modifyBits(originalregisterValue,startingReplacementBit,noReplacementBits,replacementValue);
  writeRegister(address,registerToWrite,newRegisterValue);
}


void writeBitsNew2(byte address, byte registerToWrite, byte originalReading, byte startingReplacementBit, byte noReplacementBits, byte replacementValue){
  byte newRegisterValue = modifyBits(originalReading,startingReplacementBit,noReplacementBits,replacementValue);
  writeRegister(address,registerToWrite,newRegisterValue);
}

int readBitsNew(byte address, byte registerToRead,byte startingBit, byte noOfBits){
  byte registerVal = readRegister(address,registerToRead);
  
  // create mask that will just have 1s in bits of interest
  // apply mask to read value
  //return
  return registerVal; //placeholder
}



void flushI2cBuffer(){
  while(I2c.available()){
    I2c.receive();
  }
}



