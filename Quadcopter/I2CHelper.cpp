#include "I2CHelper.h"

I2CHelper::I2CHelper() {
  begin();
  timeOut(50);
  setSpeed(1);  // fast (400Hz)
}

// standard read plus records if there was a timeout
uint8_t I2CHelper::readRegister(uint8_t address, uint8_t sensorRegister) {
  i2cTimeout = read(address, sensorRegister, 1);
  return receive();
}

// stardard write plus records if there was a timeout
void I2CHelper::writeRegister(uint8_t address, uint8_t sensorRegister, uint8_t data) {
 i2cTimeout = write(address,sensorRegister,data); // start transmission to device
} 

uint8_t I2CHelper::modifyBits(uint8_t originaluint8_t, uint8_t startingReplacementBit, uint8_t noReplacementBits, uint8_t replacementValue){
  uint8_t clearMask;
  uint8_t replaceMask;
  uint8_t newuint8_t;
  clearMask = (1<< noReplacementBits)-1;  // set required number of 1s
  clearMask = clearMask << startingReplacementBit; // move to required position
  clearMask = ~clearMask; // invert so that bits to overwrite are 0 in the mask
  newuint8_t = originaluint8_t & clearMask; // apply mask and clear bits of interest
  replaceMask = replacementValue << startingReplacementBit; // move replacement bits to correct position in mask
  newuint8_t = newuint8_t | replaceMask; // apply mask
  return newuint8_t;
}

// there's already a 'writeBits' function which has been used instead of my own???
void I2CHelper::writeBits(uint8_t address, uint8_t registerToWrite,uint8_t startingReplacementBit, uint8_t noReplacementBits, uint8_t replacementValue){
  uint8_t originalRegisterValue = readRegister(address,registerToWrite);
  uint8_t newRegisterValue = modifyBits(originalRegisterValue,startingReplacementBit,noReplacementBits,replacementValue);
  writeRegister(address,registerToWrite,newRegisterValue);
}

void I2CHelper::writeBits2(uint8_t address, uint8_t registerToWrite, uint8_t originalRegisterValue, uint8_t startingReplacementBit, uint8_t noReplacementBits, uint8_t replacementValue){
  uint8_t newRegisterValue = modifyBits(originalRegisterValue,startingReplacementBit,noReplacementBits,replacementValue);
  writeRegister(address,registerToWrite,newRegisterValue);
}

// NEEDS TESTING
uint8_t I2CHelper::readBits(uint8_t address, uint8_t registerToRead,uint8_t startingBit, uint8_t noOfBits){
  uint8_t result = readRegister(address,registerToRead);
  uint8_t readMask = (1<< noOfBits)-1;  // set required number of 1s
  readMask = readMask << startingBit; // move to required position
  result &= readMask;
  result = result >> startingBit; // move bits of interest to beginning
  return result;
}

void I2CHelper::flushI2cBuffer(){
  while(available()){
    receive();
  }
}

// Create instance
I2CHelper I2CH;
