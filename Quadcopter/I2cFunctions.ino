

void setupI2C(){
  I2c.begin();
  I2c.timeOut(50);
  I2c.setSpeed(1);  // fast (400Hz)
}



// ADD GENERAL FUNCTION TO READ MULTIPLE REGISTERS

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

void readMainSensors(byte address) {
  //
  lastReadingTime = thisReadingTime;
  thisReadingTime = millis();
  I2c.read(address, ACCEL_XOUT_H, 14);
  // read the most significant bit register into the variable then shift to the left
  // and binary add the least significant
  if(I2c.available()==14){
  AcX=I2c.receive()<<8|I2c.receive();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)  
  AcY=I2c.receive()<<8|I2c.receive();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=I2c.receive()<<8|I2c.receive();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=I2c.receive()<<8|I2c.receive();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=I2c.receive()<<8|I2c.receive();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=I2c.receive()<<8|I2c.receive();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=I2c.receive()<<8|I2c.receive();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  sensorRead = true;
  }
  else {
    sensorRead = false;
//    Serial.println(I2c.available());
    flushI2cBuffer();
  }
} 

byte getInteruptStatus(byte address){
  return readRegister(address,INT_STATUS);
}

void flushI2cBuffer(){
  while(I2c.available()){
    I2c.receive();
  }
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

