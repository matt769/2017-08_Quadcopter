// note that including this library will include an instance called I2CH

#ifndef I2CHelper_H
#define I2CHelper_H

#include <I2C.h>

class I2CHelper : public I2C {
  public:
    I2CHelper();
    uint8_t readRegister(uint8_t address, uint8_t sensorRegister);
    void writeRegister(uint8_t address, uint8_t sensorRegister, uint8_t data);
    void writeBits(uint8_t address, uint8_t registerToWrite, uint8_t startingReplacementBit, uint8_t noReplacementBits, uint8_t replacementValue);
    void writeBits2(uint8_t address, uint8_t registerToWrite, uint8_t originalRegisterValue, uint8_t startingReplacementBit, uint8_t noReplacementBits, uint8_t replacementValue);
    uint8_t readBits(uint8_t address, uint8_t registerToRead, uint8_t startingBit, uint8_t noOfBits);
    void flushI2cBuffer(void);
  private:
    bool i2cTimeout;
    uint8_t modifyBits(uint8_t originaluint8_t, uint8_t startingReplacementBit, uint8_t noReplacementBits, uint8_t replacementValue);
};


#endif
