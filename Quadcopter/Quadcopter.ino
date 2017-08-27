// RECEIVER (ignore for now - use direct control?)
// Start work on motor control

// inner loop for rate mode - constant rate of change of angle
// outer loop for balance mode

// remove debug statements later

// should I control ESC pulses myself or use servo.write - USE SERVO.WRITE for now

#include <I2C.h>
#include <PID_v1.h>
#include <Servo.h>
#include <SPI.h>
#include <RF24.h>


void setup() {
  Serial.begin(115200);

  setupI2C();
  setupMotionSensor();
  setupRadio();

  // check MPU connected
  byte MPU_ADDRESS_CHECK = readRegister(MPU_ADDRESS,WHO_AM_I);
  if(MPU_ADDRESS_CHECK==MPU_ADDRESS){
    Serial.println(F("MPU-6050 available"));
  }
  else {
    Serial.println(F("ERROR: MPU-6050 NOT FOUND"));
    Serial.println(F("Try reseting..."));
    while(1);
  }



  Serial.println(F("Setup complete"));
  // Indicate readiness somehow. LED?
  
}

void loop() {
  // at what frequency should everything run?
  // gyro 8kHz, accel 1kZ - but depends on LPF settings too I think
  // receiver 10Hz?
  // read sensor 250Hz?
  // inner loop 250Hz?
  // outer 100Hz?

  // read receiver
  // read gyros/accel
  // calculate inner PID (gyro info only)
  // update motors
  // update current angle estimation (gyro + accel info)
  // update inner loop setpoint
  // update outer loop setpoint (if balance mode)
  
  



  

}
