// use a 1k and 3.3k resistor divider. Take after 3.3 and before 1.
// (is there any min current needs for ADC?)
// 16.8 (max 4S battery voltage) should give 3.907V from the divider
// 1024 / 5 * 3.907 = 800 at max voltage
// 1024 / 5 * 3.3 = 676 at 3.3V
// 1024 / 5 * 3.1 = 635 REALLY NEED TO STOP NOW (automatic land?)


// current through 3.3k resistor will be 3.907 / 3300 = 1mA

// is there any change of initial high voltage?
// add capacitor?

byte pinBatteryMonitor = A5;
byte pinBatteryIndicator = 7;
const float scale = 5.0 / 1024;
int batteryReading;
float voltage;
byte batteryLevel = 0;

// calculate the actual voltage
void calculateBatteryVoltage(){
  batteryReading = analogRead(pinBatteryMonitor);
  voltage = voltage*0.5 + (batteryReading * scale) *0.5;  // filter a little
}

// convert voltage to a scale of 0 -7 (i.e. can be stored in 3 bits)
byte calculateBatteryLevel(){
  batteryLevel = map((int)(voltage * 10.0), 31,39,0,7);
}

// update the data being sent back to the transmitter, and if battery low then turn on the low battery LED 
// make it blink instead?
void updateBatteryIndicator(){
  updateAckStatusForTx(5, batteryLevel);  // battery level will be shown in bits 5/6/7 in status
  if(batteryLevel<2){
    digitalWrite(pinBatteryIndicator,HIGH);
  }
}


void setupBatteryMonitor(){
  pinMode(pinBatteryIndicator,OUTPUT);
  for(byte i = 0; i < 20; i++){
    calculateBatteryVoltage();
    calculateBatteryLevel();
  }
}




