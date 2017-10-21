// use a 1k and 3.3k resistor divider. Take after 3.3 and before 1.
// (is there any min current needs for ADC?)
// 16.8 (max 4S battery voltage) should give 3.907V from the divider
// 1024 / 5 * 3.907 = 800 at max voltage
// 1024 / 5 * 3.3 = 676 at 3.3V
// 1024 / 5 * 3.1 = 635 REALLY NEED TO STOP NOW (automatic land?)

// current through 3.3k resistor will be 3.907 / 3300 = 1mA

const byte PIN_BATTERY_MONITOR = A0;
const byte PIN_BATTERY_INDICATOR = 8;
const float batteryFilterAlpha = 0.5;
int dividerReading = 0;
byte batteryLevel = 0;
const int dividerMaxReading = 250; // corresponds to full charge, 16.8V
const int dividerMinReading = 100;  // minimum that the battery should ever get to
const int dividerRange = dividerMaxReading - dividerMinReading;


void calculateBatteryLevel() {
  int tmp = analogRead(PIN_BATTERY_MONITOR);
  dividerReading = (1-batteryFilterAlpha) * dividerReading + batteryFilterAlpha * tmp;
  float batteryLevelFloat = ((float)(dividerReading - dividerMinReading) / (dividerRange)) * 8;
  batteryLevel = (byte)batteryLevelFloat;
  batteryLevel = constrain(batteryLevel,0,7);
//  Serial.println(batteryLevel);
}


void setupBatteryMonitor() {
  pinMode(PIN_BATTERY_MONITOR, INPUT);
  pinMode(PIN_BATTERY_INDICATOR, OUTPUT);
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  ADCSRA |= bit (ADPS2);  // set ADC prescaler to 16 (from default 128)
  for (byte i = 0; i < 30; i++) {
    calculateBatteryLevel();
  }
}


// update the data being sent back to the transmitter, and if battery low then turn on the low battery LED
// make it blink instead?
//void updateBatteryIndicator(){
////  updateAckStatusForTx(5, batteryLevel);  // battery level will be shown in bits 5/6/7 in status
//  if(batteryLevel<=0){
////    digitalWrite(PIN_BATTERY_INDICATOR,HIGH);
//  }
//}


//void setupBatteryMonitor(){
//  pinMode(PIN_BATTERY_MONITOR,INPUT);
//  pinMode(PIN_BATTERY_INDICATOR,OUTPUT);
//  for(byte i = 0; i < 20; i++){
//    calculateBatteryVoltage();
//    calculateBatteryLevel();
//  }
//  updateBatteryIndicator();
//}




