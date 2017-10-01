// occasionally the reading seems to get 'stuck' on 1023, so far I'm not sure why


// use a 1k and 3.3k resistor divider. Take after 3.3 and before 1.
// (is there any min current needs for ADC?)
// 16.8 (max 4S battery voltage) should give 3.907V from the divider
// 1024 / 5 * 3.907 = 800 at max voltage
// 1024 / 5 * 3.3 = 676 at 3.3V
// 1024 / 5 * 3.1 = 635 REALLY NEED TO STOP NOW (automatic land?)


// current through 3.3k resistor will be 3.907 / 3300 = 1mA

// is there any change of initial high voltage?
// add capacitor?

const byte PIN_BATTERY_MONITOR = A0;
const byte PIN_BATTERY_INDICATOR = 8;
//const float SCALE = 5.0 / 1024;
//const float BATTERY_FILTER_ALPHA = 0.2;
//const float BATTERY_MAX_VOLTAGE = 4 * 4.2;
//const float DIVIDER_TO_BATTERY = BATTERY_MAX_VOLTAGE / 5;
//const float BATTERY_MIN_VOLTAGE = 4 * 3.1;
int dividerReading = 0;
//float dividerVoltage, batteryVoltage;
int batteryLevel = 0;
const int dividerMinReading = 660;  // minimum that the battery should ever get to
const int dividerRange = (1024 - dividerMinReading) / 8;


void calculateBatterySimple() {
  dividerReading = 0.5 * dividerReading + 0.5 * analogRead(PIN_BATTERY_MONITOR);
  batteryLevel = (dividerReading -  dividerMinReading / dividerRange) - 1;  // to give a number between 0 and 7
  if(batteryLevel<0) batteryLevel = 0;  // just in case
}

//void calculateBatterySimple(){
//  dividerReading = 0.5 *dividerReading + 0.5 * analogRead(PIN_BATTERY_MONITOR);
//  if (dividerReading < 660){
//    batteryLevel = 0;
//  }
//  else {
//    batteryLevel = 7;
//  }
//}





void setupBatteryMonitor() {
  pinMode(PIN_BATTERY_MONITOR, INPUT);
  pinMode(PIN_BATTERY_INDICATOR, OUTPUT);
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  ADCSRA |= bit (ADPS2);  // set ADC prescaler to 16 (from default 128)
  for (byte i = 0; i < 30; i++) {
    calculateBatterySimple();
  }
}




//// calculate the actual voltage
//void calculateBatteryVoltage(){
//  dividerReading = analogRead(PIN_BATTERY_MONITOR);
//  dividerVoltage = dividerVoltage *(1-BATTERY_FILTER_ALPHA) + (dividerReading * SCALE) * BATTERY_FILTER_ALPHA;  // filter a little
//  batteryVoltage = dividerVoltage * DIVIDER_TO_BATTERY;
//
////  Serial.println(batteryVoltage);
//}

// convert voltage to a scale of 0 to 7 (i.e. can be stored in 3 bits)
// and only map the range I care about
// if it's below that range, set to zero (else might look like a large positive if stored in byte)
//byte calculateBatteryLevel(){
//  batteryLevel = map(batteryVoltage*10, BATTERY_MIN_VOLTAGE*10,BATTERY_MAX_VOLTAGE*10,0,7);
//  if(batteryLevel < 0){
//    batteryLevel = 0;
//  }
//}

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




