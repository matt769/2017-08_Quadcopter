// use a 1k and 3.3k resistor divider. Take after 3.3 and before 1.
// (is there any min current needs for ADC?)
// 16.8 (max 4S battery volage) should give 3.907V from the divider
// 1024 / 5 * 3.907 = 800 at max voltage
// 1024 / 5 * 3.3 = 676 at 3.3V
// 1024 / 5 * 3.1 = 635 REALLY NEED TO STOP NOW (automatic land?)


// current through 3.3k resistor will be 3.907 / 3300 = 1mA

// is there any change of initial high voltage?
// add capacitor?

byte pinBatteryMonitor = A5;

const float increment = 5.0 / 1024;

int batteryReading;
float voltage;
int batteryLevel;


void calculateVoltage(){
  batteryReading = analogRead(pinBatteryMonitor);
  voltage = batteryReading * increment;
}

byte calculateBatteryLevel(){
  batteryLevel = map((int)(voltage * 10.0), 31,39,0,7);
}

