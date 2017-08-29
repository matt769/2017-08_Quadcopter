///////////////////////////////////////////////////////////////////////////////////
// OUTPUT

void sendSerialDataFloat(float one, float two, float three){
    Serial.print(one); Serial.print('\t');
    Serial.print(two); Serial.print('\t');
    Serial.print(three); Serial.print('\t');
    Serial.print('\n');
}

void sendSerialDataInt(int one, int two, int three){
    Serial.print(one); Serial.print('\t');
    Serial.print(two); Serial.print('\t');
    Serial.print(three); Serial.print('\t');
    Serial.print('\n');
}


//
//    Serial.print("Current:  "); Serial.print('\t');
//    Serial.print(currentAngles.roll); Serial.print('\t');
//    Serial.print(currentAngles.pitch); Serial.print('\t');
//    Serial.print(currentAngles.yaw); Serial.print('\n');
//
//    Serial.print("Accel:    "); Serial.print('\t');
//    Serial.print(accelAngles.roll); Serial.print('\t');
//    Serial.print(accelAngles.pitch); Serial.print('\t');
//    Serial.print(accelAngles.yaw); Serial.print('\n');
//
//    Serial.print("GyroChange:"); Serial.print('\t');
//    Serial.print(gyroChangeAngles.roll); Serial.print('\t');
//    Serial.print(gyroChangeAngles.pitch); Serial.print('\t');
//    Serial.print(gyroChangeAngles.yaw); Serial.print('\n');

//    Serial.print("NewCurrent:"); Serial.print('\t');
//    Serial.print(currentAngles.roll); Serial.print('\t');Serial.print('\t');
//    Serial.print(currentAngles.pitch); Serial.print('\t');Serial.print('\t');
//    Serial.print(currentAngles.yaw); Serial.print('\n');



