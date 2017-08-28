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
