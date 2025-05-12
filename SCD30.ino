void CheckSCD30(unsigned long cMillis){

  #ifdef Activate_SCD30

  if (cMillis - lastSCDtime >= 1000 && !disableGPS) {  // check every second
    lastSCDtime = cMillis;

    uint16_t dataReady = 0;
    if (scd30.getDataReady(dataReady) == 0 && dataReady) {
            if (scd30.readMeasurementData(co2, temp, hum) == 0) {
                //Serial.print("CO2: ");
                //Serial.println(co2);
                //Serial.print(" ppm, Temp: ");
                //Serial.print(temp);
                //Serial.print(" °C, Humidity: ");
                //Serial.print(hum);
                //Serial.println(" %");
            } else {
                Serial.println(F("Failed to read measurement data"));
            }
        }
  }
  #endif
}

void SetupSCD30(){

    #ifdef Activate_SCD30
    Wire.begin();
    scd30.begin(Wire, 0x61);  // default I2C address for SCD30

    int16_t err = scd30.startPeriodicMeasurement(0);
    if (err != 0) {
        Serial.print(F("Error starting measurement: "));
        Serial.println(err);
    }
    #endif
}
