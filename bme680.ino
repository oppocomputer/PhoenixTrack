void CheckBME680(unsigned long cMillis){
  
#ifdef Activate_BME680
if (cMillis - lastBMEtime >= 1000 && !disableGPS) {
    lastBMEtime = cMillis;

    bme.endReading();
    //Serial.print(F("Temperature = "));
    //Serial.print(bme.temperature);
    //Serial.println(F(" *C"));

    bme.beginReading();
  }
#endif
}

void SetupBME680(){

  #ifdef Activate_BME680
  while (!bme.begin()) {
      Serial.println(F("Could not find a valid BME680 sensor! Retrying..."));
      delay(1000);
    }

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  #endif
}
