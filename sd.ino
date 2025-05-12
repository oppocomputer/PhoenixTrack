#ifdef Activate_SD

//Timing
unsigned long lastSDtime {0};
const long sdInterval = 1000;
unsigned long cMillis {0};
unsigned long counter {0};

const char* sep = ",";


void CheckSD(unsigned long cMillis)
{ 
  if (cMillis - lastSDtime >= sdInterval && !disableGPS) {
    lastSDtime = cMillis;
    #ifdef Activate_BME680
      bme.endReading();
    #endif

    WriteSD();

    #ifdef Activate_BME680
      bme.beginReading();
    #endif

    counter++;
  }
}


void WriteSD(void)
{  
  logFile = SD.open(F("log.txt"), FILE_WRITE);

  if (!logFile) {
    //Serial.println(F("Error opening file."));

    if (!disableGPS) {
      !SD.begin(SD_CS);
    }

  } else {
    
    //Header
    if (counter == 0) {
      logFile.println(F("Counter,Latitude,Longitude,Altitude,Satellites,Time,"));
      //Serial.println(F("Header written."));
    }

    //Data
    logFile.print(counter);
    logFile.print(sep);
    logFile.print(GPS.Latitude, 8);
    logFile.print(sep);
    logFile.print(GPS.Longitude, 8);
    logFile.print(sep);
    logFile.print(GPS.Altitude, 2);
    logFile.print(sep);
    logFile.print(GPS.Satellites);
    logFile.print(sep); 
    logFile.print(GPS.Time, 0);
    logFile.print(sep); 
    logFile.println("");


    /*
    Serial.print("Lat: ");
    Serial.print(GPS.Latitude,8);
    Serial.print(", Long: ");
    Serial.print(GPS.Longitude,8);
    Serial.print(", Alt: ");
    Serial.print(GPS.Altitude,2);
    Serial.print(", Sats: ");
    Serial.print(GPS.Satellites);
    Serial.print(", Time: ");
    Serial.print(GPS.Time,0);
    Serial.print(", Count: ");
    Serial.println(counter);
    */
  }

  //Close file and save data
  logFile.close();

}
#endif