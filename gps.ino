/* ========================================================================== */
/*   gps.ino                                                                  */
/*                                                                            */
/*   SoftwareSerial code for NEO 6M module                                    */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */




//Setup 
void SetupGPS(void)
{ 
    #ifdef Activate_GPS
      gpsSerial.begin(9600); 
    #endif
}


void CheckGPS(void)
{    
  #ifdef Activate_GPS
    if (gpsSerial.available() && !disableGPS) {
      String data = gpsSerial.readStringUntil('\n'); // Read incoming data
      //Serial.println(data); //MAIN DEBUGTOOL
      if (data.startsWith("$GPGGA")) {
        parseGPGGA(data, GPS);
      }
    }
  #endif
}



// Function to convert DMS to decimal degrees
float convertToDecimalDegrees(String dms, char direction) {
    int degrees = dms.substring(0, 2).toInt(); // First two characters are degrees
    float minutes = dms.substring(2).toFloat(); // Remaining part is minutes

    float decimalDegrees = degrees + (minutes / 60.0);
    
    // Adjust based on direction
    if (direction == 'S' || direction == 'W') {
        decimalDegrees = -decimalDegrees; // Make negative for S or W
    }
    return decimalDegrees;
}

void parseGPGGA(String data, GPSData &gps) {
    int commaIndex[15]; // GPGGA sentences have up to 14 commas
    int commaCount = 0;
    int index = -1;

    // Find the indices of all commas
    while ((index = data.indexOf(',', index + 1)) != -1) {
        commaIndex[commaCount++] = index;
    }

    //Debug comma indices
    /*for (int i = 0; i <= 15; i++) {
      Serial.print(commaIndex[i]);
      Serial.print(",");
    }
    Serial.println();
    */
    
    // Detect if there are less than a certain number of comma's
    //if (commaCount < 8) {Serial.println("Returned"); return; } 

    // Extract time
    String timeStr = data.substring(commaIndex[0] + 1, commaIndex[1]);
    gps.Time = timeStr.toFloat();
    
    // Extract latitude
    String latStr = data.substring(commaIndex[1] + 1, commaIndex[2]);
    char latDir = data.charAt(commaIndex[2] + 1);


    // Extract longitude
    String lonStr = data.substring(commaIndex[3] + 2, commaIndex[4]);
    char lonDir = data.charAt(commaIndex[4] + 1);

    // Extract number of satellites (7th field)
    gps.Satellites = data.substring(commaIndex[6] + 1, commaIndex[7]).toInt();

    if (gps.Satellites > 0) {
      // Convert latitude and longitude to decimal degrees
      gps.Latitude = convertToDecimalDegrees(latStr, latDir);
      gps.Longitude = convertToDecimalDegrees(lonStr, lonDir);

      // Extract altitude (10th field)
      gps.Altitude = data.substring(commaIndex[8] + 1, commaIndex[9]).toFloat();
    }
}