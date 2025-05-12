#include <avr/pgmspace.h>
#include <Wire.h>


//------------------------------------------------------------------------------------------------------
//PhoenixTrack is adapted from https://github.com/daveake/FlexTrack

//This program uses multiple connected files: aprs, gps, misc and a sine_table.h 

//The most important settings are located here in the config section.


// CONFIGURATION SECTION.

// APRS + GPS = OK
// SD + GPS = OK
// SD + APRS = OK
// SD + APRS + GPS = OK

// Main settings
#define Activate_APRS 
#define Activate_GPS
//#define Activate_SD
#define Activate_BME680

// APRS transmission conditions
#define minimumSatellites 0                 //Amount of connected GPS satellites necessary to transmit APRS messages (to test can be set to 0)


// APRS settings
#define APRS_CALLSIGN    "ON6GMZ"               // Callsign (Max 6 characters)
#define APRS_SSID            11                 // Balloon launch
#define APRS_PATH_ALTITUDE   1500              // Below this altitude, (metres), path will switch to WIDE1-1, WIDE2-1.  Above it will be or path or WIDE2-1 (see below)
#define APRS_HIGH_USE_WIDE2    1                 // 1 means WIDE2-1 is used at altitude; 0 means no path is used

#define APRS_PRE_DEACTIVATION_TIME 100
#define APRS_REACTIVATION_TIME 1000

#define APRS_TX_INTERVAL      60                 // APRS Transmission Interval in seconds
#define APRS_RANDOM          1                // Adjusts time to next transmission by up to +/1 this figure, in seconds.
                                               // So for interval of 60 (seconds), and random of 30 (seconds), each gap could be 30 - 90 seconds.
                                               // Set to 0 to disable!
                                               
#define APRS_TELEM_INTERVAL  1                // How often to send telemetry packets.  Comment out to disable
#define APRS_PRE_EMPHASIS                      // Comment out to disable 3dB pre-emphasis.

#define APRS_DEVID "PHOENIX"                     // APRS packet device id


//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------

// HARDWARE DEFINITIONS

//Habduino board pins 
#define LED_WARN           7
#define LED_OK             4
#define GPS_RX             5
#define GPS_TX             6
#define APRS_ENABLE        2 //HX1 enable
#define APRS_DATA          3 //TXD HX1
#define SD_CS              10

// Define GPS data structure
struct GPSData {
  float Time;
  float Latitude;
  float Longitude;
  float Altitude;
  int Satellites;
}; 

// GPS struct object
GPSData GPS; 

//States
bool enabled = false;
bool disableGPS = false;

#ifdef Activate_GPS
  #include <SoftwareSerial.h>
  SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
#endif

#ifdef Activate_SD
  #include <SD.h>
  File logFile;
#endif

#ifdef Activate_BME680
  #include "Adafruit_BME680.h"

  Adafruit_BME680 bme; // I2C
#endif

//------------------------------------------------------------------------------------------------------


void setup() { 

  // Set up leds
  InitLEDS(LED_OK, LED_WARN);

  //Setup and annoucements
  Serial.begin(9600);
  Serial.println("");
  Serial.println(F("Phoenix Flight Computer, payload ID(s):"));
     
  #ifdef Activate_APRS
    Serial.print(F("APRS: "));
    Serial.print(APRS_CALLSIGN);
    Serial.print(F("-"));
    Serial.print(APRS_SSID);
    Serial.println("");
  #endif  
  
  Serial.println(F("Active systems:"));

  #ifdef Activate_GPS
    Serial.println(F("GPS enabled"));
    SetupGPS();
  #endif

    
  #ifdef Activate_APRS 
    Serial.println(F("APRS telemetry enabled"));
    SetupAPRS();
  #endif

  #ifdef Activate_SD
    while (!SD.begin(SD_CS)) {
      Serial.println(F("SD initialization failed! Retrying..."));
      delay(1000);
    }
    
    Serial.println(F("SD initialized."));
  #endif

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


  Serial.print(F("Free memory = "));
  Serial.println(freeRam());
  delay(3000);
}


void loop() {  
  //Each check function updates the data or starts the correct radio functions
  #ifdef Activate_GPS
    CheckGPS(); 
  #endif
  
  #ifdef Activate_APRS
    CheckAPRS(millis());
  #endif

  #ifdef Activate_SD
    CheckSD(millis());
  #endif

  UpdateLEDS(enabled);
}



//With this function you can find out how much room you have left on the Arduino.
int freeRam(void)
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
