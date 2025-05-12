/* From Project Swift - High altitude balloon flight software                 */
/*=======================================================================*/
/* Copyright 2010-2012 Philip Heron <phil@sanslogic.co.uk>               */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */

//If you want to change the message format go to char data (ctrl+f)

#ifdef APRS_DATA

#define FRAME_LENGTH 100 //Do not change

#include <util/crc16.h>
#include <avr/pgmspace.h>

#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (50)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)



// APRS ariables

//Timing
unsigned long NextAPRS = 0;
unsigned long GPS_Reenable_Time = 0;

//Buffers
//char data[FRAME_LENGTH];

volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

//Message
uint16_t messagesTransmitted = 0;

static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
};

// Code

void SetupAPRS(void)
{

#ifdef Activate_APRS
  pinMode(APRS_ENABLE, OUTPUT);
  digitalWrite(APRS_ENABLE, 0);
#endif

  // Fast PWM mode, non-inverting output on OC2A
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  
  pinMode(APRS_DATA, OUTPUT);
}

void CheckAPRS(unsigned long cMillis) // Function to decide when to transmit a message.
{   
    //First time enabling the switching of GPS
    if (!enabled && GPS.Satellites >= minimumSatellites) //&& GPS.Latitude != 0.0 && GPS.Longitude != 0.0) 
      enabled = true; 
    
    //Disable GPS and SD a set time before next aprs message
    if (cMillis >= NextAPRS - APRS_PRE_DEACTIVATION_TIME && !disableGPS && enabled) {
        disableGPS = true;
        //Serial.println(F("Disabled GPS and SD"));
        // Disable GPS Serial 
        #ifdef Activate_GPS
          gpsSerial.end(); 
        #endif

        //Disable SD card
        #ifdef Activate_SD
          SD.end();
        #endif
    }


    // Check if it's time to send an APRS packet
    if (cMillis >= NextAPRS && enabled && _txlen == 0) 
    {
        Serial.println(F("Sending APRS Packet."));
        tx_aprs(); // Send the APRS packet

        // Update the next transmission time
        NextAPRS = cMillis + (APRS_TX_INTERVAL * 1000UL); // 🔧OPTIMIZATION: Forces unsigned long math, avoids float promotion

        GPS_Reenable_Time = cMillis + APRS_REACTIVATION_TIME; 
        
    } else if (cMillis >= NextAPRS && !enabled) {
      Serial.print(F("Too few sats, current amount: "));
      Serial.println(GPS.Satellites);
      NextAPRS = cMillis + (APRS_TX_INTERVAL * 1000UL); 
    }
    
    //Re-enable GPS and SD Serial 
    if ( cMillis >= GPS_Reenable_Time && disableGPS && enabled) {
        //Serial.println(F("Enabled GPS and SD"));
        #ifdef Activate_GPS
          gpsSerial.begin(9600);
        #endif
        
        #ifdef Activate_SD
          SD.begin(SD_CS);
        #endif 
        disableGPS = false;

        GPS_Reenable_Time = 4200000000;
    }


}


//Very important function, this makes the total message for APRS. DO NOT CHANGE!
void ax25_frame(const char *scallsign, const char sssid, const char *dcallsign, const char dssid, const char ttl1, const char ttl2, const char *data, ...)
{
  static uint8_t frame[FRAME_LENGTH];
  uint8_t *s;
  uint16_t x;
  va_list va;

  va_start(va, data);
  
  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if (ttl1) s = _ax25_callsign(s, "WIDE1", ttl1);
  if (ttl2) s = _ax25_callsign(s, "WIDE2", ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  vsnprintf((char *) s, FRAME_LENGTH - (s - frame) - 2, data, va);
  va_end(va);

  /* Calculate and append the checksum */
  for(x = 0xFFFF, s = frame; *s; s++)
    x = _crc_ccitt_update(x, *s);

  *(s++) = ~(x & 0xFF);
  *(s++) = ~((x >> 8) & 0xFF);

  /* Point the interrupt at the data to be transmit */
  _txbuf = frame;
  _txlen = s - frame;

  /* Enable the timer and key the radio */
  TIMSK2 |= _BV(TOIE2);
  
#ifdef LED_TX
  digitalWrite(LED_TX, 1);
#endif

#ifdef Activate_APRS
  digitalWrite(APRS_ENABLE, 1);
#endif
}


//Message container function, here you can change the content of the message
void tx_aprs(void)
{
  int32_t aprs_alt_ft; 
  char slat[5];
  char slng[5];
  //char Wide1Path, Wide2Path;

  //Debug GPS data
  //Serial.print(F("Lat: ")); Serial.print(GPS.Latitude, 6);
  //Serial.print(F(", Lon: ")); Serial.print(GPS.Longitude, 6);
  //Serial.print(F(", Sats: ")); Serial.print(GPS.Satellites);
  //Serial.print(F(", Alt(m): ")); Serial.print(GPS.Altitude, 2);

  ax25_base91enc(slat, 4, 380926 * (90.0 - GPS.Latitude));
  ax25_base91enc(slng, 4, 190463 * (180.0 + GPS.Longitude));

  //Serial.print(F(", APRS: "));
  //Serial.print(slat);
  //Serial.println(slng);

  aprs_alt_ft = GPS.Altitude * 32808 / 10000;


  /*float temp = 27; //float(bme.temperature);
  float humidity = 69; //float(bme.humidity);
  float pressure = 1013; //float(bme.pressure / 100.0);
  float gas_R = 15; //float(bme.gas_resistance / 1000.0);

  //BME
  char buffer_temp[9];
  char buffer_pressure[9];
  char buffer_humidity[9];
  char buffer_gas_R[9];

  dtostrf(temp,4,2,buffer_temp);
  dtostrf(pressure,6,2,buffer_pressure);
  dtostrf(humidity,4,2,buffer_humidity);
  dtostrf(gas_R,5,2,buffer_gas_R);
  */  
    
  /*
  if (GPS.Altitude > APRS_PATH_ALTITUDE) //Change APRS above a certain altitude (standard = 1500m)
  {
    Wide1Path = 1;
    Wide2Path = 0; // used to be APRS_HIGH_USE_WIDE2;
  }
  else
  {
    Wide1Path = 1;
    Wide2Path = 0; // used to be 1
  }
    sprintf(data,"Sat:%d, Count:%d",
      GPS.Satellites,
      messagesTransmitted
    );
  */  

 char bmeT[8] = "-80.00"; 
 char bmeP[8] = "1013.25";
 char bmeH[8] = "99.99";
 char bmeR[8] = "14000";
 char scdCO2[9] = "40000.00";
 char scdT[8] = "-70.00";
 #ifdef Activate_BME680
 dtostrf(bme.temperature, 0, 2, bmeT);  // minimal width 0, 2 BC
 dtostrf(bme.pressure / 100.0, 0, 2, bmeP);  // minimal width 0, 2 BC
 dtostrf(bme.humidity, 0, 2, bmeH);  // minimal width 0, 2 BC
 dtostrf(bme.gas_resistance / 1000.0, 0, 0, bmeR);  // minimal width 0, 0 BC
 #endif

 #ifdef Activate_SCD30
 dtostrf(co2, 0, 2, scdCO2);  // minimal width 0, 2 BC
 dtostrf(temp, 0, 2, scdT);  // minimal width 0, 2 BC
 #endif

    ax25_frame(
    APRS_CALLSIGN, APRS_SSID, // ON6GMZ - 11
    APRS_DEVID, 0, 
    0, 1, // WIDE2-1
    "!/%s%sO   /A=%06ld|%d,%s,%s,%s,%s,%s,%s,%d abcdefghijklmnop",// 7 characters room
    slat, slng,
    aprs_alt_ft,
    GPS.Satellites,
    bmeT,
    bmeP,
    bmeH,
    bmeR,
    scdCO2,
    scdT,
    messagesTransmitted
);
    
    /*
    ax25_frame(
      APRS_CALLSIGN, APRS_SSID,
      APRS_DEVID, 0,
      1, 0, // Used to be Wide1Path = 1, Wide2Path = 0
      "!/%s%sO   /A=%06ld|%s",
      slat, slng,  
      GPS.Altitude * 32808 / 10000, // Calculate height in feet
      data        
    );
    */
    messagesTransmitted++; //amount of messages transmitted
}



//More APRS functions
ISR(TIMER2_OVF_vect)
{

  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;
  uint8_t value;
  
  /* Update the PWM output */
  value = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  #ifdef APRS_PRE_EMPHASIS
  if (step == PHASE_DELTA_1200)
  {
    value = (value >> 1) + 64;
  }
  #endif
  OCR2B = value;
  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {

        // Disable radio, Tx LED off, disable interrupt

#ifdef Activate_APRS
        digitalWrite(APRS_ENABLE, 0);
#endif

#ifdef LED_TX
  digitalWrite(LED_TX, 0);
#endif

        TIMSK2 &= ~_BV(TOIE2);

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;
        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}
char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

static uint8_t *_ax25_callsign(uint8_t *s, const char *callsign, const char ssid)
{
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

#endif
