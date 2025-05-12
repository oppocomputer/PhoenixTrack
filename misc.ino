#define INTERVAL 500

unsigned long previousTime = 0;
bool ledState = LOW;

void InitLEDS(int OK_Led_pin, int Warn_Led_pin) {
  pinMode(OK_Led_pin, OUTPUT);
  pinMode(Warn_Led_pin, OUTPUT);
}

void UpdateLEDS(bool aprs_state) {
#ifdef Activate_GPS
  if (GPS.Altitude > APRS_PATH_ALTITUDE) {
    digitalWrite(LED_WARN, LOW);
    digitalWrite(LED_OK, LOW);
    return;
  }
#endif

  if (!aprs_state) {
    if (millis() - previousTime >= INTERVAL) {
      previousTime = millis();
      ledState = !ledState;
      digitalWrite(LED_WARN, ledState);
    }
    digitalWrite(LED_OK, LOW);
    return;
  }

  digitalWrite(LED_OK, HIGH);
  digitalWrite(LED_WARN, _txlen ? HIGH : LOW);
}
