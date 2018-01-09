#include <Arduino.h>
#include "hc_sr04.h"

Ultrasonic::Ultrasonic(uint8_t trigPin, uint8_t max_dist) {
  maxdist = max_dist;
  trig = trigPin;
  timeout = 0;
}

unsigned int Ultrasonic::get_sound_speed(float temp) {
  return SOUNDSPEED + 0.6 * temp;
}

unsigned int Ultrasonic::get_speed_to_cm_factor(float temp) {
  float sound_speed = get_sound_speed(temp);
  return 10000 / sound_speed;
}

unsigned int Ultrasonic::get_timeout(float temp) {
  // sound travel time both ways roundtrip in given temp 
  return (get_speed_to_cm_factor(temp) * 2) * maxdist;
}

unsigned int Ultrasonic::timing(float temp) {

  pinMode(trig, OUTPUT);

  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  pinMode(trig, INPUT);

  timeout = get_timeout(temp);

  if (digitalRead(trig)) return NO_ECHO;          // Previous ping hasn't finished, abort.
  _max_time = micros() + timeout + MAX_SENSOR_DELAY;  // Maximum time we'll wait for ping to start (most sensors are <450uS, the SRF06 can take up to 34,300uS!)
  while (!digitalRead(trig))                          // Wait for ping to start.
    if (micros() > _max_time) return NO_ECHO;         // Took too long to start, abort.
  //Serial.println("started");
  _max_time = micros() + timeout;               // Ping started, set the time-out.
  while (digitalRead(trig))                     // Wait for the ping echo.
    if (micros() > _max_time) return NO_ECHO;   // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
  //Serial.println(micros() - (_max_time - timeout - PING_OVERHEAD));
  return micros() - (_max_time - timeout - PING_OVERHEAD); // Calculate ping time, include overhead.
}

unsigned int Ultrasonic::distance_read(float temp) {
  float cm_factor = get_speed_to_cm_factor(temp);
  float round_trip = timing(temp);
  return round_trip / cm_factor / 2;  
}


unsigned int Ultrasonic::get_n_readings_pct(float temp, int n) {
  // make n readings and calcualte hit/miss procentage
  unsigned int successful= 0;
  //String  results;
  for (int i=0; i<n; i++) {
    unsigned int ms = timing(temp);
    //results = results + String(ms) + "\n\r\t";
    if (timing(temp) > 0) {
      successful++;
    }
    delay(PORBEDELAY);
  }
  //return results;
  return  (successful > 0) ? (successful / n * 100) : successful;  
}
