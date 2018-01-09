#ifndef hc_sr04_h
#define hc_sr04_h

#define SOUNDSPEED 321
#define PORBEDELAY 100
#define NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance. Default=0
#define MAX_SENSOR_DELAY 5800   // Maximum uS it takes for sensor to start the ping. Default=5800
#define PING_OVERHEAD 5

class Ultrasonic {
  public:
    Ultrasonic(uint8_t trigpin, uint8_t maxdist = 10);
    unsigned int distance_read(float temp);
    unsigned int get_n_readings_pct(float temp, int n = 10);

  private:
    uint8_t trig;
    uint8_t maxdist;
    unsigned long timeout;
    unsigned int timing(float temp);
    unsigned int get_sound_speed(float temp);
    unsigned int get_speed_to_cm_factor(float temp);
    unsigned int get_timeout(float temp);
    unsigned long _max_time;
};

#endif // hc_sr04_h
