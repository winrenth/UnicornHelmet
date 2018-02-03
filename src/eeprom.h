#ifndef EEPROM_H_
#define EEPROM_H_

#include "application.h"
#include "Particle.h"

#define SINGLERESULT_SIZE 32
#define TOTAL_RESULTS 48          // 1 full day, 2 per hour

typedef struct {
  float temperature;
  float humidity;
  uint16_t light;
  float CO2;
  float pressure;
  uint16_t altitude;
  int16_t snow_intensity;
  int16_t rain_intensity;
  uint8_t battery;
  unsigned long timestamp=0;
} SingleResult;

typedef struct {
  char device[24];
  unsigned long timestamp=0;
  float temperature;
  float pressure;
  float humidity;
  float CO2;
  int16_t light;
  uint8_t battery;
  int16_t snow_intensity;
  int16_t rain_intensity;
  int16_t result=0;
} UnicornProto;

int get_nth_result_address(uint16_t n);
bool nth_result_exists(uint16_t n);
SingleResult get_nth_result(uint16_t n=0);
SingleResult* get_all_results();
void save_current(SingleResult& result);


#endif
