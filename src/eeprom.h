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
  float altitude;
  short snow_intensity;
  short rain_intensity;
  unsigned long timestamp;
} SingleResult;

int get_nth_result_address(uint16_t n);
bool nth_result_exists(uint16_t n);
SingleResult get_nth_result(uint16_t n=0);
SingleResult* get_all_results();
void save_current(SingleResult& result);
