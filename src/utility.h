#include "application.h"
#include "eeprom.h"
#include "timelib/TimeLib.h"

uint32_t round_float_to_int(float num);
int get_num_of_results(SingleResult* results, unsigned long now, unsigned long max_offset=18000);
String get_time_string(time_t& tnow);
String two_digit(int i);
