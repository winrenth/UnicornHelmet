#include "utility.h"

uint32_t round_float_to_int(float num) {
    return (num >= 0) ? (uint32_t)(num + 0.5) : (uint32_t)(num - 0.5);
}

int get_num_of_results(SingleResult* results, unsigned long now, unsigned long max_offset){
    for (int i=0; i<TOTAL_RESULTS; i++) {
        if ((now - results[i].timestamp) > max_offset){
            return i;
        }
    }
    return TOTAL_RESULTS;
}

String get_time_string(time_t& tnow){
    String time_str = String(year(tnow) % 100);
    time_str += "-";
    time_str += two_digit(month(tnow));
    time_str += "-";
    time_str += two_digit(day(tnow));
    time_str += " ";
    time_str += two_digit(hour(tnow));
    time_str += ":";
    time_str += two_digit(minute(tnow));
    return time_str;
}

String two_digit(int i){

    if (i < 10){
        return "0" + String(i);
    }
    return String(i);
}
