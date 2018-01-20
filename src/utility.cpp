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

String get_time_string(SparkTime& rtc, unsigned long tnow){
    String time_str = rtc.yearShortString(tnow);
    time_str += "-";
    time_str += rtc.monthString(tnow);
    time_str += "-";
    time_str += rtc.dayString(tnow);
    time_str += " ";
    time_str += rtc.hourString(tnow);
    time_str += ":";
    time_str += rtc.minuteString(tnow);
    return time_str;
}
