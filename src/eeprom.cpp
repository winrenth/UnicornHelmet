#include "eeprom.h"


int get_nth_result_address(uint16_t n) {
    return n*SINGLERESULT_SIZE;
}

bool nth_result_exists(uint16_t n) {
    return true;
    return EEPROM.read(get_nth_result_address(n)) != 255;
}

SingleResult get_nth_result(uint16_t n) {
    uint16_t address = get_nth_result_address(n);
    SingleResult result;
    if (nth_result_exists(address)) {
        EEPROM.get(address, result);
    }
    return result;
}

SingleResult* get_all_results(){
    SingleResult *all_results = new SingleResult[TOTAL_RESULTS];
    for (int i = 0; i<TOTAL_RESULTS; i++) {
        all_results[i] = get_nth_result(i);
    }
    return all_results;
}

void save_current(SingleResult& result) {
    // pop + unishift current measurement store to include new result as the
    // first one
    SingleResult *results = get_all_results();
    EEPROM.put(0, result);
    for (int i = 1; i<TOTAL_RESULTS; i++) {
        EEPROM.put(i*SINGLERESULT_SIZE, results[i-1]);
    }
    delete[] results;
}
