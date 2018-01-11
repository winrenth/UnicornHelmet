#include "eeprom.h"

int get_nth_result_address(uint16_t n) {
    return n*SINGLERESULT_SIZE;
}

bool nth_result_exists(uint16_t n) {
    return EEPROM.read(get_nth_result_address(n)) != 255;
}

SingleResult get_nth_result(uint16_t n) {
    uint16_t address = get_nth_result_address(n);
    SingleResult result;
    EEPROM.get(address, result);
    return result;
}

SingleResult* get_all_results(){
    SingleResult results [TOTAL_RESULTS];
    for (int i = 0; i<TOTAL_RESULTS; i++) {
        results[i] = get_nth_result(i);
    }
    return results;
}

void save_current(SingleResult& result) {
    // pop + unishift current measurement store to include new result as the
    // first one
    SingleResult *results = get_all_results();
    results[0] = result;
    for (int i = 0; i<TOTAL_RESULTS; i++) {
        EEPROM.put(i*SINGLERESULT_SIZE, results[i]);
    }
}
