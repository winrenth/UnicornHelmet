#include "eeprom.h"

int get_nth_result_address(n) {
    return n*SINGLERESULT_SIZE;
}

bool nth_result_exists(n) {
    return EEPROM.read(get_nth_result_address(n)) != 255;
}

SingleResult get_nth_result(n=0) {
    address = get_nth_result_address(n);
    SingleResult result = EEPROM.read(address);
    reuturn result;
}

SingleResult* get_all_results(){
    SingleResult results [TOTAL_RESULTS];
    for (int i = 0; i<TOTAL_RESULTS; i++ {
        results[i] = get_nth_result(i);
    }
    return results;
}

void save_current(SingleResult& result) {
    SingleResult results [TOTAL_RESULTS] = get_all_results();
    results[0] = result;

}
