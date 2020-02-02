//
// Created by Nejc on 2020-02-02.
//

#ifndef HOVERGATE_FW_LATCH_H
#define HOVERGATE_FW_LATCH_H


#include <cstdint>
#include "gpio.h"

class Latch {
public:
    Latch();
    void retract();
    void extend();
    void handler();
private:
    const uint32_t latch_full_current_time = 800; // ms
    uint8_t cmd_state = 0;

};


#endif //HOVERGATE_FW_LATCH_H
