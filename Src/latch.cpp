//
// Created by Nejc on 2020-02-02.
//

#include "latch.h"

Latch::Latch() {

}

void Latch::retract() {
    cmd_state = 1;
}

void Latch::extend() {
    cmd_state = 0;
}

void Latch::handler() {
    static uint32_t retract_start_time = 0;
    static uint8_t loopCtrl = 0;
    switch(loopCtrl){
        case 0:
            //latch current zero
            HAL_GPIO_WritePin(LATCH_OFF_GPIO_Port, LATCH_OFF_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LATCH_HOLD_GPIO_Port, LATCH_HOLD_Pin, GPIO_PIN_RESET);
            if(cmd_state == 1) loopCtrl = 1;
            break;
        case 1:
            //set latch current to full
            HAL_GPIO_WritePin(LATCH_OFF_GPIO_Port, LATCH_OFF_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LATCH_HOLD_GPIO_Port, LATCH_HOLD_Pin, GPIO_PIN_SET);
            retract_start_time = HAL_GetTick();
            if(cmd_state == 0) loopCtrl = 0;
            loopCtrl = 2;
            break;
        case 2:
            if(HAL_GetTick() - retract_start_time >= latch_full_current_time){
                //set current to hold setting
                HAL_GPIO_WritePin(LATCH_OFF_GPIO_Port, LATCH_OFF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LATCH_HOLD_GPIO_Port, LATCH_HOLD_Pin, GPIO_PIN_RESET);
                loopCtrl = 3;
            }
            if(cmd_state == 0) loopCtrl = 0;
            break;
        case 3:
            if(cmd_state == 0) loopCtrl = 0;
            break;
    }
}
