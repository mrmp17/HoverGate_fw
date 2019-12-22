//
// Created by matej on 22/12/2019.
//

#ifndef HOVERGATE_FW_BLDC_DRIVER_H
#define HOVERGATE_FW_BLDC_DRIVER_H


#include <stdint.h>
#include "tim.h"

#define PHASE_Y 1
#define PHASE_B 2
#define PHASE_G 3
#define PHASE_LOW 0
#define PHASE_PWM 1
#define PHASE_FLOAT 2

#define BLDC_MAX_PWM 1900

extern TIM_HandleTypeDef htim1;

class BLDC_driver {
public:
    BLDC_driver();
    void begin();
    void enable();
    bool is_enabled();
    void disable();
    void set_pwm(int16_t pwm);  //+- 1000 //TODO: map to 0-2k timer values
    int32_t get_encoder();

    void interrupt_handler();

    uint8_t forceStep = 1;


private:
    void phase_set_low(uint8_t phase);
    void phase_set_float(uint8_t phase);
    void phase_set_pwm(uint8_t phase, uint16_t pwmSetCompare);
    void set_phases(uint8_t step, uint16_t pwmSetCompare, bool direction, bool is_enabled);

    bool BLDC_enabled = false;
    bool BLDC_direction = false;
    uint16_t BLDC_pwm_set_value = 0; //this should be 0-2k - direct pwm value
    uint8_t phase_states [4] = {0,PHASE_FLOAT,PHASE_FLOAT,PHASE_FLOAT}; //use PHASE_X defines to get state from array


};


#endif //HOVERGATE_FW_BLDC_DRIVER_H
