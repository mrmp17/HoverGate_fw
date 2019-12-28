//
// Created by Nejc on 2019-12-28.
//

#ifndef HOVERGATE_FW_DRIVER_H
#define HOVERGATE_FW_DRIVER_H

#include <stdint.h>

class Driver {
public:
    Driver() {};
    virtual void begin() {};
    virtual void enable() {};
    virtual bool is_enabled() {return false;};
    virtual void disable() {};
    virtual void set_pwm(int16_t pwm) {};
    virtual int16_t get_pwm() {return 0;};
    virtual int32_t get_encoder() {return 0;};
    virtual void reset_encoder() {};
    virtual float get_current() {return 0.0;};
    virtual void ramp_pwm(int16_t pwm_to, uint32_t time_ms) {};
    virtual bool is_ramp_active() {return false;};
};

#endif //HOVERGATE_FW_DRIVER_H
