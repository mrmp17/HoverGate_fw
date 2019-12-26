//
// Created by Nejc on 2019-12-25.
//

#ifndef HOVERGATE_FW_GATE_H
#define HOVERGATE_FW_GATE_H

#include "BLDC_driver.h"
#include "pid.h"


class Gate {
public:
    enum class GateState {
        closed = 0,
        opening = 1,
        open = 2,
        closing = 3,
        undefined = 4,
    };

    Gate();
    void begin();
    void open();
    void close();
    GateState get_state();
    void loop();
    void set_pid(double kp, double ki, double dt);
    void set_driver(BLDC_driver *driver);

private:
    GateState _state = GateState::closed;
    BLDC_driver *_driver;
    Pid *_pid;
    bool _initialized = false;
};


#endif //HOVERGATE_FW_GATE_H
