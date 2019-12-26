//
// Created by Nejc on 2019-12-25.
//

#include "gate.h"

Gate::Gate() {
    _pid = new Pid();
}

void Gate::begin() {
    if (_driver == nullptr) return;

    _driver->begin();
    _initialized = true;
}

void Gate::open() {
    switch (_state) {
        case GateState::closed: {

        } break;
        case GateState::closing: {

        } break;
        default: break;
    }
}

void Gate::close() {

}

Gate::GateState Gate::get_state() {
    return _state;
}

void Gate::loop() {
    switch (_state) {
        case GateState::closed: {

        } break;
        case GateState::opening: {

        } break;
        case GateState::open: {

        }
        case GateState::closing: {

        } break;
        case GateState::undefined: {

        } break;
    }
}

void Gate::set_pid(double kp, double ki, double dt) {
    _pid->set_parameters(kp, ki, 0.0, dt);
}

void Gate::set_driver(BLDC_driver *driver) {
    _driver = driver;
}
