//
// Created by Nejc on 2019-12-25.
//

#include "gate.h"
#include <math.h>

Gate::Gate() {
    pid = new Pid();
}

Gate::Gate(gate_params params) {
    loop_dt = params.loop_dt;
    enc_ticks_per_deg = params.enc_ticks_per_deg;
    angle_open = params.angle_open;
    angle_closed = params.angle_closed;
    a_max = params.a_max; // max acceleration
    v_max = params.v_max; // max velocity
    v_min = params.v_min;
    driver_open_dir = params.driver_open_dir;
    max_pwm = params.max_pwm;
    pid_kp = params.pid_kp;
    pid_ki = params.pid_ki;
    pid_slow_kp = params.pid_slow_kp;
    pid_slow_ki = params.pid_slow_ki;
    vel_update_tick_num = params.vel_update_tick_num;
    zero_vel_timeout = params.zero_vel_timeout;
    move_uncert_before = params.move_uncert_before;
    move_uncert_after = params.move_uncert_after;
    max_angle_follow_error = params.max_angle_follow_error;
    hold_open_offset = params.hold_open_offset;

    pid = new Pid();

}

void Gate::begin() {
    if (driver == nullptr) return;

    driver->begin();
    pid->start();

    state = GateState::closed;
    angle = 0.0;

    velocity = 0.0;
    last_vel_tick = 0;
    last_tick_change_time = 0;

    initialized = true;
    prev_state = GateState::closed;
    debug_print("GATE initialized\n");

}

void Gate::open() {
    switch (state) {
        case GateState::closed:
        case GateState::closing: {
            move_(angle_open);
            state = GateState::opening;
            debug_print("GATE open start\n");
        } break;
        default: break;
    }
}

void Gate::close() {
    switch (state) {
        case GateState::open:
        case GateState::opening: {
            move_(angle_closed);
            state = GateState::closing;
            debug_print("GATE close start\n");
        } break;
        default: break;
    }
}

void Gate::toggle() {
    switch(get_state()) {
        case Gate::GateState::closing:
        case Gate::GateState::closed:
            open();
            break;
        case Gate::GateState::opening:
        case Gate::GateState::open:
            close();
            break;
    }
}

void Gate::stop() {
    state = GateState::error;
    error_code = 3;
    set_pwm_(0);
    disable_motor_();
}

Gate::GateState Gate::get_state() {
    return state;
}

float Gate::get_angle() {
    return (float)angle;
}

void Gate::loop() {
    if(!initialized) return;

    if(state != prev_state) {
        debug_print("GATE state changed to: %d \n",  static_cast<uint8_t >(state));
        prev_state = state;
    }

    // angle
    time += loop_dt;
    time_sec = time / 1000.0;
    int32_t ticks = driver->get_encoder();
    angle = ticks / enc_ticks_per_deg + angle_offset;

    // velocity
    if(ticks - last_vel_tick >= vel_update_tick_num) {
        uint32_t dt = time - last_tick_change_time;
        if(dt >= zero_vel_timeout) {
            velocity = 0.0;
        }
        else {
            velocity = (vel_update_tick_num / (double)dt) / enc_ticks_per_deg;
        }
        last_vel_tick = ticks;
        last_tick_change_time = time;
    }

    static double setpoint = 0.0;

    switch (state) {
        case GateState::closed: {

        } break;
        case GateState::opening: {
            switch(active_move.status) {
                case 2:
                    state = GateState::open;
                    setpoint = angle_open + hold_open_offset;
                    break;
                case 3:
                    state = GateState::error;
                    error_code = 1;
                    break;
                case 4:
                    state = GateState::error;
                    error_code = 2;
                    break;
            }
        } break;
        case GateState::open: {

        } break;
        case GateState::closing: {
            switch(active_move.status) {
                case 2:
                    state = GateState::closed;
                    disable_motor_();
                    break;
                case 3:
                    state = GateState::error;
                    error_code = 1;
                    break;
                case 4:
                    state = GateState::error;
                    error_code = 2;
                    break;
            }
        } break;
        case GateState::error: {
            return;
        } break;
    }

    // Move switch
    switch(active_move.profile_type) {
        case 0: // ramp
            switch(move_state_ctrl) {
                case 0: { // start move
                    if (active_move.status == 1) {
                        setpoint = active_move.s0;
                        set_pid_(pid_slow_kp, pid_slow_ki);
                        enable_motor_();
                        if (latch != nullptr) latch->retract();
                        move_state_ctrl = 1;
                    }
                }
                break; // case 0
                case 1: { // ramp
                    setpoint = active_move.a_max * pow((time_sec - active_move.t0), 2) / 2 + active_move.s0;
                    if (time_sec > active_move.t1) {
                        move_state_ctrl = 2;
                    }
                    else if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stopped before expected zone
                        active_move.status = 3;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 1
                case 2: { // constant vel
                    setpoint = active_move.v_min * (time_sec - active_move.t1) + active_move.s1;
                    if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stoped in expected zone
                        active_move.status = 2;
                        angle_offset += active_move.target - angle;
                        if (latch != nullptr) latch->extend();
                        setpoint = active_move.target;
                        move_state_ctrl = 0;
                    }
                    else if (time_sec > active_move.t2) {
                        // did not stop in expected zone
                        active_move.status = 4;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 2
            }
            break; // case 0 - ramp profile
        case 1: // triangular
            switch(move_state_ctrl) {
                case 0: {
                    if (active_move.status == 1) {
                        setpoint = active_move.s0;
                        set_pid_(pid_kp, pid_ki);
                        enable_motor_();
                        if (latch != nullptr) latch->retract();
                        move_state_ctrl = 1;
                    }
                }
                break; // case 0
                case 1: { // ramp up
                    setpoint = active_move.a_max * pow((time_sec - active_move.t0), 2) / 2 + active_move.s0;
                    if (time_sec > active_move.t1) {
                        move_state_ctrl = 2;
                    }
                    else if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stopped before expected zone
                        active_move.status = 3;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 1
                case 2: { // ramp down
                    setpoint = -active_move.a_max * pow((time_sec - active_move.t1), 2) / 2 +
                            active_move.v_max * (time_sec - active_move.t1) + active_move.s1;
                    if (time_sec > active_move.t2) {
                        set_pid_(pid_slow_kp, pid_slow_ki);
                        move_state_ctrl = 3;
                    }
                    else if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stopped before expected zone
                        active_move.status = 3;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 2
                case 3: { // const vel
                    setpoint = active_move.v_min * (time_sec - active_move.t2) + active_move.s2;
                    if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stoped in expected zone
                        active_move.status = 2;
                        angle_offset += active_move.target - angle;
                        if (latch != nullptr) latch->extend();
                        setpoint = active_move.target;
                        move_state_ctrl = 0;
                    }
                    else if (time_sec > active_move.t3) {
                        // did not stop in expected zone
                        active_move.status = 4;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 3
            }
            break; // case 1 - triangular profile
        case 2: // trapezoidal profile
            switch(move_state_ctrl) {
                case 0: {
                    if (active_move.status == 1) {
                        setpoint = active_move.s0;
                        set_pid_(pid_kp, pid_ki);
                        enable_motor_();
                        if (latch != nullptr) latch->retract();
                        move_state_ctrl = 1;
                    }
                }
                    break; // case 0
                case 1: { // ramp up
                    setpoint = active_move.a_max * pow((time_sec - active_move.t0), 2) / 2 + active_move.s0;
                    if (time_sec > active_move.t1) {
                        move_state_ctrl = 2;
                    }
                    else if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stopped before expected zone
                        active_move.status = 3;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 1
                case 2: { // const vel
                    setpoint = active_move.v_max * (time_sec - active_move.t1) + active_move.s1;
                    if (time_sec > active_move.t2) {
                        move_state_ctrl = 3;
                    }
                    else if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stopped before expected zone
                        active_move.status = 3;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 2
                case 3: { // ramp down
                    setpoint = -active_move.a_max * pow((time_sec - active_move.t2), 2) / 2 +
                               active_move.v_max * (time_sec - active_move.t2) + active_move.s2;
                    if (time_sec > active_move.t3) {
                        set_pid_(pid_slow_kp, pid_slow_ki);
                        move_state_ctrl = 4;
                    }
                    else if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stopped before expected zone
                        active_move.status = 3;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 3
                case 4: { // const vel
                    setpoint = active_move.v_min * (time_sec - active_move.t3) + active_move.s3;
                    if (abs(angle - setpoint) > max_angle_follow_error) {
                        // stoped in expected zone
                        active_move.status = 2;
                        angle_offset += active_move.target - angle;
                        if (latch != nullptr) latch->extend();
                        setpoint = active_move.target;
                        move_state_ctrl = 0;
                    }
                    else if (time_sec > active_move.t4) {
                        // did not stop in expected zone
                        active_move.status = 4;
                        set_pwm_(0);
                        disable_motor_();
                        if (latch != nullptr) latch->extend();
                        move_state_ctrl = 0;
                    }
                }
                break; // case 4
            }
            break; // case 3 - trapezoidal profile
    }


    if(motor_enabled) {
        double pwm = pid->compute(angle, setpoint);
        set_pwm_((int16_t) pwm);
    }

    if (latch != nullptr) latch->handler();

}

void Gate::set_pid_(double kp, double ki) {
    pid->set_parameters(kp, ki, 0.0, pid_imax, 1./loop_dt);
}

void Gate::set_driver(Driver *new_driver) {
    driver = new_driver;
}

void Gate::set_latch(Latch *new_latch) {
    latch = new_latch;
}

uint8_t Gate::get_error_code() {
    return error_code;
}


/**
 * Reset gate to closed position.
 */
void Gate::reset() {
    active_move = {};
    move_state_ctrl = 0;
    set_pwm_(0);
    pid->reset();
    angle_offset = 0.0;
    angle = 0.0;
    velocity = 0.0;
    driver->reset_encoder();
    disable_motor_();
    if (latch != nullptr) latch->extend();
    state = GateState::closed;
    error_code = 0;
}

void Gate::move_(double target) {
    double s = target - angle;
    double s_first;
    double s_end;
    if (s >= 0) {
        if (s > move_uncert_before) {
            s_first = s - move_uncert_before;
        }
        else {
            s_first = s;
        }
        s_end = s + move_uncert_after;
    }
    else {
        if (s < -move_uncert_before) {
            s_first = s + move_uncert_before;
        }
        else {
            s_first = s;
        }
        s_end = s - move_uncert_after;
    }

    double t0 = time_sec;
    double s0 = angle;
    int8_t dir = (s_first < 0) ? -1 : 1;

    // profile selection
    double s01 = 0.5 * pow(v_max, 2) / a_max;
    double s12 = 0.5 * (v_max - v_min) / a_max * (v_max + v_min);
    double s_triang = s01 + s12; // max possible distance using triangular profile
    double s_min = 0.5 * pow(v_min, 2) / a_max; // max possible distance using ramp profile

    // direction
    a_max = dir * a_max;
    v_max = dir * v_max;
    v_min = dir * v_min;
    s_triang = s_triang * dir;
    s_min = s_min * dir;

    if (abs(s_first) <= abs(s_min)) {
        // ramp profile
        double v_min_limit = sqrt(2 * s * a_max) * dir;

        // 0-1
        double t01 = v_min_limit / a_max;
        double t1 = t0 + t01;
        double s01 = t01 * v_min_limit / 2;
        double s1 = s0 + s01;

        // 1-2
        double t12 = (s_end - s_first) / v_min_limit;
        double t2 = t1 + t12;

        move new_move = {
                .profile_type = 0,
                .target = target,
                .t0 = t0,
                .s0 = s0,
                .t1 = t1,
                .s1  =s1,
                .t2 = t2,
                .s2 = 0,
                .t3 = 0,
                .s3 = 0,
                .t4 = 0,
                .s4 = 0,
                .v_max = 0,
                .v_min = v_min_limit,
                .a_max = a_max,
                .status = 1
        };
        active_move = new_move;
    }
    else if (abs(s_first) <= abs(s_triang)) {
        // triangular profile
        double v_max_triang = sqrt(a_max * s_first + pow(v_min, 2) / 2) * dir;

        // 0-1
        double t01 = v_max_triang / a_max;
        double t1 = t0 + t01;
        double s01 = t01 * v_max_triang / 2;
        double s1 = s0 + s01;

        // 1-2
        double t12 = (v_max_triang - v_min) / a_max;
        double t2 = t1 + t12;
        double s12 = t12 * (v_max_triang - v_min) / 2 + t12 * v_min;
        double s2 = s1 + s12;

        // 2-3
        double t23 = (s_end - s_first) / v_min;
        double t3 = t2 + t23;

        move new_move = {
                .profile_type = 1,
                .target = target,
                .t0 = t0,
                .s0 = s0,
                .t1 = t1,
                .s1 = s1,
                .t2 = t2,
                .s2 = s2,
                .t3 = t3,
                .s3 = 0,
                .t4 = 0,
                .s4 = 0,
                .v_max = v_max_triang,
                .v_min = v_min,
                .a_max = a_max,
                .status = 1
        };
        active_move = new_move;
    }
    else {
        // trapezoidal profile
        double s_v_const = s_first - s_triang;

        // 0-1
        double t01 = v_max / a_max;
        double t1 = t0 + t01;
        double s01 = t01 * v_max / 2;
        double s1 = s0 + s01;

        // 1-2
        double t12 = s_v_const / v_max;
        double t2 = t1 + t12;
        double s12 = t12 * v_max;
        double s2 = s1 + s12;

        // 2-3
        double t23 = (v_max - v_min) / a_max;
        double t3 = t2 + t23;
        double s23 = t23 * (v_max - v_min) / 2 + t23 * v_min;
        double s3 = s2 + s23;

        // 3-4
        double t34 = (s_end - s_first) / v_min;
        double t4 = t3 + t34;

        move new_move = {
                .profile_type = 2,
                .target = target,
                .t0 = t0,
                .s0 = s0,
                .t1 = t1,
                .s1 = s1,
                .t2 = t2,
                .s2 = s2,
                .t3 = t3,
                .s3 = s3,
                .t4 = t4,
                .s4 = 0,
                .v_max = v_max,
                .v_min = v_min,
                .a_max = a_max,
                .status = 1
        };
        active_move = new_move;
    }
    move_state_ctrl = 0;
}

void Gate::set_pwm_(int16_t pwm) {
    pwm *= driver_open_dir;
    if(pwm > max_pwm) pwm = max_pwm;
    else if(pwm < -max_pwm) pwm = -max_pwm;
    driver->set_pwm(pwm);
}

void Gate::enable_motor_() {
    driver->enable();
    motor_enabled = true;
}

void Gate::disable_motor_() {
    driver->disable();
    motor_enabled = false;
}
