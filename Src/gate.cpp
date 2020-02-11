//
// Created by Nejc on 2019-12-25.
//

#include "gate.h"

Gate::Gate() {
    pid = new Pid();
}

Gate::Gate(gate_params params) {
    loop_dt = params.loop_dt;
    enc_ticks_per_deg = params.enc_ticks_per_deg;
    angle_open = params.angle_open;
    angle_closed = params.angle_closed;
    target_velocity = params.target_velocity;
    target_velocity_slow = params.target_velocity_slow;
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
    switch(move_state_ctrl) {
        case 0:
            if(active_move.status == 1) {
                pid->reset();
                set_pid_(pid_kp, pid_ki);
                enable_motor_();
                if (latch != nullptr) latch->retract();
                move_state_ctrl = 1;
            }
            break;
        case 1: // first stage
            setpoint = time * active_move.stage_1_k + active_move.stage_1_n;
            if(time > active_move.stage_1_end) {
                set_pid_(pid_slow_kp, pid_slow_ki);
                move_state_ctrl = 2;
                break;
            }
            else if(abs(angle - setpoint) > max_angle_follow_error) {
                // error - stopped before expected zone
                active_move.status = 3;
                set_pwm_(0);
                disable_motor_();
                if (latch != nullptr) latch->extend();
                debug_print("GATE stopped before expected\n");
                move_state_ctrl = 0;
                break;
            }
            break;
        case 2: // second stage
            setpoint = time * active_move.stage_2_k + active_move.stage_2_n;
            if(time > active_move.stage_2_end) {
                active_move.status = 4;
                set_pwm_(0);
                disable_motor_();
                if (latch != nullptr) latch->extend();
                debug_print("GATE did not stop\n");
                move_state_ctrl = 0;
                break;
            }
            else if(abs(angle - setpoint) > max_angle_follow_error) {
                // stopped in expected zone
                active_move.status = 2;
                angle_offset += active_move.target - angle;
                if (latch != nullptr) latch->extend();
                setpoint = active_move.target;
                debug_print("GATE stopped in expected zone\n");
                move_state_ctrl = 0;
                break;
            }
            break;
    }

    if(motor_enabled) {
        double pwm = pid->compute(angle, setpoint);
        set_pwm_((int16_t) pwm);
    }

    if (latch != nullptr) latch->handler();

}

void Gate::set_pid_(double kp, double ki) {
    pid->set_parameters(kp, ki, 0.0, 1./loop_dt);
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
    double  start_angle = angle;
    uint32_t start_time = time;

    double t1, t2, dist_1, dist_2;
    if(target >= start_angle) {
        t1 = target - move_uncert_before;
        t2 = target + move_uncert_after;
        dist_1 = t1 - start_angle;
        dist_2 = t2 - t1;
    }
    else {
        t1 = target + move_uncert_before;
        t2 = target - move_uncert_after;
        dist_1 = t1 - start_angle;
        dist_2 = t2 - t1;
    }

    uint32_t time_1 = abs(dist_1) / target_velocity * 1000;
    double k1 = dist_1 / time_1;
    double n1 = start_angle - k1 * start_time;

    uint32_t time_2 = abs(dist_2) / target_velocity_slow * 1000;
    double k2 = dist_2 / time_2;
    double n2 = t1 - k2 * (start_time + time_1);

    move new_move = {start_time,
                     target,
                     start_time + time_1,
                     start_time + time_1 + time_2,
                     k1,
                     n1,
                     k2,
                     n2,
                     1};
    active_move = new_move;
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
