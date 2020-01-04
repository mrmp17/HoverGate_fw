//
// Created by Nejc on 2019-12-25.
//

#ifndef HOVERGATE_FW_GATE_H
#define HOVERGATE_FW_GATE_H

#include "driver.h"
#include "pid.h"
#include "math.h"
#include "debug.h"

struct gate_params {
    uint16_t loop_dt = 10; // milliseconds between loops

    double enc_ticks_per_deg = 2.725; // encoder ticks per degree of gate angle
    double angle_open = -80.0; // angle when gate open
    double angle_closed = 0.0; // angle when gate closed

    double target_velocity = 15.0; // target opening/closing speed in deg/s
    double target_velocity_slow = 7.5; // final movement reduced velocity

    int8_t driver_open_dir = -1; // driver pwm sign for open direction. 1 or -1.
    uint16_t max_pwm = 150; // max driver pwm

    double pid_kp = 15;
    double pid_ki = 4;
    double pid_slow_kp = 10;
    double pid_slow_ki = 2;

    uint16_t vel_update_tick_num = 5;
    uint16_t zero_vel_timeout = 2000;

    double move_uncert_before = 20.0; // degrees before target when velocity is reduced
    double move_uncert_after = 20.0; // degrees after target when velocity still set

    double max_angle_follow_error = 10.0; // max error when gate stopped is detected
};

class Gate {
public:
    enum class GateState {
        closed = 0,
        opening = 1,
        open = 2,
        closing = 3,
        error = 4
    };

    Gate();
    Gate(gate_params params);
    void begin();
    void open();
    void close();
    void toggle();
    GateState get_state();
    void loop();
    void set_driver(Driver *driver);
    void reset();

private:
    struct move {
        uint32_t start_time;
        double_t target;
        uint32_t stage_1_end;
        uint32_t stage_2_end;
        double stage_1_k;
        double stage_1_n;
        double stage_2_k;
        double stage_2_n;
        // Move status: 0 not started, 1 in progress, 2 stopped in expected zone,
        // 3 stopped before expected zone, 4 not stopped in expected zone
        uint8_t status;
    };

    // private variables
    uint32_t time = 0; // internal gate time in ms
    GateState state = GateState::closed;
    GateState prev_state = GateState::closed;
    Driver *driver = nullptr;
    Pid *pid;
    bool initialized = false;
    bool motor_enabled = false;
    double angle = 0.0;
    double angle_offset = 0.0;
    double velocity = 0.0;
    move active_move = {};
    int32_t last_vel_tick = 0;
    uint32_t last_tick_change_time = 0;


    // private functions
    void move_(double target);
    void set_pwm_(int16_t pwm);
    void set_pid_(double kp, double ki);
    void enable_motor_();
    void disable_motor_();


    // parameters
    uint16_t loop_dt = 10; // milliseconds between loops

    double enc_ticks_per_deg = 2.725; // encoder ticks per degree of gate angle
    double angle_open = -80.0; // angle when gate open
    double angle_closed = 0.0; // angle when gate closed

    double target_velocity = 15.0; // target opening/closing speed in deg/s
    double target_velocity_slow = 7.5; // final movement reduced velocity

    int8_t driver_open_dir = -1; // driver pwm sign for open direction. 1 or -1.
    uint16_t max_pwm = 150; // max driver pwm

    double pid_kp = 15;
    double pid_ki = 4;
    double pid_slow_kp = 10;
    double pid_slow_ki = 2;

    uint16_t vel_update_tick_num = 5;
    uint16_t zero_vel_timeout = 2000;

    double move_uncert_before = 20.0; // degrees before target when velocity is reduced
    double move_uncert_after = 20.0; // degrees after target when velocity still set

    double max_angle_follow_error = 10.0; // max error when gate stopped is detected
};


#endif //HOVERGATE_FW_GATE_H
