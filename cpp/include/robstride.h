#pragma once
#include "can_interface.h"

class RobStride {
public:
    explicit RobStride(CanInterface& can, int motor_id);

    // ===== BASIC =====
    bool enable();
    bool stop();
    bool set_runmode(uint8_t mode);

    // ===== PP MODE =====
    bool pp_control(float pos);
    bool set_velocity_limit(float rad_s);
    bool set_acc_limit(float rad_s2);

    // ===== MIT MODE =====
    bool mit_control(double pos, double kp, double kd);

private:
    CanInterface& can_;
    int motor_id_;
};
