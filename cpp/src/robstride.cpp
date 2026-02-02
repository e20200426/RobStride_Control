#include "../include/robstride.h"
#include "../include/protocol.h"

#include <algorithm>
#include <cmath>

RobStride::RobStride(CanInterface& can, int motor_id)
: can_(can), motor_id_(motor_id) {}

const int HOST_ID = 0xFF;

// ================= BASIC =================
bool RobStride::enable() {
    uint32_t id =
        (CommType::ENABLE << 24) |
        (HOST_ID << 8) |
        motor_id_;

    uint8_t data[8] = {0};
    return can_.send_frame(id, data, 8);
}

bool RobStride::stop() {
    uint32_t id =
        (CommType::DISABLE << 24) |
        (HOST_ID << 8) |
        motor_id_;

    uint8_t data[8] = {0};
    return can_.send_frame(id, data, 8);
}

bool RobStride::set_runmode(uint8_t mode) {
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], ParamID::MODE);
    data[4] = mode;

    uint32_t id =
        (CommType::WRITE_PARAMETER << 24) |
        (HOST_ID << 8) |
        motor_id_;

    return can_.send_frame(id, data, 8);
}

// ================= PP MODE =================
bool RobStride::pp_control(float pos) {
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], ParamID::POSITION_TARGET);
    pack_float_le(&data[4], pos);

    uint32_t id =
        (CommType::WRITE_PARAMETER << 24) |
        (HOST_ID << 8) |
        motor_id_;

    return can_.send_frame(id, data, 8);
}

bool RobStride::set_velocity_limit(float rad_s) {
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], ParamID::PP_VELOCITY_MAX);
    pack_float_le(&data[4], rad_s);

    uint32_t id =
        (CommType::WRITE_PARAMETER << 24) |
        (HOST_ID << 8) |
        motor_id_;

    return can_.send_frame(id, data, 8);
}

bool RobStride::set_acc_limit(float rad_s2) {
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], ParamID::PP_ACCELERATION_TARGET);
    pack_float_le(&data[4], rad_s2);

    uint32_t id =
        (CommType::WRITE_PARAMETER << 24) |
        (HOST_ID << 8) |
        motor_id_;

    return can_.send_frame(id, data, 8);
}

// ================= MIT MODE =================
bool RobStride::mit_control(double pos, double kp, double kd) {
    pos = std::clamp(pos, -ModelScale::POSITION, ModelScale::POSITION);
    kp  = std::clamp(kp,  0.0, ModelScale::KP);
    kd  = std::clamp(kd,  0.0, ModelScale::KD);

    uint16_t pos_u16 =
        static_cast<uint16_t>(((pos / ModelScale::POSITION) + 1.0) * 0x7FFF);
    uint16_t kp_u16 =
        static_cast<uint16_t>((kp / ModelScale::KP) * 0xFFFF);
    uint16_t kd_u16 =
        static_cast<uint16_t>((kd / ModelScale::KD) * 0xFFFF);

    uint8_t data[8];
    pack_u16_be(&data[0], pos_u16);
    pack_u16_be(&data[2], 0x7FFF);  // velocity = 0
    pack_u16_be(&data[4], kp_u16);
    pack_u16_be(&data[6], kd_u16);

    uint32_t id =
        (CommType::OPERATION_CONTROL << 24) |
        (0x7FFF << 8) |
        motor_id_;

    return can_.send_frame(id, data, 8);
}
