/*
 * RobStride Private Protocol - Location Mode (PP)
 *
 * Uses "communication type 18" parameter write:
 *  - runmode (index 0x7005) = 1  -> PP
 *  - vel_max (0x7024)            -> PP speed limit (rad/s)
 *  - acc_set (0x7025)            -> PP acceleration (rad/s^2)
 *  - loc_ref (0x7016)            -> target position (rad)
 *
 * Build:
 *   g++ -O2 -std=c++17 -o position_control_pp_private position_control_pp_private.cpp
 *
 * Run:
 *   sudo ./position_control_pp_private <motor_id>
 */

#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cmath>

// Linux SocketCAN
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "../include/protocol.h"  // Including the provided protocol.h

using namespace std::chrono_literals;

// ================== GLOBALS ==================
std::atomic<bool> running(true);

const char* CAN_INTERFACE = "can0";
const int MOTOR_ID_DEFAULT = 1;
const int HOST_ID = 0xFF;

// ================== CAN IO ===================
bool send_frame(int s, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    struct can_frame frame{};
    frame.can_id = can_id | CAN_EFF_FLAG;  // 29-bit extended
    frame.can_dlc = dlc;

    if (data && dlc > 0) std::memcpy(frame.data, data, dlc);
    else std::memset(frame.data, 0, 8);

    ssize_t n = write(s, &frame, sizeof(frame));
    if (n != (ssize_t)sizeof(frame)) {
        perror("write");
        return false;
    }
    return true;
}

int init_can(const char* ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return -1; }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX)");
        close(s);
        return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }
    return s;
}

// ================== MOTOR CMDS =================
bool enable_motor(int s, int motor_id) {
    // manual sample uses mode=3, data=master_id, dlc=8 (data cleared)
    // but many implementations send dlc=0 too; we'll send dlc=8 zeros for safety
    uint8_t data[8] = {0};
    uint32_t ext_id = (CommType::ENABLE << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    return send_frame(s, ext_id, data, 8);
}

bool stop_motor(int s, int motor_id) {
    uint8_t data[8] = {0};
    uint32_t ext_id = (CommType::DISABLE << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    return send_frame(s, ext_id, data, 8);
}

// Type 18 "running mode switch" (write index + runmode byte at data[4]) :contentReference[oaicite:12]{index=12}
bool set_runmode(int s, int motor_id, uint8_t runmode_val) {
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], ParamID::MODE); // index
    data[4] = runmode_val;                // runmode (uint8)
    uint32_t ext_id = (CommType::WRITE_PARAMETER << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    return send_frame(s, ext_id, data, 8);
}

// Type 18 control parameter write: index at [0..1], float at [4..7] :contentReference[oaicite:13]{index=13}
bool write_param_f32(int s, int motor_id, uint16_t index, float value) {
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], index);
    pack_float_le(&data[4], value);
    uint32_t ext_id = (CommType::WRITE_PARAMETER << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    return send_frame(s, ext_id, data, 8);
}

// PP target position = write loc_ref (rad) in private protocol PP mode :contentReference[oaicite:14]{index=14}
bool pp_set_target_rad(int s, int motor_id, float pos_rad) {
    return write_param_f32(s, motor_id, ParamID::POSITION_TARGET, pos_rad);
}

// Function to read motor status (position, speed, torque, temperature)
bool read_motor_status(int s, int motor_id) {
    uint8_t data[8] = {0};  // Prepare buffer to hold data received from the motor
    uint32_t ext_id = (CommType::READ_PARAMETER << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    
    // Request position data by sending the frame to motor (Type 2)
    send_frame(s, ext_id, data, 8);  // Assuming send_frame function is defined elsewhere

    // Read response from the motor
    struct can_frame frame;
    int nbytes = read(s, &frame, sizeof(frame));
    if (nbytes <= 0) {
        std::cerr << "❌ Error reading from CAN bus: " << nbytes << "\n";
        return false;
    }

    // Extract the motor feedback data from the response frame
    uint16_t current_angle = (frame.data[0] << 8) | frame.data[1]; // Byte0~Byte1
    uint16_t current_speed = (frame.data[2] << 8) | frame.data[3]; // Byte2~Byte3
    uint16_t current_torque = (frame.data[4] << 8) | frame.data[5]; // Byte4~Byte5
    uint16_t current_temp = (frame.data[6] << 8) | frame.data[7]; // Byte6~Byte7

    // Convert the data to real-world units based on the datasheet
    float angle_rad = (current_angle / 65535.0f) * (2 * M_PI) - M_PI;  // Angle: -π to π rad
    float speed_rad_per_s = (current_speed / 65535.0f) * 66.0f - 33.0f;  // Speed: -33 to 33 rad/s
    float torque_Nm = (current_torque / 65535.0f) * 28.0f - 14.0f;  // Torque: -14 to 14 N·m
    float temp_celsius = current_temp / 10.0f;  // Temperature: in Celsius

    std::cout << "Motor ID: " << motor_id << "\n";
    std::cout << "Angle: " << angle_rad << " rad\n";
    std::cout << "Speed: " << speed_rad_per_s << " rad/s\n";
    std::cout << "Torque: " << torque_Nm << " N·m\n";
    std::cout << "Temperature: " << temp_celsius << " °C\n";

    return true;
}

// To read the motor fault status (communication type 21)
bool read_motor_fault(int s, int motor_id) {
    uint8_t data[8] = {0};  // Prepare buffer to hold fault data
    uint32_t ext_id = (CommType::READ_PARAMETER << 24) | (HOST_ID << 8) | (uint8_t)motor_id;

    send_frame(s, ext_id, data, 8);  // Send frame to query fault status

    struct can_frame frame;
    int nbytes = read(s, &frame, sizeof(frame));
    if (nbytes <= 0) {
        std::cerr << "❌ Error reading from CAN bus: " << nbytes << "\n";
        return false;
    }

    uint32_t fault_status = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];

    // Check for faults, based on fault bits
    if (fault_status & (1 << 0)) std::cout << "Motor overtemperature detected.\n";
    if (fault_status & (1 << 3)) std::cout << "Motor encoder uncalibrated.\n";
    if (fault_status & (1 << 8)) std::cout << "Motor hardware identification fault.\n";
    // Add additional checks for other fault conditions as needed...

    return true;
}


// ================== SIGNAL ====================
void signal_handler(int) {
    std::cout << "\n🛑 Exit signal received\n";
    running = false;
}

// ================== MAIN ======================
int main(int argc, char* argv[]) {
    int motor_id = MOTOR_ID_DEFAULT;
    if (argc > 1) motor_id = std::atoi(argv[1]);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::cout << "🎯 RobStride Private Protocol PP Controller\n";
    std::cout << "📡 Motor ID: " << motor_id << " | IF: " << CAN_INTERFACE << "\n";

    int s = init_can(CAN_INTERFACE);
    if (s < 0) {
        std::cerr << "❌ CAN init failed\n";
        return 1;
    }

    std::cout << "🛑 Stop/reset motor...\n";
    stop_motor(s, motor_id);
    std::this_thread::sleep_for(300ms);

    std::cout << "⚙️ Set runmode = 1 (PP)...\n";
    if (!set_runmode(s, motor_id, 1)) {
        std::cerr << "❌ Failed to set runmode\n";
    }
    std::this_thread::sleep_for(300ms);

    std::cout << "⚡ Enable motor...\n";
    if (!enable_motor(s, motor_id)) {
        std::cerr << "❌ Failed to enable motor\n";
    }
    std::this_thread::sleep_for(300ms);

    // Set parameters (optional)
    float vel_max = 20.0f;   // rad/s
    float acc_set = 10.0f;  // rad/s^2
    write_param_f32(s, motor_id, ParamID::PP_VELOCITY_MAX, vel_max);
    std::this_thread::sleep_for(50ms);
    write_param_f32(s, motor_id, ParamID::PP_ACCELERATION_TARGET, acc_set);
    std::this_thread::sleep_for(50ms);

    // Start reading the motor status in a loop
    std::cout << "✅ Ready to read motor status...\n";
    while (running) {
        // Optionally add a delay between each status check
        std::this_thread::sleep_for(1s);  // Read status every 1 second
        
        // Read and print the motor status
        if (!read_motor_status(s, motor_id)) {
            std::cerr << "❌ Failed to read motor status\n";
        }

        // Read and print the motor fault status
        if (!read_motor_fault(s, motor_id)) {
            std::cerr << "❌ Failed to read motor fault status\n";
        }
    }

    // Ensure motor stops before exiting
    std::cout << "🏁 Returning home...\n";
    pp_set_target_rad(s, motor_id, 0.0f);
    std::this_thread::sleep_for(800ms);

    std::cout << "🛑 Stop motor...\n";
    stop_motor(s, motor_id);
    std::this_thread::sleep_for(200ms);

    close(s);
    std::cout << "👋 Program exited cleanly\n";
    return 0;
}
