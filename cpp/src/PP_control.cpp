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

using namespace std::chrono_literals;

// ================== GLOBALS ==================
std::atomic<bool> running(true);

const char* CAN_INTERFACE = "can0";
const int MOTOR_ID_DEFAULT = 1;
const int HOST_ID = 0xFF;

// ================== PRIVATE PROTOCOL (EXT ID) =================
// ext_id = (comm_type << 24) | (data_or_host << 8) | motor_id
// For type 3/4: data field carries master_id (HOST_ID)
// For type 18: data field carries master_id (HOST_ID)
// (Matches manual code samples using txCanIdEx.mode/id/data) :contentReference[oaicite:6]{index=6} :contentReference[oaicite:7]{index=7}

const uint32_t COMM_ENABLE          = 3;   // enable run frame
const uint32_t COMM_STOP            = 4;   // stop/reset frame
const uint32_t COMM_WRITE_PARAMETER = 18;  // single parameter write

// Parameter indices (private protocol table)
const uint16_t PARAM_RUNMODE = 0x7005; // runmode (written as uint8 at byte4 in "modechange") :contentReference[oaicite:8]{index=8}
const uint16_t PARAM_LOC_REF = 0x7016; // loc_ref target position (float rad) :contentReference[oaicite:9]{index=9}
const uint16_t PARAM_VEL_MAX = 0x7024; // vel_max (float rad/s) :contentReference[oaicite:10]{index=10}
const uint16_t PARAM_ACC_SET = 0x7025; // acc_set (float rad/s^2) :contentReference[oaicite:11]{index=11}

// ================== HELPERS ==================
static inline void pack_u16_le(uint8_t* buf, uint16_t val) {
    std::memcpy(buf, &val, sizeof(uint16_t));
}
static inline void pack_float_le(uint8_t* buf, float val) {
    std::memcpy(buf, &val, sizeof(float));
}

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
    uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    return send_frame(s, ext_id, data, 8);
}

bool stop_motor(int s, int motor_id) {
    uint8_t data[8] = {0};
    uint32_t ext_id = (COMM_STOP << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    return send_frame(s, ext_id, data, 8);
}

// Type 18 "running mode switch" (write index + runmode byte at data[4]) :contentReference[oaicite:12]{index=12}
bool set_runmode(int s, int motor_id, uint8_t runmode_val) {
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], PARAM_RUNMODE); // index
    data[4] = runmode_val;                // runmode (uint8)
    uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    return send_frame(s, ext_id, data, 8);
}

// Type 18 control parameter write: index at [0..1], float at [4..7] :contentReference[oaicite:13]{index=13}
bool write_param_f32(int s, int motor_id, uint16_t index, float value) {
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], index);
    pack_float_le(&data[4], value);
    uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | (uint8_t)motor_id;
    return send_frame(s, ext_id, data, 8);
}

// PP target position = write loc_ref (rad) in private protocol PP mode :contentReference[oaicite:14]{index=14}
bool pp_set_target_rad(int s, int motor_id, float pos_rad) {
    return write_param_f32(s, motor_id, PARAM_LOC_REF, pos_rad);
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

    // ---- According to manual PP steps:
    // runmode=1 while disabled -> enable -> set vel_max -> set acc_set -> set loc_ref :contentReference[oaicite:15]{index=15}

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

    // NOTE: manual says PP does NOT support changing speed/acc during operation.
    // So set these once (or re-set before the next move if you want). :contentReference[oaicite:16]{index=16}
    float vel_max = 20.0f;   // rad/s
    float acc_set = 10.0f;  // rad/s^2

    std::cout << "⚙️ Set vel_max=" << vel_max << " rad/s\n";
    write_param_f32(s, motor_id, PARAM_VEL_MAX, vel_max);
    std::this_thread::sleep_for(50ms);

    std::cout << "⚙️ Set acc_set=" << acc_set << " rad/s^2\n";
    write_param_f32(s, motor_id, PARAM_ACC_SET, acc_set);
    std::this_thread::sleep_for(50ms);

    std::cout << "✅ Ready (Private Protocol PP)\n";
    std::cout << "----------------------------------\n";
    std::cout << "👉 Enter angle in degrees\n";
    std::cout << "👉 'vel <rad/s>' to change vel_max (writes param)\n";
    std::cout << "👉 'acc <rad/s2>' to change acc_set (writes param)\n";
    std::cout << "👉 'home' or '0' -> 0°\n";
    std::cout << "👉 'q' -> quit\n";
    std::cout << "----------------------------------\n";

    std::string line;
    while (running) {
        std::cout << ">> ";
        if (!std::getline(std::cin, line)) break;

        if (line == "q" || line == "quit" || line == "exit") {
            running = false;
            break;
        }

        if (line == "home" || line == "0") {
            pp_set_target_rad(s, motor_id, 0.0f);
            std::cout << "🏠 Move to 0° (loc_ref=0 rad)\n";
            continue;
        }

        // optional: allow user to write vel/acc (manual warns this is not supported "during operation",
        // but you can still write; some firmware accepts it, some ignores it). :contentReference[oaicite:17]{index=17}
        if (line.rfind("vel ", 0) == 0) {
            try {
                vel_max = std::stof(line.substr(4));
                write_param_f32(s, motor_id, PARAM_VEL_MAX, vel_max);
                std::cout << "✅ vel_max updated: " << vel_max << " rad/s\n";
            } catch (...) {
                std::cout << "❌ usage: vel 5.0\n";
            }
            continue;
        }
        if (line.rfind("acc ", 0) == 0) {
            try {
                acc_set = std::stof(line.substr(4));
                write_param_f32(s, motor_id, PARAM_ACC_SET, acc_set);
                std::cout << "✅ acc_set updated: " << acc_set << " rad/s^2\n";
            } catch (...) {
                std::cout << "❌ usage: acc 10.0\n";
            }
            continue;
        }

        // angle command
        try {
            float deg = std::stof(line);
            deg = std::max(-720.0f, std::min(720.0f, deg));
            float rad = deg * (float)M_PI / 180.0f;

            // PP command = write loc_ref (rad)
            pp_set_target_rad(s, motor_id, rad);
            std::cout << "➡ Target: " << deg << "°  (loc_ref=" << rad << " rad)\n";
        } catch (...) {
            std::cout << "❌ Invalid input\n";
        }
    }

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
