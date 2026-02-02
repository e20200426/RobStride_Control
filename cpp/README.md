# RobStride Control C++

C++ implementation of the RobStride motor control library, providing high-performance real-time control capabilities.

## Features

- ⚡ **High Performance**: 200Hz control frequency, 1ms latency  
- 🔧 **Direct Control**: Low-level implementation based on SocketCAN  
- 🛡️ **Type Safety**: Strong type checking, memory safety  
- 📦 **Easy Integration**: Standard CMake build system  
- 🎯 **Professional-Grade**: Suitable for industrial-level applications  

## System Requirements

- Linux system (Ubuntu 18.04+, Debian 10+)  
- GCC 7+ or Clang 8+  
- CMake 3.12+  
- SocketCAN support  

## Install Dependencies

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install build-essential cmake can-utils

# Use project script to install dependencies
make install-deps
```

## Build

### Using Makefile

```bash
# Build
make

# Debug build
make debug

# Release build
make release

# Install to system
sudo make install
```

### Using CMake

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)

# Optional: install
sudo make install
```

## Usage

### Basic Usage

```cpp
#include "can_interface.h"
#include "protocol.h"

int main(int argc, char* argv[]) {
    int motor_id = 11;

    // 初始化 CAN 接口
    CanInterface can;
    if (!can.init("can0")) {
        std::cerr << "Failed to initialize CAN" << std::endl;
        return 1;
    }

    // 设置电机参数
    enable_motor(can.socket(), motor_id);
    set_mode_raw(can.socket(), motor_id, ControlMode::MIT_MODE);

    // 设置限制
    write_limit(can.socket(), motor_id, ParamID::VELOCITY_LIMIT, 20.0);
    write_limit(can.socket(), motor_id, ParamID::TORQUE_LIMIT, 20.0);

    // 位置控制
    double target_pos = M_PI / 2; // 90度
    write_operation_frame(can.socket(), motor_id, target_pos, 30.0, 0.5);

    return 0;
}
```

### Compile and Run

```bash
# Build
make

# Run (requires sudo privileges)
sudo ./build/robstride-mit-position 11
```

## API Reference

### CanInterface Class

```cpp
class CanInterface {
public:
    CanInterface();
    ~CanInterface();

    bool init(const std::string& interface = "can0");
    void close();
    bool send_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
    bool read_frame(can_frame* frame, int timeout_ms = 100);
    bool is_ready() const;
};
```

### Protocol Functions

```cpp
// Motor control
bool enable_motor(int socket, int motor_id);
bool set_mode_raw(int socket, int motor_id, int8_t mode);
bool write_operation_frame(int socket, int motor_id,
                          double pos, double kp, double kd);

// Parameter settings
bool write_limit(int socket, int motor_id, uint16_t param_id, float limit);

// Status reading
bool read_operation_frame(int socket);
```

### Protocol Constants

```cpp
namespace CommType {
    constexpr uint32_t ENABLE = 3;
    constexpr uint32_t OPERATION_CONTROL = 1;
    constexpr uint32_t WRITE_PARAMETER = 18;
}

namespace ControlMode {
    constexpr int8_t MIT_MODE = 0;
    constexpr int8_t POSITION_MODE = 1;
    constexpr int8_t SPEED_MODE = 2;
}
```

## Control Modes

### MIT MODE (Mode 0)

```cpp
// Switch to MIT mode
set_mode_raw(socket, motor_id, ControlMode::MIT_MODE);

// Send position command
double position = M_PI / 2;    // 90度
double kp = 30.0;              // 位置增益
double kd = 0.5;               // 阻尼增益

while (running) {
    write_operation_frame(socket, motor_id, position, kp, kd);
    read_operation_frame(socket);  // 清空接收缓冲区

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
}
```

### Parameter Configuration

```cpp
// Set control parameters
write_limit(socket, motor_id, ParamID::VELOCITY_LIMIT, 20.0);
write_limit(socket, motor_id, ParamID::TORQUE_LIMIT, 20.0);
write_limit(socket, motor_id, ParamID::POSITION_KP, 30.0);
write_limit(socket, motor_id, ParamID::VELOCITY_KP, 0.5);
```

## Interactive Control

After the program starts, an interactive interface is provided:

```
🎯 MIT Position Control Console (ID: 11)
========================================
👉 Enter a number (degrees) and press Enter to change position
👉 'kp <value>' (e.g., kp 100) to adjust stiffness
👉 'kd <value>' (e.g., kd 2.0) to adjust damping (anti-shake)
👉 '0' or 'home' to return to zero position
👉 'q' to quit
⚠️  Current Kp=100 | Kd=2.0
----------------------------------------
[0.0°] >> 90
 -> Target set: 90.0°
```

## Performance Optimization

### Compile-Time Optimization

```bash
# Release build (optimized)
CXXFLAGS="-O3 -DNDEBUG" make

# Enable LTO
cmake -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=TRUE ..
```

### Runtime Optimization

```cpp
// High-priority scheduling
struct sched_param param;
param.sched_priority = 99;
sched_setscheduler(0, SCHED_FIFO, &param);

// CPU affinity
cpu_set_t cpuset;
CPU_ZERO(&cpuset);
CPU_SET(2, &cpuset);  // Bind CPU 2
sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
```

### Memory Optimization

```cpp
// se object pool
std::vector<can_frame> frame_pool;
frame_pool.reserve(1000);  // Pre-allocate

// Avoid dynamic allocation
uint8_t data[8];  // Stack allocation
```

## Debugging

### CAN Monitoring

```bash
# Monitor CAN traffic
sudo candump can0

# Filter specific ID
sudo candump can0,0C0:7FF
```

### Debug Output

```cpp
// Enable debug at compile time
#ifdef DEBUG
    std::cout << "Debug: " << message << std::endl;
#endif

// Runtime debugging
const bool debug = true;
if (debug) {
    printf("Pos: %.3f, Kp: %.1f, Kd: %.1f\n", pos, kp, kd);
}
```

## Testing

### Unit Tests

```bash
# Install Google Test
sudo apt-get install libgtest-dev

# Build tests
cmake -DBUILD_TESTING=ON ..
make

# Run tests
./tests/robstride_test
```

### Integration Tests

```bash
# Motor connection test
make test

# Manual test
sudo ./build/robstride-mit-position --test

# Run example program
g++ -std=c++17 -I../include examples/basic_control.cpp -o basic_control
sudo ./basic_control 11
```

## Troubleshooting

### Compilation Errors

```bash
# Check C++ standard
g++ --version  # Requires GCC 7+

# Check CMake
cmake --version  # Requires 3.12+

# Clean and rebuild
make clean
make
```

### Runtime Errors

```bash
# Check CAN interface
ip link show can0

# Check permissions
groups  # Should include dialout

# Check device
ls -l /sys/class/net/can0
```

## Deployment

### System Service

```bash
# Create service file
sudo cp scripts/robstride.service /etc/systemd/system/
sudo systemctl enable robstride
sudo systemctl start robstride
```

### Docker

```dockerfile
FROM ubuntu:20.04
RUN apt-get update && apt-get install -y build-essential cmake
COPY . /app
WORKDIR /app
RUN make
CMD ["./build/robstride-mit-position"]
```

## License

MIT License - 详见 [LICENSE](../LICENSE) 文件

## Contribution

欢迎提交 Issue 和 Pull Request！

## Support

- 📖 [完整文档](../docs/)
- 🐛 [问题反馈](https://github.com/tianrking/robstride-control/issues)