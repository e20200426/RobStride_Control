# rob_py

A minimal ROS2 Python package (`ament_python`) for controlling **RobStride** brushless motors over SocketCAN.

## Package layout

```
rob_py/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/rob_py
├── rob_py/
│   ├── can_setup.py          # CAN interface bring-up helper
│   ├── mit_control_node.py   # MIT-mode position control (interactive)
│   └── motor_scan_node.py    # CAN bus scanner
└── robstride_dynamics/       # bundled SDK (bus, protocol, tables)
```

---

## Prerequisites

| Requirement | Notes |
|---|---|
| ROS2 (Humble / Iron / Jazzy) | `source /opt/ros/<distro>/setup.bash` |
| `python-can` | `pip install python-can` |
| `numpy`, `tqdm` | `pip install numpy tqdm` |
| `can-utils` | `sudo apt install can-utils` |
| SocketCAN hardware | USB-CAN adapter or on-board CAN |
| Passwordless `sudo` for `ip` | See [CAN setup](#can-interface-setup) |

---

## Installation

### Clone the repository

```bash
git clone https://github.com/e20200426/robstride_ros2.git
cd robstride_ros2
```

## Build

```bash
# Source your ROS2 distro first
source /opt/ros/<distro>/setup.bash   # e.g. humble, iron, jazzy

cd robstride_ros2
colcon build --packages-select rob_py
source install/setup.bash
```

---

## CAN interface setup

Every node **automatically** runs:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

before opening the bus. For this to work without a password prompt, add one line to `/etc/sudoers` (run `sudo visudo`):

```
<your_username>  ALL=(ALL) NOPASSWD: /sbin/ip
```

If the interface is already `UP` the commands are skipped.

---

## Nodes

### `mit_control_node` — interactive MIT-mode position control

Connects to a single motor, switches it to **MIT mode (Mode 0)**, holds it at **0 rad**, then lets you type new target positions at any time.

#### Configuration

Edit the constants at the top of [rob_py/mit_control_node.py](rob_py/mit_control_node.py):

```python
CAN_CHANNEL  = 'can0'       # SocketCAN interface name
BITRATE      = 1_000_000    # CAN bitrate (bps)
MOTOR_ID     = 1            # motor ID on the bus
MOTOR_MODEL  = 'rs-00'      # RobStride model string
KP           = 30.0         # MIT stiffness (Nm/rad)
KD           = 5.0          # MIT damping   (Nm·s/rad)
LOOP_RATE_HZ = 100          # control-loop frequency (Hz)
```

#### Run

```bash
ros2 run rob_py mit_control_node
```

#### Usage

```
[INFO] MIT mode active — motor 1 on can0
Enter target position in rad (or 'q' to quit):
>
  pos=+0.0003 rad  vel=+0.0001 rad/s  trq=+0.0000 Nm  temp=25.0°C
> 1.57
  → target set to 1.5700 rad
> -0.5
  → target set to -0.5000 rad
> q
[INFO] Stopping motor ...
[INFO] Done.
```

- Type a **number** (radians) and press Enter to move the motor.
- Type **`q`** or press **Ctrl+C** to safely stop and disable the motor.

---

### `motor_scan_node` — CAN bus scanner

Scans the bus for all responding motor IDs and publishes the result as JSON, then exits.

#### Run

```bash
ros2 run rob_py motor_scan_node
```

Pass optional arguments:

```bash
ros2 run rob_py motor_scan_node --ros-args \
  -p can_channel:=can0 \
  -p start_id:=1 \
  -p end_id:=255 \
  -p bitrate:=1000000
```

#### Published topic

| Topic | Type | Content |
|---|---|---|
| `/rob_py/scan_result` | `std_msgs/String` | JSON object `{"<id>": "<uuid_hex>", ...}` |

Subscribe in another terminal:

```bash
ros2 topic echo /rob_py/scan_result
```

Example output:

```json
{"1": "a1b2c3d4e5f60102", "3": "deadbeef01020304"}
```

---

## can_setup helper

`rob_py.can_setup` can also be used directly in your own scripts:

```python
from rob_py.can_setup import setup_can_interface, teardown_can_interface

setup_can_interface('can0', bitrate=1_000_000)   # brings interface UP
# ... your code ...
teardown_can_interface('can0')                   # brings interface DOWN
```

---

## Supported motor models

| Model string | Notes |
|---|---|
| `rs-00` | RobStride 00 |
| `rs-01` | RobStride 01 |
| `rs-02` | RobStride 02 |
| `rs-03` | RobStride 03 |
| `rs-06` | RobStride 06 |

Model strings must match the keys defined in `robstride_dynamics/table.py`.
