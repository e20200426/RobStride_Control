#!/usr/bin/env python3
"""
mit_control_node.py
-------------------
Minimal MIT-mode (Mode 0) motor control — no ROS topics.
Edit the constants below, then run:

    ros2 run rob_py mit_control_node
"""

import signal
import struct
import threading
import time

import rclpy

from rob_py.can_setup import setup_can_interface
from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType

# ── Configuration ─────────────────────────────────────────────────────────────
CAN_CHANNEL  = 'can0'
BITRATE      = 1_000_000   # bps
MOTOR_ID     = 1
MOTOR_MODEL  = 'rs-00'
KP           = 30.0        # stiffness  Nm/rad
KD           = 5.0         # damping    Nm·s/rad
LOOP_RATE_HZ = 100         # control loop frequency
# ──────────────────────────────────────────────────────────────────────────────


def set_mit_mode(bus: RobstrideBus, motor_name: str):
    param_id, _dtype, _ = ParameterType.MODE
    value_buffer = struct.pack('<bBH', 0, 0, 0)
    data = struct.pack('<HH', param_id, 0x00) + value_buffer
    bus.transmit(
        CommunicationType.WRITE_PARAMETER,
        bus.host_id,
        bus.motors[motor_name].id,
        data,
    )
    time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    if not setup_can_interface(CAN_CHANNEL, BITRATE):
        print(f"[ERROR] Could not bring up CAN interface '{CAN_CHANNEL}'. Exiting.")
        rclpy.shutdown()
        return

    motor_name  = f'motor_{MOTOR_ID}'
    motors      = {motor_name: Motor(id=MOTOR_ID, model=MOTOR_MODEL)}
    calibration = {motor_name: {'direction': 1, 'homing_offset': 0.0}}

    bus = RobstrideBus(CAN_CHANNEL, motors, calibration)
    bus.connect(handshake=True)
    bus.enable(motor_name)
    time.sleep(0.5)

    set_mit_mode(bus, motor_name)
    print(f"[INFO] MIT mode active — motor {MOTOR_ID} on {CAN_CHANNEL}")

    running = True
    target_position = 0.0   # start at zero

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    def _input_thread():
        nonlocal target_position, running
        print("Enter target position in rad (or 'q' to quit):")
        while running:
            try:
                line = input("> ").strip()
                if line.lower() == 'q':
                    running = False
                    break
                target_position = float(line)
                print(f"  → target set to {target_position:.4f} rad")
            except ValueError:
                print("  Invalid input. Enter a number in radians.")
            except EOFError:
                running = False
                break

    input_t = threading.Thread(target=_input_thread, daemon=True)
    input_t.start()

    dt = 1.0 / LOOP_RATE_HZ

    while running:
        try:
            bus.control_mit(
                motor_name,
                position=target_position,
                velocity=0.0,
                kp=KP,
                kd=KD,
                torque=0.0,
            )

            pos, vel, trq, temp = bus.read_operation_frame(motor_name)
            print(
                f"\r  pos={pos:+.4f} rad  vel={vel:+.4f} rad/s  "
                f"trq={trq:+.4f} Nm  temp={temp:.1f}°C   ",
                end='', flush=True,
            )
        except Exception as exc:
            print(f"\n[WARN] Loop error: {exc}")

        time.sleep(dt)

    print("\n[INFO] Stopping motor ...")
    try:
        bus.control_mit(motor_name, position=0.0, velocity=0.0,
                        kp=0.0, kd=KD, torque=0.0)
        time.sleep(0.1)
        bus.disable(motor_name)
        bus.disconnect()
    except Exception:
        pass

    rclpy.shutdown()
    print("[INFO] Done.")


if __name__ == '__main__':
    main()
