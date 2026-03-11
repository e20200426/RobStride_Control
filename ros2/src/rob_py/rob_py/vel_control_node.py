#!/usr/bin/env python3
"""
vel_control_node.py
-------------------
Minimal velocity-mode (Mode 2) motor control — no ROS topics.
Edit the constants below, then run:

    ros2 run rob_py vel_control_node
"""

import signal
import threading
import time

import rclpy

from rob_py.can_setup import setup_can_interface
from robstride_dynamics import RobstrideBus, Motor

# ── Configuration ─────────────────────────────────────────────────────────────
CAN_CHANNEL    = 'can0'
BITRATE        = 1_000_000   # bps
MOTOR_ID       = 3
MOTOR_MODEL    = 'rs-00'
TORQUE_LIMIT   = 2.0         # A  — max current in velocity mode
LOOP_RATE_HZ   = 100         # control loop frequency
# ──────────────────────────────────────────────────────────────────────────────


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

    bus.set_velocity_mode(motor_name, torque_limit=TORQUE_LIMIT)

    print(f"[INFO] Velocity mode active — motor {MOTOR_ID} on {CAN_CHANNEL}")

    running = True
    target_velocity = 0.0   # rad/s

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    def _input_thread():
        nonlocal target_velocity, running
        print("Enter target velocity in rad/s (or 'q' to quit):")
        while running:
            try:
                line = input("> ").strip()
                if line.lower() == 'q':
                    running = False
                    break
                target_velocity = float(line)
                print(f"  → target set to {target_velocity:.4f} rad/s")
            except ValueError:
                print("  Invalid input. Enter a number in rad/s.")
            except EOFError:
                running = False
                break

    input_t = threading.Thread(target=_input_thread, daemon=True)
    input_t.start()

    dt = 1.0 / LOOP_RATE_HZ

    while running:
        try:
            pos, vel, trq, temp = bus.control_velocity(motor_name, target_velocity)
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
        bus.control_velocity(motor_name, 0.0)
        time.sleep(0.1)
        bus.disable(motor_name)
        bus.disconnect()
    except Exception:
        pass

    rclpy.shutdown()
    print("[INFO] Done.")


if __name__ == '__main__':
    main()
