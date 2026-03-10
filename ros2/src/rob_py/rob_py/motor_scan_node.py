#!/usr/bin/env python3
"""
motor_scan_node.py
------------------
One-shot node that scans the CAN bus for RobStride motors and publishes
the result, then shuts down.

Published topic:
  /rob_py/scan_result   [std_msgs/String]   JSON string of {motor_id: uuid_hex}

Parameters:
  can_channel  (str, default 'can0')
  start_id     (int, default 1)
  end_id       (int, default 255)   scan IDs [start_id, end_id)
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robstride_dynamics import RobstrideBus
from rob_py.can_setup import setup_can_interface


class MotorScanNode(Node):
    """Scans CAN bus for motors and publishes discovered IDs as JSON."""

    def __init__(self):
        super().__init__('motor_scan_node')

        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('start_id', 1)
        self.declare_parameter('end_id', 255)
        self.declare_parameter('bitrate', 1000000)

        self._channel = self.get_parameter('can_channel').value
        self._start_id = self.get_parameter('start_id').value
        self._end_id = self.get_parameter('end_id').value
        bitrate = self.get_parameter('bitrate').value

        # --- Bring up CAN interface ---
        if not setup_can_interface(self._channel, bitrate):
            self.get_logger().error(
                f"Failed to bring up CAN interface '{self._channel}'. Shutting down.")
            raise RuntimeError(f"CAN interface '{self._channel}' could not be started.")

        self._pub = self.create_publisher(String, '/rob_py/scan_result', 10)

        # Run scan once after a short delay so the publisher is fully registered
        self.create_timer(0.5, self._run_scan)

    def _run_scan(self):
        self.destroy_timer(list(self._timers)[0])  # cancel the one-shot timer

        self.get_logger().info(
            f"Scanning '{self._channel}' IDs {self._start_id}–{self._end_id - 1} ...")
        try:
            found = RobstrideBus.scan_channel(
                self._channel,
                start_id=self._start_id,
                end_id=self._end_id,
            )
        except Exception as exc:
            self.get_logger().error(f"Scan failed: {exc}")
            rclpy.shutdown()
            return

        result = {}
        if found:
            for motor_id, (_id, uuid) in found.items():
                result[str(motor_id)] = uuid.hex()
            self.get_logger().info(f"Found motors: {list(result.keys())}")
        else:
            self.get_logger().warn("No motors found on the bus.")

        msg = String(data=json.dumps(result))
        self._pub.publish(msg)

        self.get_logger().info("Scan complete. Shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotorScanNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
