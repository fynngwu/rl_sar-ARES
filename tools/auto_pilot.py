#!/usr/bin/env -S uv run python
"""
Auto Pilot — reads a YAML command sequence and publishes /auto_cmd via ROS2.

Usage:
    python3 tools/auto_pilot.py --config tools/auto_pilot_config.yaml

Prerequisites:
    1. 手柄 LB+Start → 进入 AUTO MODE
    2. 手柄 LB+A → STAND, LB+Y → RL (或 LB+B → GAIT)
    3. 运行此脚本
"""

import argparse
import time

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoPilot(Node):
    def __init__(self, config_path: str):
        super().__init__("auto_pilot")

        with open(config_path, "r") as f:
            self.cfg = yaml.safe_load(f)

        self.pub_auto_cmd = self.create_publisher(Twist, "/auto_cmd", 10)

        steps = self.cfg.get("steps", [])
        self.get_logger().info(f"Config: {config_path}, Steps: {len(steps)}")

    def publish_cmd(self, vx: float, vy: float, wz: float, height: float):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = height
        msg.angular.z = wz
        self.pub_auto_cmd.publish(msg)

    def publish_zero(self):
        self.publish_cmd(0.0, 0.0, 0.0, 0.0)

    def run(self):
        steps = self.cfg.get("steps", [])
        if not steps:
            self.get_logger().error("No steps in config")
            return

        hz = 50.0
        dt = 1.0 / hz

        self.get_logger().info("=== Auto Pilot START ===")
        self.get_logger().info("Waiting 2s for /auto_cmd subscriber to connect...")
        time.sleep(2.0)

        prev_vx, prev_vy, prev_wz, prev_h = 0.0, 0.0, 0.0, 0.0

        for i, step in enumerate(steps):
            vx = step.get("vx", 0.0)
            vy = step.get("vy", 0.0)
            wz = step.get("wz", 0.0)
            h = step.get("height", 0.0)
            duration = step["duration"]
            smooth = step.get("smooth", True)

            self.get_logger().info(
                f"Step {i+1}/{len(steps)}: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f} "
                f"h={h:.3f} dur={duration:.1f}s smooth={smooth}"
            )

            total_ticks = max(1, int(duration * hz))

            for t in range(total_ticks):
                if not rclpy.ok():
                    break

                if smooth and total_ticks > 1:
                    alpha = t / (total_ticks - 1)
                else:
                    alpha = 1.0 if t == total_ticks - 1 else 0.0

                cvx = prev_vx + alpha * (vx - prev_vx)
                cvy = prev_vy + alpha * (vy - prev_vy)
                cwz = prev_wz + alpha * (wz - prev_wz)
                ch = prev_h + alpha * (h - prev_h)

                self.publish_cmd(cvx, cvy, cwz, ch)
                time.sleep(dt)

            prev_vx, prev_vy, prev_wz, prev_h = vx, vy, wz, h

        # Ramp down to zero
        self.get_logger().info("Ramping down to zero...")
        total_ramp = max(1, int(1.0 * hz))
        for t in range(total_ramp):
            if not rclpy.ok():
                break
            alpha = t / (total_ramp - 1) if total_ramp > 1 else 1.0
            cvx = prev_vx * (1.0 - alpha)
            cvy = prev_vy * (1.0 - alpha)
            cwz = prev_wz * (1.0 - alpha)
            ch = prev_h * (1.0 - alpha)
            self.publish_cmd(cvx, cvy, cwz, ch)
            time.sleep(dt)

        self.publish_zero()
        self.get_logger().info("=== Auto Pilot DONE ===")


def main():
    parser = argparse.ArgumentParser(description="ARES Auto Pilot")
    parser.add_argument("--config", "-c", required=True, help="Path to YAML config")
    args = parser.parse_args()

    rclpy.init()
    node = AutoPilot(args.config)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
        node.publish_zero()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
