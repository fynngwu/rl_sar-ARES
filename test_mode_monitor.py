#!/usr/bin/env python3
"""Monitor /driver_mode topic to see actual mode values being published."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import time

MODE_NAMES = {0: "DISABLE", 1: "STAND", 2: "RL", 3: "DAMPING", 4: "GAIT", 5: "JUMP", 6: "CLIMB"}

class ModeMonitor(Node):
    def __init__(self):
        super().__init__('mode_monitor')
        self.prev_mode = None
        self.prev_time = time.time()
        self.transition_count = 0
        self.sub = self.create_subscription(UInt8, '/driver_mode', self.callback, 10)

    def callback(self, msg):
        now = time.time()
        mode = msg.data
        name = MODE_NAMES.get(mode, f"UNKNOWN({mode})")
        dt = (now - self.prev_time) * 1000

        if self.prev_mode is not None and mode != self.prev_mode:
            self.transition_count += 1
            prev_name = MODE_NAMES.get(self.prev_mode, f"?{self.prev_mode}")
            print(f"[{now:.3f}] {prev_name} → {name}  (+{dt:.1f}ms)  [transition #{self.transition_count}]")
            if dt < 50:
                print(f"  ⚠ FAST TRANSITION ({dt:.1f}ms)")
        else:
            if self.prev_mode is None:
                print(f"[{now:.3f}] initial: {name}")

        self.prev_mode = mode
        self.prev_time = now

def main():
    rclpy.init()
    node = ModeMonitor()
    print(f"Monitoring /driver_mode — Ctrl+C to stop\n")
    print(f"Mode codes: {MODE_NAMES}\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\nTotal transitions: {node.transition_count}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
