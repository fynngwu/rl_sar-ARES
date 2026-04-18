#!/usr/bin/env python3
"""Monitor /motor_feedback torque and warn if any joint exceeds threshold."""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

THRESHOLD = 10.0
TOPIC = "/motor_feedback"

# FL RL FR RR ordering, matching driver_node kJointNames
JOINT_NAMES = [
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
]

LEG_MAP = {
    "LF": "左前", "LR": "左后", "RF": "右前", "RR": "右后",
}


def leg_of(joint_name: str) -> str:
    prefix = joint_name.split("_")[0]
    return LEG_MAP.get(prefix, prefix)


class TorqueMonitor(Node):
    def __init__(self, threshold: float):
        super().__init__("torque_monitor")
        self.threshold = threshold
        self.sub = self.create_subscription(JointState, TOPIC, self.callback, 10)

    def callback(self, msg: JointState):
        names = msg.name if msg.name else JOINT_NAMES
        for i, effort in enumerate(msg.effort):
            if abs(effort) > self.threshold:
                name = names[i] if i < len(names) else f"joint_{i}"
                print(
                    f"\033[91mWARNING\033[0m {name} ({leg_of(name)}) "
                    f"torque={effort:+.2f} Nm exceeds ±{self.threshold}"
                )


def main():
    threshold = float(sys.argv[1]) if len(sys.argv) > 1 else THRESHOLD
    rclpy.init()
    node = TorqueMonitor(threshold)
    print(f"Monitoring {TOPIC} — threshold ±{threshold} Nm. Ctrl-C to stop.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
