#!/usr/bin/env python3
import argparse
import json
import socket
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


DEFAULT_SERVER = "100.108.115.42"
DEFAULT_PORT = 8888
DEFAULT_SCALE = 0.5


class VirtualGamepadClient(Node):
    def __init__(self, server_ip, port, gamepad_scale):
        super().__init__("virtual_gamepad_client")
        self.server_ip = server_ip
        self.port = port
        self.gamepad_scale = gamepad_scale
        self.pub = self.create_publisher(Twist, "/xbox_vel", 10)
        self.sock = None
        self.buf = b""
        self.get_logger().info(f"Connecting to {server_ip}:{port}...")
        self._connect()

    def _connect(self):
        while rclpy.ok():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect((self.server_ip, self.port))
                s.settimeout(None)
                self.sock = s
                self.buf = b""
                self.get_logger().info(f"Connected to {self.server_ip}:{self.port}")
                return
            except (ConnectionRefusedError, OSError):
                s.close()
                time.sleep(1.0)

    def _reconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self.get_logger().warn("Disconnected, reconnecting...")
        self._connect()

    def spin_once_with_io(self):
        if self.sock is None:
            self._publish_zero()
            return
        try:
            self.sock.setblocking(False)
            try:
                data = self.sock.recv(4096)
                if not data:
                    self._reconnect()
                    return
                self.buf += data
                while b"\n" in self.buf:
                    line, self.buf = self.buf.split(b"\n", 1)
                    if line.strip():
                        self._handle_line(line.decode("utf-8"))
            except BlockingIOError:
                pass
            finally:
                self.sock.setblocking(True)
        except OSError:
            self._reconnect()

    def _handle_line(self, line):
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return
        twist = Twist()
        twist.linear.x = msg.get("ly", 0.0) * self.gamepad_scale
        twist.linear.y = msg.get("lx", 0.0) * self.gamepad_scale
        twist.linear.z = msg.get("height", 0.0)
        twist.angular.z = msg.get("rx", 0.0) * self.gamepad_scale
        self.pub.publish(twist)

    def _publish_zero(self):
        twist = Twist()
        self.pub.publish(twist)


def main():
    parser = argparse.ArgumentParser(description="ARES Virtual Gamepad Client")
    parser.add_argument("--server", default=DEFAULT_SERVER, help="Server IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Server port")
    parser.add_argument("--scale", type=float, default=DEFAULT_SCALE, help="gamepad_scale")
    args = parser.parse_args()

    rclpy.init()
    node = VirtualGamepadClient(args.server, args.port, args.scale)

    while rclpy.ok():
        node.spin_once_with_io()
        time.sleep(0.02)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
