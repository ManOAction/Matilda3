import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        self.declare_parameter("port", "/dev/matilda-uno")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_pwm", 255)
        self.declare_parameter("timeout_ms", 500)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        self.last_cmd_time = time.time()

        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info(f"Motor node started on {port}")

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Simple diff-drive mixing
        left = v - w
        right = v + w

        max_pwm = self.get_parameter("max_pwm").value
        scale = max(abs(left), abs(right), 1.0)

        left_pwm = int(max_pwm * left / scale)
        right_pwm = int(max_pwm * right / scale)

        left_pwm = max(-max_pwm, min(max_pwm, left_pwm))
        right_pwm = max(-max_pwm, min(max_pwm, right_pwm))

        cmd = f"M,{left_pwm},{right_pwm}\n"
        self.ser.write(cmd.encode())
        self.last_cmd_time = time.time()

    def watchdog(self):
        timeout = self.get_parameter("timeout_ms").value / 1000.0
        if time.time() - self.last_cmd_time > timeout:
            self.ser.write(b"S\n")  # stop command
            self.last_cmd_time = time.time()


def main():
    rclpy.init()
    node = MotorNode()
    try:
        rclpy.spin(node)
    finally:
        node.ser.write(b"S\n")
        node.destroy_node()
        rclpy.shutdown()
