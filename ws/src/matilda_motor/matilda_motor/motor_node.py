"""
Motor Node - Serial bridge between ROS2 and Arduino motor controller

This node:
  - Subscribes to /cmd_vel
  - Converts Twist messages to PWM values for differential drive
  - Sends motor commands to Arduino over serial
  - Reads Arduino responses to prevent serial buffer overflow
  - Implements a watchdog to stop motors if commands stop arriving
"""

import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        # Declare parameters
        self.declare_parameter("port", "/dev/matilda-uno")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_pwm", 255)
        self.declare_parameter("timeout_ms", 500)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        # Open serial connection
        # timeout=0.1 means non-blocking reads will wait up to 100ms
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        self.last_cmd_time = time.time()

        # Subscribe to velocity commands
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        # Timer for watchdog AND serial reading (runs at 10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Wait for Arduino to be ready after serial connection
        # The Arduino resets when serial connects, needs time to boot
        time.sleep(2.0)
        self._drain_serial()  # Clear any startup messages

        self.get_logger().info(f"Motor node started on {port}")

    def _drain_serial(self):
        """
        Read and log any available data from Arduino.

        This is CRITICAL - the Arduino sends responses like "OK" after
        every command. If we never read them, the Arduino's 64-byte TX
        buffer fills up and Serial.println() blocks, freezing the Arduino.

        Think of it like a pipe - if you don't drain the output,
        the source eventually backs up and stops.
        """
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    # Log Arduino responses at debug level
                    # Change to .info() if you want to see them during normal operation
                    self.get_logger().debug(f"Arduino: {line}")
            except Exception as e:
                self.get_logger().warn(f"Error reading serial: {e}")
                break

    def cmd_vel_cb(self, msg: Twist):
        """Convert Twist to differential drive PWM and send to Arduino."""
        v = msg.linear.x   # Forward/backward velocity
        w = msg.angular.z  # Rotational velocity

        # Differential drive mixing:
        # - Positive w (counter-clockwise) means left wheel slower, right faster
        # - The signs here depend on your motor wiring
        left = v - w
        right = v + w

        # Scale to PWM range while preserving the ratio
        max_pwm = self.get_parameter("max_pwm").value
        scale = max(abs(left), abs(right), 1.0)

        left_pwm = int(max_pwm * left / scale)
        right_pwm = int(max_pwm * right / scale)

        # Clamp to valid range (redundant but safe)
        left_pwm = max(-max_pwm, min(max_pwm, left_pwm))
        right_pwm = max(-max_pwm, min(max_pwm, right_pwm))

        # Send command to Arduino
        cmd = f"M,{left_pwm},{right_pwm}\n"
        self.ser.write(cmd.encode())
        self.last_cmd_time = time.time()

    def timer_callback(self):
        """
        Periodic callback that:
        1. Drains Arduino serial responses (prevents buffer overflow)
        2. Checks command timeout (watchdog)
        """
        # IMPORTANT: Always drain serial to prevent Arduino lockup
        self._drain_serial()

        # Watchdog: stop motors if no commands received recently
        timeout = self.get_parameter("timeout_ms").value / 1000.0
        if time.time() - self.last_cmd_time > timeout:
            self.ser.write(b"S\n")  # Stop command
            self.last_cmd_time = time.time()


def main():
    rclpy.init()
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before exiting
        node.ser.write(b"S\n")
        node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()