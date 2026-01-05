import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class BehaviorManagerNode(Node):
    def __init__(self):
        super().__init__("behavior_manager")

        # Parameters
        self.declare_parameter("timeout_ms", 500)
        self.declare_parameter("loop_hz", 10.0)

        self.timeout_ms = self.get_parameter("timeout_ms").get_parameter_value().integer_value
        self.loop_hz = self.get_parameter("loop_hz").get_parameter_value().double_value

        # Current mode: "idle", "joystick", "box", "forward_back"
        self.mode = "idle"

        # Last known commands and timestamps
        self.last_joystick_cmd = Twist()
        self.last_box_cmd = Twist()
        self.last_forward_back_cmd = Twist()

        now = time.time()
        self.last_joystick_time = now
        self.last_box_time = now
        self.last_forward_back_time = now

        # Publisher to REAL cmd_vel that motor_node listens to
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers to the three command sources
        self.joystick_sub = self.create_subscription(
            Twist,
            "cmd_vel/joystick",
            self._joystick_callback,
            10,
        )
        self.box_sub = self.create_subscription(
            Twist,
            "cmd_vel/box",
            self._box_callback,
            10,
        )
        self.forward_back_sub = self.create_subscription(
            Twist,
            "cmd_vel/forward_back",
            self._forward_back_callback,
            10,
        )

        # Mode command subscriber
        self.command_sub = self.create_subscription(
            String,
            "behavior_command",
            self._command_callback,
            10,
        )

        # Main loop timer
        period = 1.0 / self.loop_hz
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            "BehaviorManagerNode started. Modes: idle, joystick, box, forward_back."
        )

    def _joystick_callback(self, msg: Twist):
        self.last_joystick_cmd = msg
        self.last_joystick_time = time.time()

    def _box_callback(self, msg: Twist):
        self.last_box_cmd = msg
        self.last_box_time = time.time()

    def _forward_back_callback(self, msg: Twist):
        self.last_forward_back_cmd = msg
        self.last_forward_back_time = time.time()

    def _command_callback(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode in ("idle", "stop"):
            self.mode = "idle"
        elif new_mode in ("joystick", "box", "forward_back"):
            self.mode = new_mode
        else:
            self.get_logger().warn(f"Unknown behavior mode: '{msg.data}'")
            return

        self.get_logger().info(f"Behavior mode set to: {self.mode}")

    def _timer_callback(self):
        now = time.time()
        timeout_sec = self.timeout_ms / 1000.0

        cmd = Twist()

        if self.mode == "joystick":
            if now - self.last_joystick_time <= timeout_sec:
                cmd = self.last_joystick_cmd
            else:
                self.get_logger().warn_once("Joystick command timed out, stopping.")
        elif self.mode == "box":
            if now - self.last_box_time <= timeout_sec:
                cmd = self.last_box_cmd
            else:
                self.get_logger().info_once("Box behavior finished or timed out, stopping.")
                self.mode = "idle"
        elif self.mode == "forward_back":
            if now - self.last_forward_back_time <= timeout_sec:
                cmd = self.last_forward_back_cmd
            else:
                self.get_logger().info_once("Forward/back behavior finished or timed out, stopping.")
                self.mode = "idle"
        else:
            # idle -> keep cmd zero
            pass

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
