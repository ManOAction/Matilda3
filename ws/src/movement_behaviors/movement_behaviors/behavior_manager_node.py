"""
Behavior Manager Node - Refactored

This version embeds behavior logic directly in the manager.
No need to run separate nodes for forward_back or box behaviors.

Usage:
    ros2 topic pub --once /behavior_command std_msgs/String "data: forward_back"
    ros2 topic pub --once /behavior_command std_msgs/String "data: box"
    ros2 topic pub --once /behavior_command std_msgs/String "data: idle"

Joystick mode still listens to external cmd_vel/joystick topic.
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class BehaviorManagerNode(Node):
    def __init__(self):
        super().__init__("behavior_manager")

        # ─────────────────────────────────────────────────────────────
        # Parameters
        # ─────────────────────────────────────────────────────────────
        self.declare_parameter("timeout_ms", 500)
        self.declare_parameter("loop_hz", 10.0)

        # Forward/back behavior parameters
        self.declare_parameter("fb_speed", 0.3)              # m/s
        self.declare_parameter("fb_forward_duration", 2.0)   # seconds
        self.declare_parameter("fb_back_duration", 2.0)      # seconds
        self.declare_parameter("fb_pause_duration", 0.5)     # seconds

        # Box behavior parameters
        self.declare_parameter("box_forward_speed", 0.3)     # m/s
        self.declare_parameter("box_turn_speed", 0.5)        # rad/s
        self.declare_parameter("box_side_duration", 2.0)     # seconds per side
        self.declare_parameter("box_turn_duration", 1.5)     # seconds per turn
        self.declare_parameter("box_loops", 1)               # number of boxes

        # Load parameters
        self.timeout_ms = self.get_parameter("timeout_ms").get_parameter_value().integer_value
        self.loop_hz = self.get_parameter("loop_hz").get_parameter_value().double_value

        # Forward/back params
        self.fb_speed = self.get_parameter("fb_speed").get_parameter_value().double_value
        self.fb_forward_duration = self.get_parameter("fb_forward_duration").get_parameter_value().double_value
        self.fb_back_duration = self.get_parameter("fb_back_duration").get_parameter_value().double_value
        self.fb_pause_duration = self.get_parameter("fb_pause_duration").get_parameter_value().double_value

        # Box params
        self.box_forward_speed = self.get_parameter("box_forward_speed").get_parameter_value().double_value
        self.box_turn_speed = self.get_parameter("box_turn_speed").get_parameter_value().double_value
        self.box_side_duration = self.get_parameter("box_side_duration").get_parameter_value().double_value
        self.box_turn_duration = self.get_parameter("box_turn_duration").get_parameter_value().double_value
        self.box_loops = self.get_parameter("box_loops").get_parameter_value().integer_value

        # ─────────────────────────────────────────────────────────────
        # State
        # ─────────────────────────────────────────────────────────────
        self.mode = "idle"  # "idle", "joystick", "forward_back", "box"

        # Joystick state (external input)
        self.last_joystick_cmd = Twist()
        self.last_joystick_time = time.time()

        # Forward/back state machine
        self.fb_state = "FORWARD"  # FORWARD, PAUSE1, BACK, PAUSE2
        self.fb_state_start_time = 0.0

        # Box state machine
        self.box_state = "FORWARD"  # FORWARD, TURN
        self.box_state_start_time = 0.0
        self.box_side_index = 0  # 0-3 within current loop
        self.box_loop_index = 0

        # ─────────────────────────────────────────────────────────────
        # Publishers & Subscribers
        # ─────────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Only joystick comes from external source now
        self.joystick_sub = self.create_subscription(
            Twist,
            "cmd_vel/joystick",
            self._joystick_callback,
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
            "BehaviorManagerNode started. Modes: idle, joystick, forward_back, box"
        )

    # ─────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────

    def _joystick_callback(self, msg: Twist):
        self.last_joystick_cmd = msg
        self.last_joystick_time = time.time()

    def _command_callback(self, msg: String):
        new_mode = msg.data.strip().lower()

        if new_mode in ("idle", "stop"):
            self.mode = "idle"
            self.get_logger().info("Mode set to: idle")

        elif new_mode == "joystick":
            self.mode = "joystick"
            self.get_logger().info("Mode set to: joystick")

        elif new_mode == "forward_back":
            # Reset state machine and start
            self.fb_state = "FORWARD"
            self.fb_state_start_time = time.time()
            self.mode = "forward_back"
            self.get_logger().info(
                f"Starting forward_back: {self.fb_forward_duration}s fwd, "
                f"{self.fb_pause_duration}s pause, {self.fb_back_duration}s back"
            )

        elif new_mode == "box":
            # Reset state machine and start
            self.box_state = "FORWARD"
            self.box_state_start_time = time.time()
            self.box_side_index = 0
            self.box_loop_index = 0
            self.mode = "box"
            self.get_logger().info(
                f"Starting box: {self.box_loops} loop(s), "
                f"{self.box_side_duration}s/side, {self.box_turn_duration}s/turn"
            )

        else:
            self.get_logger().warn(f"Unknown behavior mode: '{msg.data}'")

    # ─────────────────────────────────────────────────────────────────
    # Main Timer Loop
    # ─────────────────────────────────────────────────────────────────

    def _timer_callback(self):
        cmd = Twist()

        if self.mode == "idle":
            pass  # cmd stays zero

        elif self.mode == "joystick":
            cmd = self._handle_joystick()

        elif self.mode == "forward_back":
            cmd = self._handle_forward_back()

        elif self.mode == "box":
            cmd = self._handle_box()

        self.cmd_pub.publish(cmd)

    # ─────────────────────────────────────────────────────────────────
    # Joystick Handler (external input with timeout)
    # ─────────────────────────────────────────────────────────────────

    def _handle_joystick(self) -> Twist:
        now = time.time()
        timeout_sec = self.timeout_ms / 1000.0

        if now - self.last_joystick_time <= timeout_sec:
            return self.last_joystick_cmd
        else:
            self.get_logger().warn_once("Joystick command timed out, stopping.")
            return Twist()

    # ─────────────────────────────────────────────────────────────────
    # Forward/Back Behavior (internal state machine)
    # ─────────────────────────────────────────────────────────────────

    def _handle_forward_back(self) -> Twist:
        now = time.time()
        elapsed = now - self.fb_state_start_time
        cmd = Twist()

        if self.fb_state == "FORWARD":
            cmd.linear.x = self.fb_speed
            if elapsed >= self.fb_forward_duration:
                self.fb_state = "PAUSE1"
                self.fb_state_start_time = now
                self.get_logger().debug("forward_back: FORWARD -> PAUSE1")

        elif self.fb_state == "PAUSE1":
            # cmd stays zero
            if elapsed >= self.fb_pause_duration:
                self.fb_state = "BACK"
                self.fb_state_start_time = now
                self.get_logger().debug("forward_back: PAUSE1 -> BACK")

        elif self.fb_state == "BACK":
            cmd.linear.x = -self.fb_speed
            if elapsed >= self.fb_back_duration:
                self.fb_state = "PAUSE2"
                self.fb_state_start_time = now
                self.get_logger().debug("forward_back: BACK -> PAUSE2")

        elif self.fb_state == "PAUSE2":
            # cmd stays zero
            if elapsed >= self.fb_pause_duration:
                self.get_logger().info("forward_back: Complete. Returning to idle.")
                self.mode = "idle"

        return cmd

    # ─────────────────────────────────────────────────────────────────
    # Box Behavior (internal state machine)
    # ─────────────────────────────────────────────────────────────────

    def _handle_box(self) -> Twist:
        now = time.time()
        elapsed = now - self.box_state_start_time
        cmd = Twist()

        # Check if we've completed all loops
        if self.box_loop_index >= self.box_loops:
            self.get_logger().info("box: All loops complete. Returning to idle.")
            self.mode = "idle"
            return cmd

        if self.box_state == "FORWARD":
            cmd.linear.x = self.box_forward_speed
            if elapsed >= self.box_side_duration:
                self.box_state = "TURN"
                self.box_state_start_time = now
                self.get_logger().debug("box: FORWARD -> TURN")

        elif self.box_state == "TURN":
            cmd.angular.z = self.box_turn_speed
            if elapsed >= self.box_turn_duration:
                # Completed a corner
                self.box_side_index += 1
                if self.box_side_index >= 4:
                    self.box_side_index = 0
                    self.box_loop_index += 1
                    self.get_logger().info(
                        f"box: Completed loop {self.box_loop_index}/{self.box_loops}"
                    )
                self.box_state = "FORWARD"
                self.box_state_start_time = now
                self.get_logger().debug("box: TURN -> FORWARD")

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutting down
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()