import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class BoxDriveNode(Node):
    def __init__(self):
        super().__init__("box_drive_node")

        # Parameters
        self.declare_parameter("forward_speed", 1)    # m/s
        self.declare_parameter("turn_speed", 1)       # rad/s
        self.declare_parameter("side_duration", 2.0)    # seconds forward per side
        self.declare_parameter("turn_duration", 1.5)    # seconds per 90° turn (approx)
        self.declare_parameter("loops", 1)              # number of boxes to drive
        self.declare_parameter("loop_hz", 10.0)

        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.turn_speed = self.get_parameter("turn_speed").get_parameter_value().double_value
        self.side_duration = self.get_parameter("side_duration").get_parameter_value().double_value
        self.turn_duration = self.get_parameter("turn_duration").get_parameter_value().double_value
        self.loops = self.get_parameter("loops").get_parameter_value().integer_value
        self.loop_hz = self.get_parameter("loop_hz").get_parameter_value().double_value

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel/box", 10)

        self.state = "FORWARD"
        self.side_index = 0  # 0-3 inside current loop
        self.loop_index = 0
        self.state_start_time = time.time()

        period = 1.0 / self.loop_hz
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f"BoxDriveNode started: {self.loops} loop(s), "
            f"{self.side_duration}s per side, {self.turn_duration}s per turn."
        )

    def _timer_callback(self):
        now = time.time()
        elapsed = now - self.state_start_time
        cmd = Twist()

        if self.loop_index >= self.loops:
            # Finished all loops -> publish stop and shut down
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Completed box pattern, shutting down node.")
            rclpy.shutdown()
            return

        if self.state == "FORWARD":
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            if elapsed >= self.side_duration:
                # Transition to turn
                self.state = "TURN"
                self.state_start_time = now
        elif self.state == "TURN":
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
            if elapsed >= self.turn_duration:
                # Finished a corner
                self.side_index += 1
                if self.side_index >= 4:
                    self.side_index = 0
                    self.loop_index += 1
                    self.get_logger().info(f"Completed loop {self.loop_index}/{self.loops}")
                self.state = "FORWARD"
                self.state_start_time = now
        else:
            # Shouldn't happen; stop
            self.state = "FORWARD"
            self.state_start_time = now

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BoxDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure we send a stop before quitting
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
