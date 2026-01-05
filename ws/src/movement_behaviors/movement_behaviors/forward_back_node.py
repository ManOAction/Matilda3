import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ForwardBackNode(Node):
    def __init__(self):
        super().__init__("forward_back_node")

        # Parameters
        self.declare_parameter("speed", 1)            # m/s
        self.declare_parameter("forward_duration", 2.0) # seconds
        self.declare_parameter("back_duration", 2.0)    # seconds
        self.declare_parameter("pause_duration", 0.5)   # seconds between segments
        self.declare_parameter("loop_hz", 10.0)

        self.speed = self.get_parameter("speed").get_parameter_value().double_value
        self.forward_duration = self.get_parameter("forward_duration").get_parameter_value().double_value
        self.back_duration = self.get_parameter("back_duration").get_parameter_value().double_value
        self.pause_duration = self.get_parameter("pause_duration").get_parameter_value().double_value
        self.loop_hz = self.get_parameter("loop_hz").get_parameter_value().double_value

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel/forward_back", 10)

        self.state = "FORWARD"
        self.state_start_time = time.time()

        period = 1.0 / self.loop_hz
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            "ForwardBackNode started: "
            f"{self.forward_duration}s forward, pause {self.pause_duration}s, "
            f"{self.back_duration}s backward."
        )

    def _timer_callback(self):
        now = time.time()
        elapsed = now - self.state_start_time
        cmd = Twist()

        if self.state == "FORWARD":
            cmd.linear.x = self.speed
            if elapsed >= self.forward_duration:
                self.state = "PAUSE1"
                self.state_start_time = now
        elif self.state == "PAUSE1":
            # stop
            if elapsed >= self.pause_duration:
                self.state = "BACK"
                self.state_start_time = now
        elif self.state == "BACK":
            cmd.linear.x = -self.speed
            if elapsed >= self.back_duration:
                self.state = "PAUSE2"
                self.state_start_time = now
        elif self.state == "PAUSE2":
            # stop
            if elapsed >= self.pause_duration:
                self.get_logger().info("Completed forward/back pattern, shutting down node.")
                self.cmd_pub.publish(cmd)  # zero
                rclpy.shutdown()
                return
        else:
            self.state = "FORWARD"
            self.state_start_time = now

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardBackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
