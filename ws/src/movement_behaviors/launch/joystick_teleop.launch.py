from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"deadzone": 0.05, "autorepeat_rate": 20.0}],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",  # may be 'teleop_twist_joy_node' on some distros
                name="teleop_twist_joy",
                output="screen",
                parameters=[
                    # Adjust axes/buttons mapping as needed; see teleop_twist_joy docs
                    {"enable_button": 0, "axis_linear.x": 1, "axis_angular.z": 0}
                ],
                remappings=[("cmd_vel", "cmd_vel/joystick")],
            ),
        ]
    )
