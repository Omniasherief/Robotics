from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="task1_rpm_speed",
            executable="rpm_publisher",
            name="rpm_publisher",
            output="screen"
        ),
        Node(
            package="task1_rpm_speed",
            executable="speed_calculator",
            name="speed_calculator",
            output="screen",
            parameters=[{"wheel_radius": 0.2}]  # example: 20 cm
        ),
    ])
