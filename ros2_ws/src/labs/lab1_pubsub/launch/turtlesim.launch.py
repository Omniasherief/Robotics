from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen'
        ),
        Node(
            package='lab2_turtlesim_pub_pkg',
            executable='turtle_mover',
            name='circle_turtle',
            output='screen'
        ),
    ])
