from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pubsub',
            executable='pubc',
            name='talker',
            output='screen'
        ),
        Node(
            package='lab1_pubsub',
            executable='subc',
            name='listener',
            output='screen'
        ),
    ])
