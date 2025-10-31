import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    my_robot_description = get_package_share_directory("lab5_my_robot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        my_robot_description, "urdf", "begginer_robot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )
    

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(my_robot_description).parent.resolve())
            ]
        )
 
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [ " -r", " empty.sdf"]
                    )
                ]
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "my_robot",
                   "-x", "1.0" ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
            "/laser_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ]
    )
    # joint_state_publisher_node = Node(
    # package="joint_state_publisher",
    # executable="joint_state_publisher"
    # )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        #joint_state_publisher_node,
        gz_ros2_bridge,
    ])