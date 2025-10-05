import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Package path setup (Replace 'my_robot_description' with your actual package name)
    my_robot_description_pkg = get_package_share_directory('lab5_my_robot_description')
    
    # 1. Declare Launch Argument for the model (URDF/XACRO file path)
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            my_robot_description_pkg,
            'urdf',
            'begginer_robot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )
    
    # 2. Process XACRO file to get the robot description
    robot_description = ParameterValue(
        Command([
            "xacro", 
            " ", 
            LaunchConfiguration("model")
        ]),
        value_type=str
    )
    
    # 3. Set Gazebo/Ignition Resource Path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            str(Path(LaunchConfiguration('model')).parent.resolve())
        ]
    )
    
    # 4. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # 5. Launch Gazebo/Ignition Simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 
                'launch', 
                'gz_sim.launch.py'
            )
        ),
        launch_arguments=[
            ('gz_args', '-v 4 -r empty.sdf')
        ]
    )
    
    # 6. Spawn the robot entity in the simulation
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-x', '1.0',
        ]
    )
    
    # 7. ROS 2 - Gazebo Bridge Node
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/laser_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ]
    )
    
    # 8. Return the list of actions
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
