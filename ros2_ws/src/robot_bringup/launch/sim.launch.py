from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():

    pkg_sim = get_package_share_directory('robot_sim')
    pkg_desc = get_package_share_directory('robot_description')

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_desc, "urdf", "two_wheel_robot.urdf.xacro"),
        description="Robot URDF"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(pkg_desc).parent.resolve())
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items()
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "two_wheel_robot"
        ]
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ("/imu", "/imu/data"),
        ],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_pub,
        gazebo,
        spawn_entity,
        bridge
    ])