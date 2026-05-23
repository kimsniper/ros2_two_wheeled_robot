from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_sim = get_package_share_directory('robot_sim')
    pkg_control = get_package_share_directory('robot_control')
    pkg_estimation = get_package_share_directory('robot_estimation')

    use_sim_time = {'use_sim_time': True}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'sim.launch.py')
        )
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg_sim, 'config', 'controllers.yaml'),
            use_sim_time
        ],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_controller'],
        output='screen'
    )

    estimator = Node(
        package='robot_estimation',
        executable='robot_estimator',
        parameters=[use_sim_time],
        output='screen'
    )

    balance_controller = Node(
        package='robot_control',
        executable='balance_node',
        parameters=[
            os.path.join(pkg_control, 'config', 'balance_params.yaml'),
            use_sim_time
        ],
        output='screen'
    )

    odometry = Node(
        package='robot_odometry',
        executable='odometry_node',
        output='screen',
        parameters=[use_sim_time]
    )

    return LaunchDescription([
        sim,

        controller_manager,
        load_joint_state_broadcaster,
        load_wheel_controller,
        odometry,

        estimator,
        balance_controller
    ])