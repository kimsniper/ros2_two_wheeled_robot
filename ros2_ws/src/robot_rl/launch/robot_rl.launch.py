from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rl_pid_agent_node = Node(
        package='robot_rl',
        executable='rl_pid_agent',
        name='rl_pid_agent',
        output='screen'
    )

    return LaunchDescription([
        rl_pid_agent_node,
    ])