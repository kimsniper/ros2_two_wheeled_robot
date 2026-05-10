from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rl_pid_agent_node = Node(
        package='robot_rl',
        executable='rl_pid_agent',
        name='rl_pid_agent',
        output='screen'
    )

    rl_episode_manager_node = Node(
        package='robot_rl',
        executable='rl_episode_manager',
        name='rl_episode_manager',
        output='screen'
    )

    return LaunchDescription([
        rl_pid_agent_node,
        rl_episode_manager_node
    ])