from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='maze_simulation',
            executable='maze_sim_2',
            name='turtle_control'
        )
    ])