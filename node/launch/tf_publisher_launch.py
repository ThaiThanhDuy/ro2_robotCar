import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='node',  # Replace with your package name
            executable='joint_state_publisher',  # Replace with your node executable name
            name='joint_state_publisher',
            output='screen',
            parameters=[],
            remappings=[],
        ),
    ])