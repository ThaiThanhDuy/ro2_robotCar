from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='node',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[],
            remappings=[],
        ),
        # Add other nodes related to your robot here
    ])