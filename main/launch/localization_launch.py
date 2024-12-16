import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the package share directory
    package_share_directory = get_package_share_directory('your_package_name')

    # Path to the YAML configuration file
    params_file = os.path.join(package_share_directory, 'config', 'localization_params.yaml')

    # Define the map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file]
    )

    # Define the AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # Define the lifecycle manager node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[params_file]
    )

    # Return the launch description
    return LaunchDescription([
        map_server_node,
         amcl_node,
        lifecycle_manager
    ])