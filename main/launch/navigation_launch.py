import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # Define the package share directory
    package_share_directory = get_package_share_directory('your_package_name')

    # Path to the YAML configuration file
    params_file = os.path.join(package_share_directory, 'config', 'navigation_params.yaml')

    # Define the lifecycle manager node
    lifecycle_manager = LifecycleNode(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[params_file]
    )

    # Define the controller server node
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    # Define the planner server node
    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # Define the recoveries server node
    recoveries_server = LifecycleNode(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[params_file]
    )

    # Define the behavior tree navigator node
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    # Define the waypoint follower node
    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )

    # Return the launch description