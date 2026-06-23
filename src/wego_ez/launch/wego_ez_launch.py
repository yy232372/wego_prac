from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    limo_application_share_dir = get_package_share_directory('limo_ros2_application')
    rviz_config_path = os.path.join(limo_application_share_dir, 'rviz', 'move_to_pose.rviz')

    return LaunchDescription([
        Node(
            package='wego_ez',
            executable='distance_calculator',
            name='distance_calculator',
            output='screen'
        ),

        Node(
            package='wego_ez',
            executable='safety_decision',
            name='safety_decision',
        ),

        Node(
            package='wego_ez',
            executable='drive_limo',
            name='drie_limo'
        )
    ])