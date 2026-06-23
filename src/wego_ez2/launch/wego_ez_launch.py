from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. 원본 /scan을 받아서 ROI 적용 후 /scan_roi 발행
        Node(
            package='wego_ez2',
            executable='scan_roi_filter',
            name='scan_roi_filter',
            output='screen'
        ),

        # 2. ROI 처리된 /scan_roi를 받아서 전방 최단거리 계산
        Node(
            package='wego_ez2',
            executable='distance_calculator',
            name='distance_calculator',
            output='screen'
        ),

        # 3. 거리값 /front_dist를 받아서 STOP / SLOW / DRIVE 판단
        Node(
            package='wego_ez2',
            executable='safety_decision',
            name='safety_decision',
            output='screen'
        ),

        # 4. 안전 상태 /safety_state를 받아서 /cmd_vel 발행
        Node(
            package='wego_ez2',
            executable='drive_limo',
            name='drive_limo',
            output='screen'
        )
    ])
