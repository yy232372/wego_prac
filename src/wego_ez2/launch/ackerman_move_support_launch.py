from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 2. Ackermann base 실행
        # pub_odom_tf=True로 설정해서 odom -> base_link TF 발행
        Node(
            package='limo_base',
            executable='limo_ackerman_base',
            namespace='limo',
            name='limo_ackerman_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port_name': 'ttylimo',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'pub_odom_tf': True,
            }]
        ),

        # 3. odom -> goal_pose 고정 TF 발행
        # 목표점: odom 기준 x=1.0m, y=0.0m
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='goal_pose_publisher',
            output='screen',
            arguments=[
                '3.0', '-1.0', '0.0',          # x y z
                '0.0', '0.0', '0.0', '1.0',   # qx qy qz qw
                'odom',
                'goal_pose'
            ]
        ),

        # 5. /cmd_vel(Twist)을 /limo/ack_cmd(AckermannDrive)로 변환
        Node(
            package='wego_ez2',
            executable='twist_to_ackermann',
            name='twist_to_ackermann',
            output='screen'
        )
    ])
