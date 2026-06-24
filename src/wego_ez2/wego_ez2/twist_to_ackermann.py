import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')

        # LIMO Ackermann 기준값
        # limo_ackerman_driver.hpp에서 wheelbase_ = 0.2m, max_steering_angle_ = 0.42rad
        self.wheelbase = 0.2
        self.max_steering_angle = 0.42

        # driver cpp에서 steering_angle을 받을 때 / 1.215를 함.
        # 그래서 여기서 1.215를 곱해서 보내면 driver 내부에서 원래 조향각으로 돌아감.
        self.driver_steering_scale = 1.215

        # 속도가 너무 작을 때 steering_angle = atan(L*w/v)가 폭주하는 것 방지
        self.min_speed_for_steering = 0.03

        # move_to_pose가 발행하는 Twist 구독
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Ackermann driver가 구독하는 토픽 발행
        # limo_ackerman_base가 namespace='limo'로 실행되므로 실제 토픽은 /limo/ack_cmd
        self.ack_pub = self.create_publisher(
            AckermannDrive,
            '/limo/ack_cmd',
            10
        )

        self.get_logger().info(
            'twist_to_ackermann started: /cmd_vel -> /limo/ack_cmd'
        )

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        ack_msg = AckermannDrive()

        # Ackermann 차량은 제자리 회전이 불가능함.
        # linear.x가 거의 0이면 안전하게 정지 처리.
        if abs(v) < self.min_speed_for_steering:
            ack_msg.speed = 0.0
            ack_msg.steering_angle = 0.0
        else:
            # Bicycle model:
            # w = v / L * tan(delta)
            # delta = atan(L * w / v)
            steering_angle = math.atan(self.wheelbase * w / v)

            # 조향각 제한
            steering_angle = max(
                min(steering_angle, self.max_steering_angle),
                -self.max_steering_angle
            )

            ack_msg.speed = float(v)

            # driver 내부에서 /1.215를 하므로 여기서 보정해서 보냄
            ack_msg.steering_angle = float(
                steering_angle * self.driver_steering_scale
            )

        # 현재 driver는 speed와 steering_angle만 사용하지만 기본값 채워둠
        ack_msg.steering_angle_velocity = 0.0
        ack_msg.acceleration = 0.0
        ack_msg.jerk = 0.0

        self.ack_pub.publish(ack_msg)


def main(args=None):
    rclpy.init(args=args)

    node = TwistToAckermann()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
