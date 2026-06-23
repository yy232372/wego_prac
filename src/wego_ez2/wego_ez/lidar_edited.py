import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from math import isfinite


# ROI 처리된 라이다 데이터에서 가장 가까운 장애물 거리를 계산하는 노드
class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')

        # ROI 필터가 발행한 /scan_roi 토픽 구독
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_roi',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.subscription

        # 계산된 전방 최단거리 발행
        self.publisher_ = self.create_publisher(
            Float32,
            'front_dist',
            10
        )

    def laser_callback(self, msg):
        # ROI 안에서 가장 가까운 거리
        front_dist = float('inf')

        for data in msg.ranges:
            # inf, nan 제거
            if not isfinite(data):
                continue

            # 라이다 유효 측정 범위 밖이면 제거
            if data < msg.range_min or data > msg.range_max:
                continue

            # /scan_roi는 이미 각도 ROI 밖 데이터가 inf 처리되어 있으므로
            # 여기서는 남아 있는 유효 거리 중 최솟값만 찾으면 됨
            front_dist = min(front_dist, data)

        # ROI 안에 유효한 장애물이 없으면 충분히 먼 거리로 처리
        # 안전을 위해 센서값 없음 = 정지로 처리하고 싶으면 0.0으로 바꾸면 됨
        if front_dist == float('inf'):
            front_dist = msg.range_max

        self.get_logger().info(
            f'Closest distance in ROI: {front_dist:.2f} m'
        )

        cal_dist = Float32()
        cal_dist.data = round(float(front_dist), 2)
        self.publisher_.publish(cal_dist)


def main(args=None):
    rclpy.init(args=args)

    distance_calculator = DistanceCalculator()
    rclpy.spin(distance_calculator)

    distance_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
