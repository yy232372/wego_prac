import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from math import cos, sin, sqrt, isfinite

# 라이다 센서로부터 거리 정보를 수신하고, 가장 가까운 장애물까지의 거리를 계산하는 클래스
class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        # '/scan' 토픽을 구독하여 LaserScan 메시지 수신
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription
        
        # 계산된 거리를 Float32 메시지로 'front_dist' 토픽에 발행
        self.publisher_ = self.create_publisher(Float32, 'front_dist', 10)
        
    def laser_callback(self, msg):
        # 최소 거리를 무한대로 초기화
        front_dist = float('inf')

        # 라이다 스캔의 각 레이에 대해 반복
        for i, data in enumerate(msg.ranges):
            # 유효하지 않은 데이터(inf, nan)는 스킵
            if not isfinite(data):
                continue
            # 현재 레이의 각도 계산
            current_angle = msg.angle_min + msg.angle_increment * i
            # 극좌표를 직교좌표로 변환: cx(전방 거리), cy(좌우 거리)
            cx = data * cos(current_angle)
            cy = data * sin(current_angle)
            # 전방에 있는 장애물만 검사 (cx > 0.01 = 앞쪽)
            if cx > 0.01:
                # 거리 계산 (직선거리)
                dist = sqrt(cx**2 + cy**2)
                # 가장 가까운 거리 업데이트
                front_dist = min(front_dist, dist)

        # 유효한 거리가 없으면 0으로 설정
        if front_dist == float('inf'):
            front_dist = 0.0

        # 계산된 거리를 로그에 출력 (소수점 둘째 자리)
        self.get_logger().info(f"The closest distance: {front_dist:.2f} m")

        # 계산된 거리를 Float32 메시지로 변환 후 발행
        cal_dist = Float32()
        cal_dist.data = round(front_dist, 2)
        self.publisher_.publish(cal_dist)

def main(args=None):
    rclpy.init(args=args)

    distance_calculator = DistanceCalculator()
    rclpy.spin(distance_calculator)

    distance_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()