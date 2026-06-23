import rclpy, math
from rclpy.node import Node

from std_msgs.msg import Float32, String

# 거리 정보를 받아서 안전 상태(STOP/SLOW/DRIVE)를 결정하는 클래스
class SafetyDecision(Node):
    def __init__(self):
        super().__init__('safety_decision')
        # 'front_dist' 토픽에서 Float32 메시지(거리) 구독
        self.subscription = self.create_subscription(
            Float32,
            'front_dist',
            self.dist_callback,
            10
        )
        self.subscription

        # 결정된 안전 상태를 String 메시지로 'safety_state' 토픽에 발행
        self.publisher_ = self.create_publisher(
            String,
            'safety_state',
            10
        )
    
    def dist_callback(self, msg):
        # 안전 상태를 저장할 String 메시지 객체 생성
        state = String()
        # 거리에 따라 안전 상태 결정
        if msg.data <= 0.2:
            # 0.2m 이하 = 아주 가까운 장애물 = 즉시 정지
            state.data = "STOP"
        elif msg.data <= 0.5:
            # 0.2m ~ 0.5m = 중간 거리 장애물 = 감속
            state.data = "SLOW"
        else:
            # 0.5m 이상 = 충분한 거리 = 정상 주행
            state.data = "DRIVE"
        
        # 결정된 안전 상태 발행
        self.publisher_.publish(state)

def main(args=None):
    rclpy.init(args=args)

    safety_decision = SafetyDecision()
    rclpy.spin(safety_decision)

    safety_decision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()