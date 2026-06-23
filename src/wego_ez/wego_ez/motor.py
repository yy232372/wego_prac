import rclpy
from rclpy.node import Node

# LIMO 로봇 모터 제어를 위한 임포트
from geometry_msgs.msg import Twist  # 속도 명령 메시지
from std_msgs.msg import String

# 안전 상태에 따라 로봇의 선속도(linear velocity)를 제어하는 클래스
class DriveLimo(Node):
    def __init__(self):
        super().__init__('drive_limo')

        # 'safety_state' 토픽에서 String 메시지(안전 상태) 구독
        self.subscription = self.create_subscription(
            String,
            'safety_state',
            self.safety_callback,
            10
        )
        self.subscription

        # 로봇의 이동 속도 명령을 Twist 메시지로 'cmd_vel' 토픽에 발행
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def safety_callback(self, msg):
        # 속도 명령을 저장할 Twist 메시지 객체 생성 (기본값: 모든 속도 = 0)
        speed = Twist()
        
        # 안전 상태에 따라 선속도 설정
        if msg.data == "STOP":
            # STOP: 로봇 정지 (선속도 = 0.0 m/s)
            speed.linear.x = 0.0
        elif msg.data == "SLOW":
            # SLOW: 감속 주행 (선속도 = 0.5 m/s)
            speed.linear.x = 0.5
        else:
            # DRIVE: 정상 주행 (선속도 = 1.0 m/s)
            speed.linear.x = 1.0
        
        # 계산된 속도 명령 발행
        self.cmd_pub.publish(speed)
            
def main(args=None):
    rclpy.init(args=args)

    drive_limo = DriveLimo()
    rclpy.spin(drive_limo)

    drive_limo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()