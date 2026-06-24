import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener, TransformException


class MoveToPose(Node):
    def __init__(self):
        super().__init__('move_to_pose')

        # =========================
        # TF 설정
        # =========================
        # TransformListener가 내부적으로 /tf, /tf_static을 구독함
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =========================
        # move_to_pose 제어 파라미터
        # =========================
        self.declare_parameter('Kp_rho', 0.2)
        self.declare_parameter('Kp_alpha', 1.7)
        self.declare_parameter('Kp_beta', 1.0)

        self.declare_parameter('xy_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.15)

        self.declare_parameter('max_linear', 0.6)
        self.declare_parameter('max_angular', 1.0)

        self.Kp_rho = self.get_parameter('Kp_rho')
        self.Kp_alpha = self.get_parameter('Kp_alpha')
        self.Kp_beta = self.get_parameter('Kp_beta')

        self.xy_tolerance = self.get_parameter('xy_tolerance')
        self.yaw_tolerance = self.get_parameter('yaw_tolerance')

        self.max_linear = self.get_parameter('max_linear')
        self.max_angular = self.get_parameter('max_angular')

        # =========================
        # 장애물 회피 설정
        # =========================

        # 정면 장애물이 이 거리보다 가까우면 무조건 정지
        self.stop_distance = 0.20

        # 정면 장애물이 이 거리보다 가까우면 우회
        self.avoid_distance = 0.60

        # 우회 중 속도
        self.avoid_linear = 0.08
        self.avoid_angular = 0.55

        # 라이다 각도 영역
        # 정면: -20도 ~ +20도
        # 왼쪽: +20도 ~ +60도
        # 오른쪽: -60도 ~ -20도
        self.front_angle = math.radians(20.0)
        self.side_min_angle = math.radians(20.0)
        self.side_max_angle = math.radians(60.0)

        # 라이다 거리 초기값
        self.scan_received = False
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        # 상태는 STOP / DRIVE 두 개만 사용
        self.safety_state = "STOP"

        self.goal_reached = False

        # =========================
        # Publisher
        # =========================
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            'traj',
            10
        )

        self.path = Path()
        self.path.header.frame_id = 'odom'

        # =========================
        # Subscriber
        # =========================
        # ROI 처리된 라이다 데이터 구독
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_roi',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # 경로 기록용 odometry
        # /odometry/filtered가 없으면 경로 시각화만 안 될 수 있음
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            10
        )

        # 0.1초마다 제어 실행
        self.timer = self.create_timer(
            0.1,
            self.control_limo
        )

        self.get_logger().info(
            'move_to_pose started: STOP/DRIVE + obstacle avoidance'
        )

    # ============================================================
    # /scan_roi 콜백
    # ============================================================
    def scan_callback(self, msg):
        front_dist = float('inf')
        left_dist = float('inf')
        right_dist = float('inf')

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + msg.angle_increment * i
            angle = self.normalize_angle(angle)

            # 정면 영역
            if -self.front_angle <= angle <= self.front_angle:
                front_dist = min(front_dist, distance)

            # 왼쪽 영역
            elif self.side_min_angle <= angle <= self.side_max_angle:
                left_dist = min(left_dist, distance)

            # 오른쪽 영역
            elif -self.side_max_angle <= angle <= -self.side_min_angle:
                right_dist = min(right_dist, distance)

        self.front_dist = front_dist
        self.left_dist = left_dist
        self.right_dist = right_dist
        self.scan_received = True

        # STOP / DRIVE 두 조건만 사용
        if self.front_dist <= self.stop_distance:
            self.safety_state = "STOP"
        else:
            self.safety_state = "DRIVE"

    # ============================================================
    # Odometry 콜백: 경로 저장용
    # ============================================================
    def odom_callback(self, msg):
        point = PoseStamped()
        point.header = msg.header
        point.pose = msg.pose.pose
        self.path.poses.append(point)

    # ============================================================
    # 제어 메인 함수
    # ============================================================
    def control_limo(self):
        # 라이다 데이터가 아직 없으면 안전하게 정지
        if not self.scan_received:
            self.publish_stop()
            return

        # 목표 도착 후에는 정지 유지
        if self.goal_reached:
            self.publish_stop()
            return

        # =========================
        # 1. STOP 조건
        # =========================
        if self.safety_state == "STOP":
            self.publish_stop()
            return

        # =========================
        # 2. DRIVE 상태에서 우회 판단
        # =========================
        # 정면 장애물이 avoid_distance 안에 있으면 우회
        if self.front_dist <= self.avoid_distance:
            self.publish_avoid_cmd()
            return

        # =========================
        # 3. 장애물이 없으면 기존 목표점 추종
        # =========================
        self.publish_go_to_goal_cmd()

    # ============================================================
    # 정지 명령
    # ============================================================
    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    # ============================================================
    # 우회 명령
    # ============================================================
    def publish_avoid_cmd(self):
        msg = Twist()

        # 천천히 전진하면서 회피
        msg.linear.x = self.avoid_linear

        # 왼쪽과 오른쪽 중 더 넓은 쪽으로 회전
        if self.left_dist >= self.right_dist:
            msg.angular.z = self.avoid_angular
        else:
            msg.angular.z = -self.avoid_angular

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'AVOID | front: {self.front_dist:.2f}, '
            f'left: {self.left_dist:.2f}, right: {self.right_dist:.2f}'
        )

    # ============================================================
    # 기존 move_to_pose 목표점 추종
    # ============================================================
    def publish_go_to_goal_cmd(self):
        try:
            # base_link 기준 goal_pose 위치/자세 조회
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'goal_pose',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            self.publish_stop()
            return

        x_diff = t.transform.translation.x
        y_diff = t.transform.translation.y

        yaw = self.get_yaw_from_quaternion(t.transform.rotation)
        yaw_diff = self.normalize_angle(yaw)

        # 목표까지 거리
        rho = np.hypot(y_diff, x_diff)

        # 목표 방향
        alpha = self.normalize_angle(np.arctan2(y_diff, x_diff))

        # 최종 목표 자세 보정값
        beta = self.normalize_angle(yaw_diff - alpha)

        # 목표 도착 판단
        if rho < self.xy_tolerance.value and abs(yaw_diff) < self.yaw_tolerance.value:
            self.goal_reached = True

            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)

            self.publish_stop()
            self.get_logger().info('Goal reached')
            return

        # 선속도
        v = self.Kp_rho.value * rho

        # 각속도
        if rho > self.xy_tolerance.value:
            w = self.Kp_alpha.value * alpha - self.Kp_beta.value * beta
        else:
            w = self.Kp_beta.value * self.normalize_angle(yaw_diff)

        # 목표가 뒤쪽에 있으면 후진
        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        msg = Twist()

        msg.linear.x = min(
            max(v, -self.max_linear.value),
            self.max_linear.value
        )

        msg.angular.z = min(
            max(w, -self.max_angular.value),
            self.max_angular.value
        )

        self.cmd_pub.publish(msg)

    # ============================================================
    # Quaternion → yaw
    # ============================================================
    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ============================================================
    # angle 정규화: -pi ~ pi
    # ============================================================
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)

    node = MoveToPose()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
