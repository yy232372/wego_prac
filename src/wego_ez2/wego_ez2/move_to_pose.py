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

        # 너무 가까우면 충돌 방지용 완전 정지
        # 단, 너무 크게 잡으면 또 멈추기만 하니까 아주 작게 둠
        self.emergency_stop_distance = 0.08

        # 이 거리 안에 정면 장애물이 들어오면 미리 우회 시작
        self.avoid_enter_distance = 1.00

        # 이 거리보다 정면이 충분히 멀어져야 우회 종료
        # enter보다 크게 잡아서 바로 복귀하지 않게 함
        self.avoid_exit_distance = 1.30

        # 한 번 우회 시작하면 최소 이 시간 동안은 우회 유지
        self.min_avoid_time = 1.5

        # 우회 속도
        self.avoid_linear = 0.20
        self.avoid_angular = 0.95

        # 너무 가까울 때 후진 회피 속도
        self.escape_distance = 0.20
        self.escape_linear = -0.08
        self.escape_angular = 0.90

        # 라이다 각도 영역
        # 정면을 조금 넓게 봐야 얇은 장애물을 미리 감지함
        # 정면: -30도 ~ +30도
        # 왼쪽: +30도 ~ +75도
        # 오른쪽: -75도 ~ -30도
        self.front_angle = math.radians(30.0)
        self.side_min_angle = math.radians(30.0)
        self.side_max_angle = math.radians(75.0)

        # 라이다 거리 초기값
        self.scan_received = False
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        # 우회 상태 저장
        self.avoid_mode = False
        self.avoid_direction = 1.0
        self.avoid_start_time = None

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
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_roi',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            10
        )

        # =========================
        # Timer
        # =========================
        self.timer = self.create_timer(
            0.1,
            self.control_limo
        )

        self.get_logger().info(
            'move_to_pose started: early obstacle avoidance + hold turn'
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
        if not self.scan_received:
            self.publish_stop()
            return

        if self.goal_reached:
            self.publish_stop()
            return

        # 1. 정말 너무 가까우면 완전 정지
        if self.front_dist <= self.emergency_stop_distance:
            self.publish_stop()
            self.get_logger().warn(
                f'EMERGENCY STOP | front: {self.front_dist:.2f}'
            )
            return

        # 2. 매우 가까우면 후진하면서 회피
        # Ackermann은 제자리 회전이 안 되므로, 후진 속도를 같이 줌
        if self.front_dist <= self.escape_distance:
            self.start_or_keep_avoid_mode()
            self.publish_escape_cmd()
            return

        # 3. 우회 진입 조건
        if not self.avoid_mode and self.front_dist <= self.avoid_enter_distance:
            self.start_avoid_mode()

        # 4. 우회 모드 유지 / 종료 판단
        if self.avoid_mode:
            if self.can_exit_avoid_mode():
                self.avoid_mode = False
                self.get_logger().info('AVOID END -> GO TO GOAL')
            else:
                self.publish_avoid_cmd()
                return

        # 5. 장애물이 없으면 기존 목표점 추종
        self.publish_go_to_goal_cmd()

    # ============================================================
    # 우회 시작
    # ============================================================
    def start_avoid_mode(self):
        self.avoid_mode = True
        self.avoid_start_time = self.get_clock().now()

        # 우회 방향 결정
        # 왼쪽 공간이 더 넓으면 왼쪽으로, 오른쪽이 넓으면 오른쪽으로
        if self.left_dist >= self.right_dist:
            self.avoid_direction = 1.0
        else:
            self.avoid_direction = -1.0

        self.get_logger().info(
            f'AVOID START | dir: {self.avoid_direction}, '
            f'front: {self.front_dist:.2f}, '
            f'left: {self.left_dist:.2f}, right: {self.right_dist:.2f}'
        )

    # ============================================================
    # 이미 우회 중이면 방향 유지
    # ============================================================
    def start_or_keep_avoid_mode(self):
        if not self.avoid_mode:
            self.start_avoid_mode()

    # ============================================================
    # 우회 종료 조건
    # ============================================================
    def can_exit_avoid_mode(self):
        if self.avoid_start_time is None:
            return False

        now = self.get_clock().now()
        elapsed = (now - self.avoid_start_time).nanoseconds / 1e9

        # 최소 우회 시간 전에는 종료 금지
        if elapsed < self.min_avoid_time:
            return False

        # 정면이 충분히 멀어져야 종료
        if self.front_dist < self.avoid_exit_distance:
            return False

        return True

    # ============================================================
    # 정지 명령
    # ============================================================
    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    # ============================================================
    # 너무 가까울 때 후진 회피
    # ============================================================
    def publish_escape_cmd(self):
        msg = Twist()

        # Ackermann은 제자리 회전 불가
        # 후진하면서 조향해야 방향이 바뀜
        msg.linear.x = self.escape_linear
        msg.angular.z = self.avoid_direction * self.escape_angular

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'ESCAPE | front: {self.front_dist:.2f}, '
            f'left: {self.left_dist:.2f}, right: {self.right_dist:.2f}'
        )

    # ============================================================
    # 일반 우회 명령
    # ============================================================
    def publish_avoid_cmd(self):
        msg = Twist()

        # 얇은 장애물에서 꺾다가 말지 않게 어느 정도 속도와 조향을 줌
        msg.linear.x = self.avoid_linear
        msg.angular.z = self.avoid_direction * self.avoid_angular

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'AVOID | front: {self.front_dist:.2f}, '
            f'left: {self.left_dist:.2f}, right: {self.right_dist:.2f}, '
            f'dir: {self.avoid_direction}'
        )

    # ============================================================
    # 기존 move_to_pose 목표점 추종
    # ============================================================
    def publish_go_to_goal_cmd(self):
        try:
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

        rho = np.hypot(y_diff, x_diff)
        alpha = self.normalize_angle(np.arctan2(y_diff, x_diff))
        beta = self.normalize_angle(yaw_diff - alpha)

        if rho < self.xy_tolerance.value and abs(yaw_diff) < self.yaw_tolerance.value:
            self.goal_reached = True

            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)

            self.publish_stop()
            self.get_logger().info('Goal reached')
            return

        v = self.Kp_rho.value * rho

        if rho > self.xy_tolerance.value:
            w = self.Kp_alpha.value * alpha - self.Kp_beta.value * beta
        else:
            w = self.Kp_beta.value * self.normalize_angle(yaw_diff)

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
