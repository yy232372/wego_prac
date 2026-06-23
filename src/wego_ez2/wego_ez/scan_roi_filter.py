import copy
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanROIFilter(Node):
    def __init__(self):
        super().__init__('scan_roi_filter')

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.publisher_ = self.create_publisher(
            LaserScan,
            'scan_roi',
            rclpy.qos.qos_profile_sensor_data
        )

        # 남길 각도 범위
        # 단위: degree
        self.min_angle = math.radians(-60.0)
        self.max_angle = math.radians(60.0)

    def scan_callback(self, msg):
        roi_msg = copy.deepcopy(msg)

        for i, distance in enumerate(msg.ranges):
            current_angle = msg.angle_min + msg.angle_increment * i

            # 각도 범위 밖이면 제거
            if current_angle < self.min_angle or current_angle > self.max_angle:
                roi_msg.ranges[i] = float('inf')

                if len(roi_msg.intensities) == len(roi_msg.ranges):
                    roi_msg.intensities[i] = 0.0

        self.publisher_.publish(roi_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ScanROIFilter()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()