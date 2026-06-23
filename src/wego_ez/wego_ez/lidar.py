import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from math import cos, sin, sqrt, isfinite

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription
        
        self.publisher_ = self.create_publisher(Float32, 'front_dist', 10)
        
    def laser_callback(self, msg):
        front_dist = float('inf')

        for i, data in enumerate(msg.ranges):
            if not isfinite(data):
                continue
            current_angle = msg.angle_min + msg.angle_increment * i
            cx = data * cos(current_angle)
            cy = data * sin(current_angle)
            if cx > 0.01 and -0.1 < cy < 0.1:
                dist = sqrt(cx**2 + cy**2)
                front_dist = min(front_dist, dist)

        if front_dist == float('inf'):
            front_dist = 0.0

        self.get_logger().info(f"The closest distance: {front_dist:.2f} m")

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