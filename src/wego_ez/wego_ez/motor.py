import rclpy
from rclpy.node import Node

# for limo control
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class DriveLimo(Node):
    def __init__(self):
        super().__init__('drive_limo')

        self.subscription = self.create_subscription(
            String,
            'safety_state',
            self.safety_callback,
            10
        )
        self.subscription

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def safety_callback(self, msg):
        speed = Twist()
        if msg.data == "STOP":
            speed.linear.x = 0.0
        elif msg.data == "SLOW":
            speed.linear.x = 2.0
        else:
            speed.linear.x = 4.0
        
        self.cmd_pub.publish(speed)
            
