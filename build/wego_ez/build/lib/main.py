import rclpy, math
from rclpy.node import Node

from std_msgs.msg import Float32, String

class SafetyDecision(Node):
    def __init__(self):
        super().__init__('safety_decision')
        self.subscription = self.create_subscription(
            Float32,
            'front_dist',
            self.dist_callback,
            10
        )
        self.subscription

        self.publisher_ = self.create_publisher(
            String,
            'safety_state',
            10
        )
    
    def dist_callback(self, msg):
        state = String()
        if msg.data <= 0.1:
            state.data = "STOP"
        elif msg.data <=0.2:
            state.data = "SLOW"
        else:
            state.data = "DRIVE"
        
        self.publisher_.publish(state)

def main(args=None):
    rclpy.init(args=args)

    safety_decision = SafetyDecision()
    rclpy.spin(safety_decision)

    safety_decision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()