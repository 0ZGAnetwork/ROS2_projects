import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class ScalerNode(Node):
    def __init__(self):
        super().__init__('scaler')
        self.declare_parameter('scaler_factor', 1.0)
        self.declare_parameter('offset', 0.0)

        self.scaler_factor = self.get_parameter('scaler_factor').value
        self.offset = self.get_parameter('offset').value

        self.publisher_ = self.create_publisher(Float32, 'modified_signal',10)
        self.subscription_ = self.create_subscription(
            Float32,
            'signal',
            self.callback,
            10
        )
        self.get_logger().info(f'Scaler node started: scaler_factor={self.scaler_factor}, offset={self.offset}')

    def callback(self, msg):
        scaled_value = msg.data * self.scaler_factor + self.offset
        modified_msg = Float32()
        modified_msg.data = scaled_value
        self.publisher_.publish(modified_msg)
        self.get_logger().info(f'Received {msg.data:.3f}, -> Scaled {scaled_value:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = ScalerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()