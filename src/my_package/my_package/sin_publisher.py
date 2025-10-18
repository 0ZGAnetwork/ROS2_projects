import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class SinPublisher(Node):
    def __init__(self):
        super().__init__('sin_publisher')
        self.publisher_ = self.create_publisher(Float32, 'sin_wave', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info('Sinusoidal publisher node has been started.')

    def timer_callback(self):
        t = time.time() - self.start_time
        value = math.sin(t)
        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sin(t) = {value:.3f}')

def main(args=None):
    rclpy.init(args=args)   # init ROS2
    node=SinPublisher()     # create node
    rclpy.spin(node)        # loop node
    node.destroy_node()     # destroy node
    rclpy.shutdown()        # shutdown ROS2

if __name__ == '__main__':
    main()