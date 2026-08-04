import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber1')

        self.subscriber_ = self.create_subscription(String, 'sensor1_velocity', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main(arg=None):

    rclpy.init(args=None)
    subscriber1 = Subscriber()

    try:
        rclpy.spin(subscriber1)
    except KeyboardInterrupt:
        subscriber1.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        subscriber1.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()