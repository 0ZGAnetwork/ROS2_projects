import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.pub = self.create_publisher(Twist, '/cmd_vel',10)
        self.timer = self.create_timer(0.1, self.pub_callback)

    def pub_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.1
        self.pub.publish(msg)
        self.get_logger().info('Publishing /cmd_vel{linear.x: %.2f, angular.z: %.2f}' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()