# subscribe topic from controller
# publish topic to rviz_foxglove
# calculate for unicycle kinematics model
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class RobotMonitorSim(Node):
    def __init__(self):
        super().__init__('position')
        self.v = 0.0
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.teta = 0.0
        
        self.last_time = time.time()

        self.subs = self.create_subscription(Twist, '/cmd_vel', self.reset_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(2.0, self.update_callback)

    def reset_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_callback(self):
        dt = time.time() - self.last_time
        self.last_time = time.time()

        self.x += self.v * math.cos(self.teta) * dt
        self.y += self.v * math.sin(self.teta) * dt
        self.teta += self.w * dt

        odom = Odometry()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.pub.publish(odom)

def __main__(args=None):
    rclpy.init(args=args)
    node = RobotMonitorSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    __main__()
