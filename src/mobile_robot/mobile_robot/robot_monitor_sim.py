import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time
from rcl_interfaces.msg import SetParametersResult

class RobotMonitorSim(Node):
    def __init__(self):
        super().__init__('simulator')
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('angular_speed', 0.0)
        self.u1 = self.get_parameter('linear_speed').value
        self.u2 = self.get_parameter('angular_speed').value
        #
        self.declare_parameter('initial_pose.x',0.0)
        self.declare_parameter('initial_pose.y',0.0)
        self.declare_parameter('initial_pose.z',0.0)
        self.declare_parameter('initial_pose.theta',0.0)
        self.x = self.get_parameter('initial_pose.x').value
        self.y = self.get_parameter('initial_pose.y').value
        self.z = self.get_parameter('initial_pose.z').value
        self.theta = self.get_parameter('initial_pose.theta').value
        #
        self.sub = self.create_subscription(Twist, 'Twist', self.control_callback, 10)
        self.t = self.get_clock().now().to_msg()
        self.pub_ = self.create_publisher(PoseStamped, 'PoseStamped', 10)
        self.timer = self.create_timer(0.1, self.state_update)
        self.last_time = self.get_clock().now()
        self.tf_broadcaster = TransformBroadcaster(self)
        #
        self.add_on_set_parameters_callback(self.param_callback)
    
    def control_callback(self, msg: Twist):
        self.u1 = msg.linear.x
        self.u2 = msg.angular.z
        self.get_logger().info(f'Receiving control linear {self.u1}, angular {self.u2}')

    def yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def state_update(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = now 

        self.x += self.u1 * math.cos(self.theta) *dt
        self.y += self.u1 * math.sin(self.theta) *dt
        self.theta += self.u2 * dt

        PS = PoseStamped()
        PS.header.stamp = now.to_msg()
        PS.header.frame_id = 'world'

        px = self.x
        py = self.y
        PS.pose.position.x = px
        PS.pose.position.y = py
        PS.pose.position.z = 0.0

        # qz = math.sin(self.theta / 2.0)
        # qw = math.cos(self.theta / 2.0)
        qx, qy, qz, qw = self.yaw_to_quaternion(self.theta)

        PS.pose.orientation.x = qx
        PS.pose.orientation.y = qy
        PS.pose.orientation.z = qz
        PS.pose.orientation.w = qw

        self.pub_.publish(PS)
        
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0 
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(f'Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')
        #self.get_logger().warning(f'Orientation qx={qx}, qy={qy}, qz={qz}, qw={qw}')


    def param_callback(self, params):
        for param in params:
            if param.name == "initial_pose.x":
                self.x = param.value
            elif param.name == "initial_pose.y":
                self.y = param.value
            elif param.name == "initial_pose.z":
                self.z = param.value
            elif param.name == "initial_pose.theta":
                self.theta = param.value
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        print(' exit :3')
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
