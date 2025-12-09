import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

qos_profile = QoSProfile(
    depth=10
)

class Controller(Node):
    def __init__(self):
        super().__init__('controller')  # init variables
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('angular_speed', 0.0)
        self.u1 = self.get_parameter('linear_speed').value
        self.u2 = self.get_parameter('angular_speed').value

        self.add_on_set_parameters_callback(self.param_callback)

        self.pub = self.create_publisher(Twist, 'Twist', qos_profile)
        self.timer = self.create_timer(0.2, self.pub_callback)
        

    def pub_callback(self): # send msg
        msg = Twist()
        msg.linear.x = self.u1
        msg.angular.z = self.u2
        self.pub.publish(msg)
        #self.get_logger().info(f'Publishing linear {msg.linear.x}, angular {msg.angular.z}')

    def param_callback(self, params): #set variables
        for param in params:
            if param.name == "linear_speed":
                self.u1 = param.value
            elif param.name == "angular_speed":
                self.u2 = param.value
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        print(' exit :3')
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()