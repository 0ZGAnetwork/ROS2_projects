import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robot_interfaces.srv import HandleSensorError

class Sensor2Node(Node):
    def __init__(self, name):
        super().__init__(name)
        self.state = False
        self.publisher_ = self.create_publisher(Bool, f'/{name}/status', 10)
        self.timer = self.create_timer(1.0, self.publish_state)
        self.srv = self.create_service(HandleSensorError, f'restart_{name}',self.restart_callback)
    
    def publish_state(self):
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'State: {self.state}')

    def restart_callback(self, request, response):
        self.get_logger().info(f'Restarting {request.sensor_name}')
        self.state = True
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Sensor2Node('sensor2')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()