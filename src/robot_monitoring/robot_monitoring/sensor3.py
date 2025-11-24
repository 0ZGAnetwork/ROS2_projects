import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from robot_interfaces.srv import HandleSensorError

class Sensor3Node(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('sensor3', True)
        self.sensor3 = self.get_parameter('sensor3').value
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.state = bool(self.sensor3)
        self.publisher_ = self.create_publisher(Bool, f'/{name}/state', 10)
        self.timer = self.create_timer(1.0, self.publish_state)
        self.srv = self.create_service(HandleSensorError, f'restart_{name}', self.restart_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'sensor3' and isinstance(param.value, bool):
                self.sensor3 = param.value
                self.state = bool(self.sensor3)
                self.get_logger().info(f'Update sensor3 state to {self.sensor3}')
            self.publish_state()
        return SetParametersResult(successful=True)

    def publish_state(self):
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'State: {self.state}')

    def restart_callback(self, request, response):
        self.get_logger().info(f'Restarting {request.sensor_name}...')
        self.state = True
        response.success = True
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = Sensor3Node('sensor3')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()