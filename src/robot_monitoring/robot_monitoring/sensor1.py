import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from robot_interfaces.srv import HandleSensorError

class Sensor1Node(Node):
    def __init__(self, name):
        super().__init__(name)
        # declare boolean parameters with defaults so ros2 param set can change them
        self.declare_parameter('sensor1', True)
        self.declare_parameter('sensor2', True)
        self.sensor1 = self.get_parameter('sensor1').value
        #self.sensor2 = self.get_parameter('sensor2').value

        # register the correct parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # this node represents sensor1, so state initialized from sensor1 param
        self.state = bool(self.sensor1)
        self.publisher_ = self.create_publisher(Bool, f'/{name}/status', 10)
        self.timer = self.create_timer(1.0, self.publish_state)
        self.srv = self.create_service(HandleSensorError, f'restart_{name}', self.restart_callback)

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

    def parameter_callback(self, params):
        # params: list[rclpy.parameter.Parameter]
        for param in params:
            if param.name == 'sensor1' and isinstance(param.value, bool):
                self.sensor1 = param.value
                self.state = bool(self.sensor1)
                self.get_logger().info(f'Update sensor1 state to {self.sensor1}')
        #     elif param.name == 'sensor2' and isinstance(param.value, bool):
        #         self.sensor2 = param.value
        #         self.get_logger().info(f'Update sensor2 state to {self.sensor2}')
        self.publish_state()
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = Sensor1Node('sensor1')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()