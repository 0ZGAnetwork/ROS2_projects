import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from robot_interfaces.srv import HandleSensorError

class Sensor2Node(Node):
    def __init__(self, name):
        super().__init__(name)
        #self.declare_parameter('sensor1', True)
        self.declare_parameter('sensor2', True)
        self.sensor2 = self.get_parameter('sensor2').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.state = bool(self.sensor2)
        self.publisher_ = self.create_publisher(Bool, f'/{name}/state', 10)
        self.timer = self.create_timer(1.0, self.publish_state)
        self.srv = self.create_service(HandleSensorError, f'restart_{name}',self.restart_callback)
        self.subs = self.create_subscription(Bool, f'/{name}/reset', self.reset_callback, 10)
    
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
    
    def parameter_callback(self, params):
        # params: list[rclpy.parameter.Parameter]
        for param in params:
            if param.name == 'sensor2' and isinstance(param.value, bool):
                self.sensor2 = param.value
                self.state = bool(self.sensor2)
                self.get_logger().info(f'Update sensor2 state to {self.sensor2}')
            # elif param.name == 'sensor2' and isinstance(param.value, bool):
            #     self.sensor2 = param.value
            #     self.get_logger().info(f'Update sensor2 state to {self.sensor2}')
        self.publish_state()
        return SetParametersResult(successful=True)
    
    def reset_callback(self, msg):
        if msg.data:
            self.get_logger().info('Reset received, setting state to True')
            self.state = True
            self.publish_state()
    
def main(args=None):
    rclpy.init(args=args)
    node = Sensor2Node('sensor2')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()