import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time
from rcl_interfaces.msg import SetParametersResult

class GenSinus(Node):
    def __init__(self):
        super().__init__('Generator')
        
        self.declare_parameter('amplitude',1.0)
        self.declare_parameter('frequency',0.1)
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.publisher_ = self.create_publisher(Float32, 'signal', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.start_time = time.time()

        self.get_logger().info(f'Amplitude={self.amplitude}, Frequency={self.frequency}')

    def timer_callback(self):
        t = time.time() - self.start_time
        value = self.amplitude * math.sin(2 * math.pi * self.frequency * t) # A*sin(\omega*t)+B
        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sin(t) = {value:.3f}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'amplitude' and param.type_ == param.Type.DOUBLE:
                self.amplitude = param.value
                self.get_logger().info(f'Updated amplitude to {self.amplitude}')
            elif param.name == 'frequency' and param.type_ == param.Type.DOUBLE:
                self.frequency = param.value
                self.get_logger().info(f'Updated frequency to {self.frequency}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)   # init ROS2
    node=GenSinus()         # create node
    rclpy.spin(node)        # loop node
    node.destroy_node()     # destroy node
    rclpy.shutdown()        # shutdown ROS2

if __name__ == '__main__':
    main()