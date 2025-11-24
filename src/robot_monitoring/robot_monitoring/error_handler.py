import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robot_interfaces.srv import HandleSensorError

class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler')
        self.srv = self.create_service(HandleSensorError, 'handling_error', self.handle_error_callback)
        self.get_logger().info('HandleSensorError service ready.')
        self.reset_publisher ={}

    def handle_error_callback(self, request, response):
        sensor =request.sensor_name
        self.get_logger().warning(f"Handling error for {sensor}, resetting...")

        if sensor not in self.reset_publisher:
            self.reset_publisher[sensor] = self.create_publisher(Bool, f'/{sensor}/reset', 10)
        msg = Bool()
        msg.data = True
        self.reset_publisher[sensor].publish(msg)

        response.success = True
        response.message = f"Sensor {request.sensor_name} has been reset."
        return response

    
def main(args=None):
    rclpy.init(args=args)
    node = ErrorHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()