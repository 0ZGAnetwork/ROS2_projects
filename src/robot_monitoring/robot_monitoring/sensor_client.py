import rclpy
from rclpy.node import Node
from robot_interfaces.srv import HandleSensorError

class SensorClient(Node):
    def __init__(self):
        super().__init__('sensor_client')
        self.client = self.create_client(HandleSensorError, 'restart_sensor1')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.request = HandleSensorError.Request()
        self.request.sensor_name = 'sensor1'

    def send_request(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = SensorClient()
    response = node.send_request()
    node.get_logger().info(f'Service call success: {response.success}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
