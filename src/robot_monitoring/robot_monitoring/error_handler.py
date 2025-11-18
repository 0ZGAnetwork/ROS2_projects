import rclpy
from rclpy.node import Node
from robot_interfaces.srv  import HandleSensorError
from robot_interfaces.srv  import HandleSensorError as RestartService

class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler')

        self.srv = self.create_service(
            HandleSensorError,
            'handle_sensor_error',
            self.handle_sensor_error_callback
        )

    def handle_sensor_error_callback(self, request, response):
        sensor_name = request.sensor_name
        self.get_logger().warn(f'Handling error from {sensor_name}. Restarting sensor...')

        restart_client = self.create_client(RestartService, f'restart_{sensor_name}')
        if not restart_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service restart_{sensor_name} not available.')
            response.success = False
            return response
        
        restart_request = RestartService.Request()
        restart_request.sensor_name = sensor_name
        future = restart_client.call_async(restart_request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'{sensor_name} restarted successfully.')
            response.success = True
        else:
            self.get_logger().error(f'Failed to restart {sensor_name}.')
            response.success = False
        return response
    
    def call_error_handler(self, sensor_name):
        client = self.create_client(HandleSensorError, 'handle_sensor_error')

        # Poczekaj, aż handler będzie dostępny
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Error handler not available, skipping.')
            return

        # Dodatkowe sprawdzenie czy istnieje restart service dla tego sensora
        restart_client = self.create_client(HandleSensorError, f'restart_{sensor_name}')
        if not restart_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service restart_{sensor_name} not available. Sensor probably offline.')
            return  # <-- tu kończymy, nie próbujemy restartować

        # Dopiero teraz wołamy handler
        req = HandleSensorError.Request()
        req.sensor_name = sensor_name
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response:
            self.get_logger().info(f'ErrorHandler response: success={response.success}')


def main(args=None):
    rclpy.init(args=args)
    node = ErrorHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()