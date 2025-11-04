import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robot_interfaces.srv import HandleSensorError

class StateMonitor(Node):
    def __init__(self):
        super().__init__('sub_sensors')
        self.sensors = ['sensor1','sensor2']
        self.sensor_states = {s: True for s in self.sensors}

        for sensor in self.sensors:
            self.create_subscription(
                Bool,
                f'/{sensor}/status',
                lambda msg, s=sensor: self.sensor_callback(msg, s),
                10
            )
    
    def sensor_callback(self, msg, sensor_name):
        if not msg.data:
            self.get_logger().warn(f'{sensor_name} reported error!')

            self.call_error_handler(sensor_name)

    def call_error_handler(self, sensor_name):
        client = self.create_client(HandleSensorError, 'handle_sensor_error')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            return
        req = HandleSensorError.Request()
        req.sensor_name = sensor_name
        client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()