import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robot_interfaces.srv import HandleSensorError

class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')
        #self.sensors = ['sensor1','sensor2']
        self.declare_parameter('sensors', ['sensor1', 'sensor2'])
        self.declare_parameter('check_interval',2.0)

        self.sensors = self.get_parameter('sensors').get_parameter_value().string_array_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value


        self.sensor_states = {s: None for s in self.sensors}
        self.last_msg_time = {s: self.get_clock().now() for s in self.sensors}
        self.create_timer(self.check_interval, self.check_sensors)
        
        for sensor in self.sensors:
            self.create_subscription(
                Bool,
                f'/{sensor}/status',
                self.make_callback(sensor),
                10
            )
            self.get_logger().info(f'Subscribing to /{sensor}/status')
        self.robot_state_pub = self.create_publisher(Bool, '/robot_state', 10)

    def make_callback(self, sensor_name):
        def callback(msg):
            self.sensor_callback(msg, sensor_name)
        return callback

    def update_robot_state(self):
        msg = Bool()
        #msg.data =all(self.sensor_states.values())
        if any(v is False for v in self.sensor_states.values()):
            msg.data = False
        elif any(v is None for v in self.sensor_states.values()):
            self.get_logger().info('Waiting for all sensors to publish...')
            return
        else:
            msg.data = True
        self.robot_state_pub.publish(msg)

    def check_sensors(self):
        now = self.get_clock().now()
        for s in self.sensors:
            dt = (now - self.last_msg_time[s]).nanoseconds * 1e-9
            if dt > self.check_interval:
                self.get_logger().warn(f'{s} has stopped publishing! Triggering error handler.')
                self.sensor_states[s] = False
                self.call_error_handler(s)
        self.update_robot_state()


    def sensor_callback(self, msg, sensor_name):
        self.get_logger().info(f'Received {msg.data} from {sensor_name}')
        
        self.last_msg_time[sensor_name] = self.get_clock().now()
        self.sensor_states[sensor_name] = msg.data
        if not msg.data:
            self.get_logger().warn(f'{sensor_name} reported error!')
            self.call_error_handler(sensor_name)
        self.update_robot_state()
            

    def call_error_handler(self, sensor_name):
        self.pending_error = sensor_name
        self.error_client = self.create_client(HandleSensorError, 'handle_sensor_error')
        # Timer będzie co 1 sekundę próbował ponownie, aż serwis się pojawi
        self.create_timer(1.0, self.try_call_error_handler)

    def try_call_error_handler(self):
        if hasattr(self, 'pending_error') and self.pending_error is not None:
            if self.error_client.wait_for_service(timeout_sec=0.1):
                req = HandleSensorError.Request()
                req.sensor_name = self.pending_error
                future = self.error_client.call_async(req)
                self.get_logger().info(f'Service available, calling error handler for {self.pending_error}')
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response is not None:
                    self.get_logger().info(f'ErrorHandler response: success={response.success}')
                self.pending_error = None


def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()