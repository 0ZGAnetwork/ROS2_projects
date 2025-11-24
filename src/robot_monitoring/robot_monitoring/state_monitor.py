import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
from rcl_interfaces.msg import SetParametersResult
from robot_interfaces.srv import HandleSensorError

class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')
        self.sensors = ['sensor1', 'sensor2']
        self.last_msg_time = {name: 0.0 for name in self.sensors}

        for s in self.sensors:
            topic = f"/{s}/state"
            self.create_subscription(Bool, topic, lambda msg, sensor=s: self.sensor_callback(msg, sensor), 10)
            self.get_logger().info(f'Subscribed to {topic}')

        
        self.declare_parameter('check_interval', 5.0) 
        self.check_interval = self.get_parameter('check_interval').value
        self.get_logger().info(f'Check interval: {self.check_interval}s')
        #
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.timer = self.create_timer(self.check_interval, self.check_sensors)
        #
        #dictionary
        self.last_sensor_value = {name: True for name in self.sensors}

        self.error_client = self.create_client(HandleSensorError, 'handling_error')
        self.get_logger().info('Waiting for error handling service...')
        self.error_client.wait_for_service()
        self.get_logger().info('Error handling service available.')

    def sensor_callback(self, msg, sensor_name):
        self.last_msg_time[sensor_name] = time.time()
        self.last_sensor_value[sensor_name] = msg.data #store 
        self.get_logger().info(f'Received state from {sensor_name}: {msg.data}')

    def check_sensors(self):
        now = time.time()
        for sensor, last_time in self.last_msg_time.items():
            if last_time == 0.0:
                self.get_logger().warning(f"{sensor} has not sent any message yet")
                continue
            if now - last_time > self.check_interval:
                self.get_logger().error(f"{sensor} is NOT responding (last msg {now - last_time:.1f}s ago)")
            elif not self.last_sensor_value[sensor]:
                self.get_logger().error(f"{sensor} reported ERROR")
                self.call_error_handler(sensor)
            else:
                self.get_logger().info(f"{sensor} is OK (last msg {now - last_time:.1f}s ago)")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'check_interval' and isinstance(param.value, float):
                self.check_interval = float(param.value)
                self.get_logger().info(f'Updated check interval to {self.check_interval}s')
                self.timer.cancel()
                self.timer = self.create_timer(self.check_interval, self.check_sensors)
        return SetParametersResult(successful=True)

    def call_error_handler(self, sensor_name):
        req = HandleSensorError.Request()
        req.sensor_name = sensor_name
        future = self.error_client.call_async(req)

        future.add_done_callback( lambda f, s=sensor_name: self.get_logger().info(f"Error response for {s}: {f.result().message}"))
        self.get_logger().warning(f"Handlinnng error for {sensor_name}...")

def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
