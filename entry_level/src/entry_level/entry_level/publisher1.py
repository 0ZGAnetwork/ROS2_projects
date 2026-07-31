import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Sensor1(Node):
    def __init__(self) -> None:
        super().__init__('sensor1')
        self.publisher_ = self.create_publisher(String, 'sensor1_velocity', 10)
        timer_period = 0.5  # seconds
        frequency = 1 / timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self) -> None:
        msg = String()
        msg.data = f'Velocity data from sensor1 {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    sensor1 = Sensor1()

    try:
        rclpy.spin(sensor1)

    except KeyboardInterrupt:
        sensor1.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        sensor1.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()