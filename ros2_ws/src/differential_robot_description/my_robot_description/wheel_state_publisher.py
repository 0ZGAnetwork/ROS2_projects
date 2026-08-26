#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelStatePublisher(Node):

    def __init__(self):
        super().__init__('wheel_state_publisher')

        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.timer_ = self.create_timer(
            0.1,
            self.timer_handler
        )

    def timer_handler(self):

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]

        msg.position = [
            0.0,
            0.0,
            0.0,
            0.0
        ]

        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = WheelStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()