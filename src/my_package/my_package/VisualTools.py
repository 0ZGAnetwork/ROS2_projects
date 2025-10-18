import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt

class VisualTools(Node):
    def __init__(self):
        super().__init__('visualtools')
        self.subscription_ = self.create_subscription(Float32, 'signal', self.callback, 1)
        self.values = []
        plt.ion()
        self.fig, self.ax = plt.subplots()

    def callback(self, msg):
        self.values.append(msg.data)
        self.ax.clear()
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Amplitude')
        self.ax.grid(True)
        self.ax.plot(self.values[-120:],'r-',linewidth=2, label='Sin(t)')  # plot last 100 values
        self.ax.legend(loc='upper right')
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = VisualTools()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()