import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt

class VisualTools(Node):
    def __init__(self):
        super().__init__('visualtools')
        self.subscription_ = self.create_subscription(
            Float32,
            'signal',
            self.callback_signal,
            10
        )
        self.subscription_modified_ = self.create_subscription(
            Float32,
            'modified_signal',
            self.callback_modified,
            10
        )
        self.values_signal = []
        self.values_modified = []

        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8,6), sharex=True)
        self.fig.suptitle('Signal vs Modified Signal')

        for ax in (self.ax1, self.ax2):
            ax.grid(True)

        self.ax1.set_ylabel('Amplitude (signal)')
        self.ax2.set_ylabel('Amplitude (modified)')
        self.ax2.set_xlabel('Time')

    def callback_signal(self, msg):
        self.values_signal.append(msg.data)
        self.update_plot()

    def callback_modified(self, msg):
        self.values_modified.append(msg.data)
        self.update_plot()
        
    def update_plot(self):
        self.ax1.clear()
        self.ax2.clear()
        
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax1.plot(self.values_signal[-120:],'r-',linewidth=1.5, label='signal(t)')  # plot last 100 values
        self.ax2.plot(self.values_modified[-120:],'b-',linewidth=1.5, label='modified(t)')  # plot last 100 values
        
        self.ax1.set_ylabel('Amplitude (signal)')
        self.ax2.set_ylabel('Amplitude (modified)')
        self.ax2.set_xlabel('Time')
        self.ax1.grid(True)
        self.ax2.grid(True)
        
        self.ax1.legend(loc='upper right')
        self.ax2.legend(loc='upper right')
        plt.tight_layout()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = VisualTools()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()