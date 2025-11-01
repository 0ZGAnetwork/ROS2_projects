import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import time

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
        self.timer = self.create_timer(0.1, self.update_plot)
        

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
            self.line_signal, = self.ax1.plot([], [], 'r-', linewidth=1.5, label='signal(t)')
            self.line_modified, = self.ax2.plot([], [], 'b-', linewidth=1.5, label='modified(t)')

            self.ax1.legend(loc='upper right')
            self.ax2.legend(loc='upper right')
            plt.tight_layout()
            plt.show()

    def callback_signal(self, msg):
        self.values_signal.append(msg.data)
        #self.update_plot()

    def callback_modified(self, msg):
        self.values_modified.append(msg.data)
        #self.update_plot()

        
    def update_plot(self):
        #self.ax1.clear()
        #self.ax2.clear()
        
        self.line_signal.set_data(range(len(self.values_signal[-50:])), self.values_signal[-50:])
        self.line_modified.set_data(range(len(self.values_modified[-50:])), self.values_modified[-50:])
        
        
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()

        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        #self.ax1.set_ylabel('Amplitude (signal)')
        #self.ax2.set_ylabel('Amplitude (modified)')
        #self.ax2.set_xlabel('Time')
        #self.ax1.grid(True)
        #self.ax2.grid(True)
        
        
        #plt.tight_layout()
        #plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = VisualTools()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()