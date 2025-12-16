from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time
from rcl_interfaces.msg import SetParametersResult

class RobotMonitorSim(Node):
    def __init__(self): # create node
        super().__init__('simulator')
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('angular_speed', 0.0)
        self.u1 = self.get_parameter('linear_speed').value
        self.u2 = self.get_parameter('angular_speed').value
        self.history_x = []
        self.history_y = []
        #
        self.declare_parameter('x',0.0)
        self.declare_parameter('y',0.0)
        self.declare_parameter('z',0.0)
        self.declare_parameter('theta',0.0)
        self.x = self.get_parameter('x').value # take parameter from terminal
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.theta = self.get_parameter('theta').value
        #
        self.sub = self.create_subscription(Twist, 'Twist', self.control_callback, 10)  # create sub, pub, etc
        self.t = self.get_clock().now().to_msg()
        self.pub_ = self.create_publisher(PoseStamped, 'PoseStamped', 10)
        self.timer = self.create_timer(0.1, self.state_update) # < --- timer
        self.last_time = self.get_clock().now()
        self.tf_broadcaster = TransformBroadcaster(self)    # for transform
        #
        self.add_on_set_parameters_callback(self.param_callback)

        # plot runtime:
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("t [m]")
        self.ax.set_title("Trajectory")
        self.ax.grid(True)
        self.ax.axis("equal")
    
    def my_ode(self, t, state): # prepare better name
        x, y, theta = state
        dxdt = self.u1 * math.cos(theta)
        dydt = self.u1 * math.sin(theta)
        dthetadt = self.u2
        return [dxdt, dydt, dthetadt]

    def control_callback(self, msg: Twist): # receiving msg from controller
        self.u1 = msg.linear.x
        self.u2 = msg.angular.z
        #self.get_logger().info(f'Receiving control linear {self.u1}, angular {self.u2}')

    def yaw_to_quaternion(self, yaw):   #yaw_to_quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def state_update(self): # equation and function for x, y, theta
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = now 
        # self.x += self.u1 * math.cos(self.theta) *dt
        # self.y += self.u1 * math.sin(self.theta) *dt
        # self.theta += self.u2 * dt

        ## ODE SOLVER
        state0 = [self.x, self.y, self.theta]
        sol = solve_ivp( self.my_ode, [0, dt], state0, method="RK45" )
        self.x, self.y, self.theta = sol.y[:, -1]
        self.history_x.append(self.x)
        self.history_y.append(self.y)

        PS = PoseStamped()
        PS.header.stamp = now.to_msg()
        PS.header.frame_id = 'world'

        px = self.x
        py = self.y
        PS.pose.position.x = px
        PS.pose.position.y = py
        PS.pose.position.z = 0.0

        # qz = math.sin(self.theta / 2.0)
        # qw = math.cos(self.theta / 2.0)
        qx, qy, qz, qw = self.yaw_to_quaternion(self.theta)

        PS.pose.orientation.x = qx
        PS.pose.orientation.y = qy
        PS.pose.orientation.z = qz
        PS.pose.orientation.w = qw

        self.pub_.publish(PS)
        
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0 
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        
        #self.get_logger().info(f'Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')
        #self.get_logger().warning(f'Orientation qx={qx}, qy={qy}, qz={qz}, qw={qw}')

        self.line.set_xdata(self.history_x)
        self.line.set_ydata(self.history_y)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    def param_callback(self, params):   # overwritte variables
        for param in params:
            if param.name == "x":
                self.x = param.value
            elif param.name == "y":
                self.y = param.value
            elif param.name == "z":
                self.z = param.value
            elif param.name == "theta":
                self.theta = param.value
        return SetParametersResult(successful=True)



def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        print(' exit :3')
        # plt.figure()
        # plt.plot(node.history_x, node.history_y)
        # plt.xlabel("x [m]")
        # plt.ylabel("y [m]")
        # plt.title("Trajectory- ODE solver")
        # plt.grid(True)
        # plt.axis('equal')
        # plt.show()
        plt.ioff()
        plt.show()
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
