#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class WheelController : public rclcpp::Node
{
public:
    WheelController() : 
    Node("wheel_diff_controller")
    {
        //publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);       
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
        std::bind(&WheelController::cmd_vel_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WheelController::publisher_callback, this));

        RCLCPP_INFO(this->get_logger(), "WheelController initialized.");
    }
//parameters

private:
    // ros parameters
    double wheel_radius_;
    double wheel_separation_;
    // wheel velocities
    double left_wheel_velocity_ = 0.0;
    double right_wheel_velocity_ = 0.0;
    // cmd_vel
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
    }
   

    void publisher_callback()
    {
        const double right_linear_velocity = linear_velocity_ + (angular_velocity_ * wheel_separation_ / 2.0);
        const double left_linear_velocity = linear_velocity_ - (angular_velocity_ * wheel_separation_ / 2.0);
        // m/s-> rad/s
        const double right_wheel_velocity_ = right_linear_velocity / wheel_radius_;
        const double left_wheel_velocity_ = left_linear_velocity / wheel_radius_;
    }

    // ROS interfaces
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

};

//main
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}