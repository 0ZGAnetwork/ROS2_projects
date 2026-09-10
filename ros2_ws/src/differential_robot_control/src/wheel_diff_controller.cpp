#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

class WheelController : public rclcpp::Node
{
public:
    WheelController() : 
    Node("wheel_diff_controller")
    {
        wheel_radius_ = this->declare_parameter<double>("wheel_radius");

        wheel_separation_ = this->declare_parameter<double>("wheel_separation");

        //instance
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);       
        odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
            std::bind(&WheelController::cmd_vel_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WheelController::publisher_callback, this));

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "WheelController initialized.");
    }
//parameters

private:
    // robot positions
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    // ros parameters
    double wheel_radius_;
    double wheel_separation_;
    // wheel velocities
    double left_wheel_velocity_ = 0.0;
    double right_wheel_velocity_ = 0.0;
    // cmd_vel
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    double front_left_wheel_position_ = 0.0;
    double front_right_wheel_position_ = 0.0;
    double rear_left_wheel_position_ = 0.0;
    double rear_right_wheel_position_ = 0.0;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
    }
   

    void publisher_callback()
    {
        auto current_time = this->get_clock()->now();
        // calculate actual dt
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // equations for dfferential drive robot
        const double right_linear_velocity = linear_velocity_ + (angular_velocity_ * wheel_separation_ / 2.0);
        const double left_linear_velocity = linear_velocity_ - (angular_velocity_ * wheel_separation_ / 2.0);
        // m/s-> rad/s
        right_wheel_velocity_ = right_linear_velocity / wheel_radius_;
        left_wheel_velocity_ = left_linear_velocity / wheel_radius_;

        //integrate wheel positions
        front_left_wheel_position_ += left_wheel_velocity_ * dt;
        front_right_wheel_position_ += right_wheel_velocity_ * dt;
        rear_left_wheel_position_ += left_wheel_velocity_ * dt;
        rear_right_wheel_position_ += right_wheel_velocity_ * dt;

        
        //publish joint states
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();
        msg.name = {
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint"
        };
        msg.position = {
            front_left_wheel_position_,
            front_right_wheel_position_,
            rear_left_wheel_position_,
            rear_right_wheel_position_
        };
        publisher_->publish(msg);
        
        //update position (odometry)
        update_odometry(dt);
        //publish odometry
        publish_odometry(current_time);
        //publish transform for Rviz2
        publish_tf(current_time);
    }

    void update_odometry(double dt)
    {
        x_ += linear_velocity_ *  std::cos(theta_) * dt;
        y_ += linear_velocity_ *  std::sin(theta_) * dt;
        theta_ += angular_velocity_ * dt;
    }

    void publish_odometry(const rclcpp::Time & current_time)
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // convert theta to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = linear_velocity_;
        odom_msg.twist.twist.angular.z = angular_velocity_;

        odom_ -> publish(odom_msg);
    }

    void publish_tf(const rclcpp::Time & current_time)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

    // ROS members, remember to initialize them in the constructor
    rclcpp::Time last_time_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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