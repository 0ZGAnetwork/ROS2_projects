#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class WheelStatePublisher : public rclcpp::Node
{
 public:
    WheelStatePublisher(): 
        Node("wheel_state_publisher"){
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            // std::bind(&WheelStatePublisher::publish_joint_states, this)
            [this]() {
                publish_joint_states();
            }
        );
    }

    private:

        void publish_joint_states()
        {
            auto msg = sensor_msgs::msg::JointState();
        
            msg.header.stamp = this->get_clock()->now();
        
            msg.name = {
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "rear_left_wheel_joint",
                "rear_right_wheel_joint"
            };

            msg.position = {
                0.0,
                0.0,
                0.0,
                0.0
            };

            // optionaly
            msg.velocity = {};
            msg.effort = {};

            publisher_->publish(msg);

        }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WheelStatePublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}