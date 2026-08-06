#include "iostream"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscriber : public rclcpp::Node
{
public: 
    Subscriber()
    : Node("velocity_sub")
    {
        auto topic_callback =
        [this](std_msgs::msg::String::UniquePtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        };
        subscription_ =
            this->create_subscription<std_msgs::msg::String>("velocity", 10, topic_callback);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<Subscriber>());
        rclcpp::shutdown();
    }
    catch (const std::exception & e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    
        rclcpp::shutdown();
        
        return 1;
    }
}