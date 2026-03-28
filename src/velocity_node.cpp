#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ConstantVelocityPublisher : public rclcpp::Node
{
public:
  ConstantVelocityPublisher()
  : Node("constant_velocity_publisher")
  {
    // 1. Declare the parameter "velocity_topic" with a default value of "cmd_vel"
    this->declare_parameter<std::string>("velocity_topic", "cmd_vel");

    // 2. Get the value of the parameter
    std::string topic_name = this->get_parameter("velocity_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Publishing to topic: '%s'", topic_name.c_str());

    // 3. Create the publisher using the dynamic topic name
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
    
    // Create a timer to trigger the callback 10 times a second (100ms)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ConstantVelocityPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    
    // Set constant velocity values
    message.linear.x = 0.5;   
    message.linear.y = 0.0;
    message.linear.z = 0.0;
    
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = 0.0;  

    // Publish the message
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConstantVelocityPublisher>());
  rclcpp::shutdown();
  return 0;
}