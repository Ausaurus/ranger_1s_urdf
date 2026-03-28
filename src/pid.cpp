#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

class AngularPidNode : public rclcpp::Node
{
public:
    AngularPidNode() : Node("angular_pid_node")
    {
        // Declare and initialize PID parameters
        this->declare_parameter("kp", 1.0);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 0.1);
        this->declare_parameter("max_integral", 10.0); // Anti-windup limit
        this->declare_parameter("velocity_topic", "/cmd_vel");

        // Initialize state variables
        integral_error_ = 0.0;
        previous_error_ = 0.0;
        has_previous_time_ = false;

        // Create publisher for angular velocity (typically cmd_vel)
        std::string topic_name = this->get_parameter("velocity_topic").as_string();
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);

        // Create subscriber for angle error
        error_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "lookahead_angle", 10,
            std::bind(&AngularPidNode::errorCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Angular PID Node initialized.");
    }

private:
    void errorCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double current_error = msg->data;

        // Fetch current parameters in case they were updated dynamically
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double kd = this->get_parameter("kd").as_double();
        double max_integral = this->get_parameter("max_integral").as_double();

        double dt = 0.0;
        if (has_previous_time_) {
            dt = (current_time - previous_time_).seconds();
        }

        // Avoid division by zero on the first pass or if messages arrive instantly
        if (dt <= 0.0) {
            previous_time_ = current_time;
            has_previous_time_ = true;
            return;
        }

        // Proportional term
        double p_term = kp * current_error;

        // Integral term with anti-windup (clamping)
        integral_error_ += current_error * dt;
        if (integral_error_ > max_integral) integral_error_ = max_integral;
        if (integral_error_ < -max_integral) integral_error_ = -max_integral;
        double i_term = ki * integral_error_;

        // Derivative term
        double derivative = (current_error - previous_error_) / dt;
        double d_term = kd * derivative;

        // Calculate total control output (angular velocity)
        double angular_velocity = p_term + i_term + d_term;

        // Publish the output as a Twist message (rotating around the Z axis)
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = angular_velocity;
        twist_msg.linear.x = 1.5;
        velocity_pub_->publish(twist_msg);

        // Update state variables for the next iteration
        previous_error_ = current_error;
        previous_time_ = current_time;
    }

    // ROS 2 Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_sub_;

    // State memory
    double integral_error_;
    double previous_error_;
    rclcpp::Time previous_time_;
    bool has_previous_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngularPidNode>());
    rclcpp::shutdown();
    return 0;
}