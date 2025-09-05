#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <functional>
#include <random>

class RandomVelocityNode : public rclcpp::Node
{
public:
    RandomVelocityNode() : Node("random_velocity_node"), gen_(rd_()), dis_(-2.0, 2.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        this->declare_parameter("cmd_vel_rate", 3.0);
        double cmd_vel_rate = this->get_parameter("cmd_vel_rate").as_double();
        auto timer_period = std::chrono::duration<double>(1.0 / cmd_vel_rate);
        timer_ = this->create_wall_timer(timer_period, std::bind(&RandomVelocityNode::timer_callback, this));
    }
private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();

        msg.linear.x = dis_(gen_);
        msg.angular.z = dis_(gen_);

        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: %f, angular.z: %f", msg.linear.x, msg.angular.z);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomVelocityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}