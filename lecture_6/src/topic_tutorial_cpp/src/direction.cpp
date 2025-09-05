#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DirectionCheck : public rclcpp::Node
{
public:
    DirectionCheck() : Node("check_direction_node")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel", 10, std::bind(&DirectionCheck::twistCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("linear_x_sign", 10);

        RCLCPP_INFO(this->get_logger(), "DirectionCheck node has been started.");
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std_msgs::msg::Bool bool_msg;
        bool_msg.data = (msg->linear.x >= 0);

        if (bool_msg.data)
        {
            RCLCPP_INFO(this->get_logger(), "Bool Linear_x: True");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Bool Linear_x: False");
        }

        publisher_->publish(bool_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionCheck>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}