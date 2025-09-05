#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ReverseNode : public rclcpp::Node
{
public:
    ReverseNode() : Node("reverse_node")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>
            ("turtle1/cmd_vel", 10, std::bind(&ReverseNode::listner_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel_reverse", 10);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    void listner_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist new_msg;

        new_msg.linear.x = -msg->linear.x;
        new_msg.linear.y = -msg->linear.y;
        new_msg.linear.z = -msg->linear.z;
        new_msg.angular.x = -msg->angular.x;
        new_msg.angular.y = -msg->angular.y;
        new_msg.angular.z = -msg->angular.z;

        publisher_->publish(new_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReverseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}