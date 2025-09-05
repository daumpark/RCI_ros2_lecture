#include "rclcpp/rclcpp.hpp"

class HelloWorldNode : public rclcpp::Node
{
public:
    HelloWorldNode() : Node("hello_world_node")
    {
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloWorldNode>();
    RCLCPP_INFO(node->get_logger(), "Hello World!");
    rclcpp::shutdown();
}