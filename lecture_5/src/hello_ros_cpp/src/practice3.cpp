#include "rclcpp/rclcpp.hpp"

class LoggingNode: public rclcpp::Node
{
public:
    LoggingNode(): Node("logging_node"), i_(1)
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LoggingNode::timer_callback, this));
    }
private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Counted to %d", i_);
        if (i_ % 3 == 0)
            RCLCPP_INFO(this->get_logger(), "is divisible by 3.");
        else if (i_ % 5 == 0)
            RCLCPP_DEBUG(this->get_logger(), "is divisible by 5.");
        else if (i_ % 7 == 0)
            RCLCPP_WARN(this->get_logger(), "is divisible by 7.");
        else if (i_ % 11 == 0)
            RCLCPP_ERROR(this->get_logger(), "is divisible by 11.");
        else if (i_ % 13 == 0)
            RCLCPP_FATAL(this->get_logger(), "is divisible by 13.");
        i_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int i_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoggingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}