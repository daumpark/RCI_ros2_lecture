#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PubVelToggle : public rclcpp::Node
{
public:
    PubVelToggle() : Node("pubvel_toggle"), forward_(true), cnt_(0)
    {
        server_ = this->create_service<std_srvs::srv::Empty>(
            "toggle_forward",
            std::bind(&PubVelToggle::toggle_forward_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PubVelToggle::publish_velocity, this)
        );

        RCLCPP_INFO(this->get_logger(), "PubVelToggle node has been started.");
    }

private:
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool forward_;
    int cnt_;

    bool toggle_forward_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        cnt_++;
        forward_ = !forward_;
        RCLCPP_INFO(this->get_logger(), "Now sending %s commands.", forward_ ? "forward" : "rotate");
        return true;
    }

    void publish_velocity()
    {
        if (cnt_ == 0){
            return;
        }

        auto msg = std::make_shared<geometry_msgs::msg::Twist>();

        msg->linear.x = forward_ ? 1.0 : 0.0;
        msg->angular.z = forward_ ? 0.0 : 1.0;

        pub_->publish(*msg);

        RCLCPP_INFO(this->get_logger(), "Published velocity command - linear.x: %.2f, angular.z: %.2f", msg->linear.x, msg->angular.z);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubVelToggle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}