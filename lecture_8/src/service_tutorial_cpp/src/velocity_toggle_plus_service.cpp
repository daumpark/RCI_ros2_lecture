#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_srv/srv/changerate.hpp"

class PubVelToggleRate : public rclcpp::Node
{
public:
    PubVelToggleRate()
        : Node("pubvel_toggle_rate"), forward_(true), frequency_(2.0), rate_changed_(false), cnt_(0)
    {
        toggle_service_ = this->create_service<std_srvs::srv::Empty>(
            "toggle_forward",
            std::bind(&PubVelToggleRate::toggle_forward_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        change_rate_service_ = this->create_service<my_srv::srv::Changerate>(
            "change_rate",
            std::bind(&PubVelToggleRate::change_rate_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "PubVelToggleRate node has been started.");
    }

    void loop()
    {
        rclcpp::Rate rate(rate_changed_ ? frequency_ : 2.0);
        while (rclcpp::ok())
        {
            publish_velocity();
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

private:
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_service_;
    rclcpp::Service<my_srv::srv::Changerate>::SharedPtr change_rate_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    bool forward_;
    double frequency_;
    bool rate_changed_;
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

    bool change_rate_callback(
        const std::shared_ptr<my_srv::srv::Changerate::Request> request,
        std::shared_ptr<my_srv::srv::Changerate::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Changing rate to %f", request->newrate);
        frequency_ = request->newrate;
        rate_changed_ = true;
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
    auto node = std::make_shared<PubVelToggleRate>();
    node->loop();
    rclcpp::shutdown();
    return 0;
}