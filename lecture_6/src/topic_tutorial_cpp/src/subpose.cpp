#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "memory"

using std::placeholders::_1;
class SubPoseNode : public rclcpp::Node
{
public:
    SubPoseNode() : Node("subscribe_pose_node")
    {
        subscription_ = this->create_subscription<turtlesim::msg::Pose>
            ("turtle1/pose", 10, std::bind(&SubPoseNode::pose_callback, this, _1));
    }
private:
    void pose_callback(turtlesim::msg::Pose::SharedPtr msg)
    {
        double x = msg->x;
        double y = msg->y;
        double theta = msg->theta;

        RCLCPP_INFO(this->get_logger(), "Turtlebot Pose -X: %f, T: %f, Theta: %f", x, y, theta);
    }
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argv, char *argc[])
{
    rclcpp::init(argv, argc);
    auto node = std::make_shared<SubPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}