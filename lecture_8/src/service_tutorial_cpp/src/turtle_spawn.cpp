#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

class SpawnTurtleNode : public rclcpp::Node
{
public:
    SpawnTurtleNode() : Node("spawn_turtle_client")
    {
        client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        request_ = std::make_shared<turtlesim::srv::Spawn::Request>();
    }

    void send_request()
    {
        request_->x = 5.0;
        request_->y = 2.0;
        request_->name = "new_turtle";

        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto result = client_->async_send_request(request_);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "Successfully spawned a new turtle: %s", response->name.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn a new turtle");
        }
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
    turtlesim::srv::Spawn::Request::SharedPtr request_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpawnTurtleNode>();
    node->send_request();
    rclcpp::shutdown();
    return 0;
}