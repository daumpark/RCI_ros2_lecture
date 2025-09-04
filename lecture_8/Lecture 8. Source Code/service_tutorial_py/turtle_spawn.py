import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class SpawnTurtleNode(Node):
    def __init__(self):
        super().__init__('spawn_turtle_client')
        self.client_ = self.create_client(Spawn, '/spawn')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available')

        self.request_ = Spawn.Request()

    def send_request(self):
        self.request_.x = 1.0
        self.request_.y = 1.0
        self.request_.theta = -1.0

        future = self.client_.call_async(self.request_)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Successfully spawned a new turtle: {future.result().name}")
        else:
            self.get_logger().error("Failed")

def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtleNode()
    node.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
