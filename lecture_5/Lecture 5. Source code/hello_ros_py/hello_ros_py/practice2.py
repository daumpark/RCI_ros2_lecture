import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_world_node')


def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()

    
    for i in range(100):
        node.get_logger().info('Hello World!')
    node.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()
    