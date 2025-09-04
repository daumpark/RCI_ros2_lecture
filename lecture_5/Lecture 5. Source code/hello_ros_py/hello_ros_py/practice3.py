import rclpy
from rclpy.node import Node

class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')
        self.i = 1
        self.timer =self.create_timer(1.0, self.timer_callback) # 1 second

    def timer_callback(self):
        self.get_logger().info(f"Counted to {self.i}")
        if self.i % 3 == 0:
            self.get_logger().info('is divisible by 3')
        if self.i % 5 == 0:
            self.get_logger().debug('is divisible by 5')
        if self.i % 7 == 0:
            self.get_logger().warning('is divisible by 7')
        if self.i % 11 == 0:
            self.get_logger().error('is divisible by 11')
        if self.i % 13 == 0:
            self.get_logger().fatal('is divisible by 13')

        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()
    


