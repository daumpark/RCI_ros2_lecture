import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class DirectionChecker(Node):

    def __init__(self):
        super().__init__('direction_checker')
        self.subscription = self.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Bool, 'direction_status', 10)

    def listener_callback(self, msg):
        bool_msg = Bool()
        if msg.linear.x < 0:
            bool_msg.data = False
            self.get_logger().info('Publishing: "False"')
        else:
            bool_msg.data = True
            self.get_logger().info('Publishing: "True"')
        self.publisher_.publish(bool_msg)

def main(args=None):
    rclpy.init(args=args)
    direction_checker = DirectionChecker()
    rclpy.spin(direction_checker)
    direction_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
