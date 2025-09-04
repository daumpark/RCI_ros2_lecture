import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ReverseNode(Node):
    def __init__(self):
        super().__init__('reverse_node')
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel_reverse', 10)

    def listener_callback(self, msg):
        new_msg = Twist()
        new_msg.linear.x = -msg.linear.x
        new_msg.linear.y = -msg.linear.y
        new_msg.linear.z = -msg.linear.z
        new_msg.angular.x = -msg.angular.x
        new_msg.angular.y = -msg.angular.y
        new_msg.angular.z = -msg.angular.z
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    reverse_node = ReverseNode()
    rclpy.spin(reverse_node)
    reverse_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
