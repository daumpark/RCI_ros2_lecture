import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RandomVelocityNode(Node):
    def __init__(self):
        super().__init__('random_velocity_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.declare_parameter('cmd_vel_rate', 3.0)
        self.timer_period = 1.0 / self.get_parameter('cmd_vel_rate').get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = random.uniform(-2.0, 2.0)
        msg.angular.z = random.uniform(-2.0, 2.0)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "linear.x: %f, angular.z: %f"' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    random_velocity_node = RandomVelocityNode()
    rclpy.spin(random_velocity_node)
    random_velocity_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
