import rclpy
import time
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlesimPoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtlesim_pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def pose_callback(self, msg):
        self.get_logger().info(f'Pose: x={msg.x}, y={msg.y}, theta={msg.theta}')

def main(args=None):
    rclpy.init(args=args)
    turtlesim_pose_subscriber = TurtlesimPoseSubscriber()
    
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 3.0:
        rclpy.spin_once(turtlesim_pose_subscriber, timeout_sec=0.1)

    turtlesim_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
