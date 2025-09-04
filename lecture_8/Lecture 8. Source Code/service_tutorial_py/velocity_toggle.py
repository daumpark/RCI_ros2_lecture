# Import necessary ROS2 libraries
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty  # Service type for simple requests
from geometry_msgs.msg import Twist  # Message type for velocity commands

# Define the class for the ROS2 node
class PubVelToggle(Node):

    def __init__(self):
        # Initialize the node with the name 'pubvel_toggle_server'
        super().__init__('pubvel_toggle_server')
        
        # Initialize state variables
        self.forward = True  # Flag to control forward movement
        self.cnt = 0  # Counter for service calls

        # Create a service that listens for requests on the '/toggle_forward' topic
        self.server_ = self.create_service(Empty, '/toggle_forward', self.toggle_forward_callback)
        
        # Create a publisher that sends velocity commands to the 'turtle1/cmd_vel' topic
        self.pub_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # Create a timer that calls the 'publish_velocity' function every 0.5 seconds
        self.timer_ = self.create_timer(0.5, self.publish_velocity)
        
        # Log a message to indicate that the node has started
        self.get_logger().info("Pubveltoogle now has been started")

    # This function is called when the service receives a request
    def toggle_forward_callback(self, request, response):
        # Increment the counter each time the service is called
        self.cnt += 1
        
        # Toggle the 'forward' flag
        self.forward = not self.forward
        
        # Log the current movement state
        self.get_logger().info(f"Now sending {'forward' if self.forward else 'rotate'} commands.")

        # Return the response
        return response
    
    # This function is called by the timer to publish velocity commands
    def publish_velocity(self):
        # Do nothing if the service has not been called yet
        if self.cnt == 0:
            return

        # Create a new Twist message
        msg = Twist()
        
        # Set the linear and angular velocities based on the 'forward' flag
        msg.linear.x = 1.0 if self.forward else 0.0
        msg.angular.z = 0.0 if self.forward else 1.0

        # Publish the message
        self.pub_.publish(msg)

# The main function where the ROS2 node is executed
def main(args=None):
    # Initialize the ROS2 client library
    rclpy.init(args=args)
    
    # Create an instance of the PubVelToggle node
    node = PubVelToggle()
    
    # Keep the node running until it's interrupted
    rclpy.spin(node)
    
    # Shutdown the ROS2 client library
    rclpy.shutdown()

# This is the entry point of the script
if __name__ == '__main__':
    main()
        
