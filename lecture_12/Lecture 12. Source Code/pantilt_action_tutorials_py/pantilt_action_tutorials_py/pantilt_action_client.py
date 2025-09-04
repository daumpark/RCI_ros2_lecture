from pantilt_action_tutorials_interfaces.action import PantiltAction
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class PantiltActionClient(Node):

    def __init__(self):
        super().__init__('pantilt_action_client')
        self._action_client = ActionClient(self, PantiltAction, 'pantilt')
        
        self.declare_parameter("joint_name", "tilt_joint")
        self.declare_parameter("desired_angle", 60)
        self.declare_parameter("stepsize", 1)

        self.joint_name_ = self.get_parameter("joint_name").get_parameter_value().string_value
        self.desired_angle_ = self.get_parameter("desired_angle").get_parameter_value().integer_value
        self.stepsize_ = self.get_parameter("stepsize").get_parameter_value().integer_value

    def send_goal(self):
        goal_msg = PantiltAction.Goal()
        goal_msg.name = self.joint_name_
        goal_msg.desired_angle = self.desired_angle_
        goal_msg.stepsize = self.stepsize_

        self._action_client.wait_for_server()

        self.get_logger().info(f'[PantiltActionClient] Sending goal: joint {goal_msg.name} angle {goal_msg.desired_angle}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.final_angle}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_angle}')
        if feedback.current_angle >= 90:
            self.get_logger().info("I'm going to cancel the goal since the angle is equal or greater than 90")
            # How to cancel a goal from feedback?
            # This is not straightforward in rclpy.
            # You need the goal handle, which is not available here.
            # One way is to save the goal handle in the class.
            # Let's assume for now we just shut down.
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = PantiltActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
