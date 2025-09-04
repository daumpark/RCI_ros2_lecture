import time
import math
from pantilt_action_tutorials_interfaces.action import PantiltAction

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PantiltActionServer(Node):

    def __init__(self):
        super().__init__('pantilt_action_server')
        self._action_server = ActionServer(
            self,
            PantiltAction,
            'pantilt',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )
        self.joint_pub_ = self.create_publisher(JointState, "joint_states", 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        
        self.declare_parameter("mode", "REJECT")
        self.mode_ = self.get_parameter("mode").get_parameter_value().string_value

        self.joint_state_ = JointState()
        self.joint_state_.name = ['pan_joint', 'tilt_joint']
        self.joint_state_.position = [0.0, 0.0]
        
        self.abort_goal_ = False
        self.moving_ = False
        self.goal_handle_ = None

    def timer_callback(self):
        self.joint_state_.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub_.publish(self.joint_state_)

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: joint {goal_request.name} desired_angle {goal_request.desired_angle}')
        if self.moving_:
            self.get_logger().info(f'Server occupied MODE = {self.mode_}')
            if self.mode_ == "REJECT":
                self.get_logger().info('Server occupied REJECTING petition')
                return GoalResponse.REJECT
            else:  # self.mode_ == "ABORT"
                self.get_logger().info('Server occupied ABORTING current motion')
                self.abort_goal_ = True
                time.sleep(2)
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.moving_ = True
        self.goal_handle_ = goal_handle

        feedback_msg = PantiltAction.Feedback()
        result = PantiltAction.Result()

        if goal_handle.request.name == "pan_joint":
            joint_number = 0
            joint_name = "pan_joint"
        else:
            joint_number = 1
            joint_name = "tilt_joint"

        current_value = round(math.degrees(self.joint_state_.position[joint_number]))
        
        if current_value - goal_handle.request.desired_angle < 0:
            sign = 1
        else:
            sign = -1

        while rclpy.ok():
            if self.abort_goal_:
                self.get_logger().info('Aborting goal ++++++++++++')
                goal_handle.abort()
                self.abort_goal_ = False
                return result

            if abs(current_value - goal_handle.request.desired_angle) < goal_handle.request.stepsize:
                break

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.final_angle = current_value
                self.get_logger().info('Goal canceled')
                self.moving_ = False
                return result

            current_value += sign * goal_handle.request.stepsize
            if current_value < -180:
                current_value = 360 + current_value
            elif current_value > 180:
                current_value = -360 + current_value
            
            self.joint_state_.name[joint_number] = joint_name
            self.joint_state_.position[joint_number] = math.radians(current_value)

            feedback_msg.name = goal_handle.request.name
            feedback_msg.current_angle = current_value
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Publish feedback {feedback_msg.name}: {feedback_msg.current_angle} / {goal_handle.request.desired_angle}')

            time.sleep(0.1)

        if rclpy.ok():
            result.final_angle = current_value
            goal_handle.succeed()
            self.moving_ = False
            self.get_logger().info(f'Goal succeeded. Final angle is {current_value}')

        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        pantilt_action_server = PantiltActionServer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(pantilt_action_server)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            pantilt_action_server.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
