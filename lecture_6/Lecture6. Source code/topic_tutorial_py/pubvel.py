# rclpy는 ROS2의 파이썬 클라이언트 라이브러리입니다.
import rclpy
# numpy는 과학 계산을 위한 파이썬 라이브러리입니다. 여기서는 무작위 값을 생성하기 위해 사용됩니다.
import numpy as np
# rclpy.node의 Node 클래스를 가져옵니다. ROS2에서 노드를 만들 때 필요합니다.
from rclpy.node import Node
# geometry_msgs.msg의 Twist 메시지 타입을 가져옵니다. 이는 로봇의 선속도와 각속도를 나타냅니다.
from geometry_msgs.msg import Twist

# TurtlesimVelNode라는 이름의 클래스를 만들고, rclpy.node.Node를 상속받습니다.
class TurtlesimVelNode(Node):
    # 클래스의 생성자(초기화) 함수입니다.
    def __init__(self):
        # 부모 클래스인 Node의 생성자를 호출하고, 노드 이름을 "turtlesim_pub_vel_node"로 설정합니다.
        super().__init__("turtlesim_pub_vel_node")
        # "turtle1/cmd_vel" 토픽에 Twist 메시지를 발행하는 퍼블리셔를 생성합니다. 큐 크기는 10입니다.
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        # 발행할 Twist 메시지를 초기화합니다.
        self.msgs_ = Twist()
        # 1초마다 timer_callback 함수를 호출하는 타이머를 생성합니다.
        self.timer_ = self.create_timer(1., self.timer_callback)

    # 타이머에 의해 주기적으로 호출되는 콜백 함수입니다.
    def timer_callback(self):
        # -2.5에서 2.5 사이의 무작위 선속도(x축)를 설정합니다.
        self.msgs_.linear.x = 5*(np.random.rand() - 0.5) # -0.5~0.5
        # -1에서 1 사이의 무작위 각속도(z축)를 설정합니다.
        self.msgs_.angular.z = 2*np.random.rand() - 1 # -1 ~ 1
        # 설정된 속도 값을 담은 Twist 메시지를 발행합니다.
        self.publisher_.publish(self.msgs_)

        # 현재 발행하는 속도 정보를 로그에 출력합니다.
        self.get_logger().info(f"Twist Message - Linear X: {self.msgs_.linear.x}, Angular Z: {self.msgs_.angular.z}")

# 스크립트의 메인 함수입니다.
def main(args=None):
    # rclpy를 초기화합니다.
    rclpy.init(args=args)
    # TurtlesimVelNode 클래스의 인스턴스를 생성합니다.
    pub_vel_node = TurtlesimVelNode()
    # 노드가 종료될 때까지 계속 실행되도록 합니다. 콜백 함수들이 이 사이클 안에서 호출됩니다.
    rclpy.spin(pub_vel_node)
    # 노드를 소멸시킵니다.
    pub_vel_node.destroy_node()
    # rclpy를 종료합니다.
    rclpy.shutdown()

# 이 스크립트가 직접 실행될 때 main 함수를 호출합니다.
if __name__ =='__main__':
    main()