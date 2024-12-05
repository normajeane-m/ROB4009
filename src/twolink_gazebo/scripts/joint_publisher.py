#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from twolink_msgs.msg import XYpos

class JointPublisher(Node):
    """ Joint 상태와 Trajectory를 발행하는 ROS2 노드 """

    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # 초기화된 관절 위치 값
        self.position1_ = 0.0
        self.position2_ = 0.0

        # 'robot_state' 토픽 구독
        self.subscription = self.create_subscription(
            XYpos, 'robot_state', self.feedback_callback, 10
        )

        # 주기적으로 호출되는 타이머 설정
        self.trajectory_timer_ = self.create_timer(0.01, self.publish_joint_trajectory)
        self.state_timer_ = self.create_timer(0.01, self.publish_joint_state)

    def publish_joint_trajectory(self):
        """
        Joint Trajectory 메시지를 발행합니다.
        """
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = ['joint1', 'joint2']

        point = JointTrajectoryPoint()
        point.positions = [self.position1_, self.position2_]
        point.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg()

        joint_trajectory_msg.points.append(point)
        self.publisher_.publish(joint_trajectory_msg)

        self.get_logger().info(f'Published JointTrajectory: {joint_trajectory_msg}')

    def publish_joint_state(self):
        """
        Joint State 메시지를 발행합니다.
        """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2']
        joint_state_msg.position = [self.position1_, self.position2_]

        self.joint_state_publisher.publish(joint_state_msg)

        self.get_logger().info(f'Published JointState: {joint_state_msg}')

    def feedback_callback(self, msg):
        """
        'robot_state' 토픽으로부터 데이터를 수신하여 관절 위치를 업데이트합니다.
        """
        self.position1_ = msg.qpos1
        self.position2_ = msg.qpos2

        self.get_logger().info(f'Received feedback: qpos1={self.position1_}, qpos2={self.position2_}')

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
