#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # 초기값 설정
        self.position1_ = 0.0
        self.position2_ = 0.0

        # 'robot_feedback' 토픽을 구독하여 피드백 데이터를 가져옴
        self.subscription = self.create_subscription(
            String, 'robot_feedback', self.feedback_callback, 10)

        # 100ms마다 실행되는 타이머 설정
        self.timer_ = self.create_timer(0.01, self.publish_joint_trajectory)

    def publish_joint_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2']

        point = JointTrajectoryPoint()
        point.positions = [self.position1_, self.position2_]
        point.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg()
        msg.points.append(point)

        self.publisher_.publish(msg)
            
        # JointState 메시지 생성
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2']
        joint_state_msg.position = [self.position1_, self.position2_]

        # JointState 발행
        self.joint_state_publisher.publish(joint_state_msg)

        self.get_logger().info(f'Publishing: {self.position1_}, {self.position2_}')

    def feedback_callback(self, msg):
        # 'robot_feedback' 토픽에서 qpos1과 qpos2 값을 파싱
        feedback_data = msg.data.split(',')
        qpos1 = float(feedback_data[4].split('=')[1])
        qpos2 = float(feedback_data[5].split('=')[1])

        self.position1_ = qpos1
        self.position2_ = qpos2

        self.get_logger().info(f'Received feedback: qpos1={qpos1}, qpos2={qpos2}')

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
