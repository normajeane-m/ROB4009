#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import threading
import signal

class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # 초기값 설정
        self.position1_ = 0.0
        self.position2_ = 0.0
        self.mutex = threading.Lock()

        # 'robot_feedback' 토픽을 구독하여 피드백 데이터를 가져옴
        self.subscription = self.create_subscription(
            String, 'robot_feedback', self.feedback_callback, 10)

        # 100ms마다 실행되는 타이머 설정
        self.timer_ = self.create_timer(0.01, self.publish_joint_trajectory)

    def publish_joint_trajectory(self):
        with self.mutex:
            msg = JointTrajectory()
            msg.joint_names = ['joint1', 'joint2']

            point = JointTrajectoryPoint()
            point.positions = [self.position1_, self.position2_]
            point.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg()
            msg.points.append(point)

            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {self.position1_}, {self.position2_}')

    def feedback_callback(self, msg):
        # 'robot_feedback' 토픽에서 qpos1과 qpos2 값을 파싱
        try:
            feedback_data = msg.data.split(',')
            qpos1 = float(feedback_data[4].split('=')[1])
            qpos2 = float(feedback_data[5].split('=')[1])

            with self.mutex:
                self.position1_ = qpos1
                self.position2_ = qpos2

            self.get_logger().info(f'Received feedback: qpos1={qpos1}, qpos2={qpos2}')
        except (IndexError, ValueError) as e:
            self.get_logger().warn(f"Error parsing feedback: {e}")

    def destroy_node(self):
        super().destroy_node()

def shutdown_handler(signum, frame, node):
    rclpy.logging.get_logger('joint_publisher').info('Shutting down...')
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()

    signal.signal(signal.SIGINT, lambda signum, frame: shutdown_handler(signum, frame, node))

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
