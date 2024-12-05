#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from twolink_msgs.msg import XYpos  # twolink_msgs의 XYpos 메시지 가져오기
import math

class DummyDataPublisher(Node):

    def __init__(self):
        super().__init__('dummy_data_publisher')
        # 'robot_state' 토픽에 데이터를 퍼블리싱하는 퍼블리셔 생성
        self.publisher_ = self.create_publisher(XYpos, 'robot_state', 10)
        self.qpos1 = -6.0  # 초기값
        self.qpos2 = 6.0   # 초기값
        self.vel1 = 0.0
        self.vel2 = 0.0
        self.timer_period = 0.1  # 0.1초마다 데이터 퍼블리싱
        self.timer = self.create_timer(self.timer_period, self.publish_dummy_data)

    def publish_dummy_data(self):
        # qpos1과 qpos2의 값을 변경
        self.vel1 += 0.2
        self.qpos1 = math.sin(self.vel1)
        self.vel2 += 0.1
        self.qpos2 = math.sin(self.vel2)

        # XYpos 메시지 생성 및 데이터 설정
        msg = XYpos()
        msg.xpos = 0.0
        msg.ypos = 0.0
        msg.xvel = 0.0
        msg.yvel = 0.0
        msg.qpos1 = self.qpos1
        msg.qpos2 = self.qpos2
        msg.qvel1 = 0.0
        msg.qvel2 = 0.0

        # 데이터 퍼블리싱
        self.publisher_.publish(msg)

        # 로그에 출력
        self.get_logger().info(f'Publishing: qpos1={round(self.qpos1, 3)}, qpos2={round(self.qpos2, 3)}')

def main(args=None):
    rclpy.init(args=args)

    dummy_data_publisher = DummyDataPublisher()

    try:
        rclpy.spin(dummy_data_publisher)
    except KeyboardInterrupt:
        pass

    dummy_data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
