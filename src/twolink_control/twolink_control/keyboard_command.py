#!/usr/bin/env python3
import curses
import rclpy
from rclpy.node import Node
from twolink_msgs.msg import CMD, XYpos
from geometry_msgs.msg import Point32

class PosPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_command')
        self.publisher = self.create_publisher(CMD, 'des_value', 10)
        self.q1_curr = 0.0
        self.q2_curr = 0.0
        self.delta = 0.0
        self.q1des = None
        self.q2des = None
        
        
        
        
        
    def update_position(self, stdscr):
        stdscr.nodelay(True)
        while rclpy.ok():
            key = stdscr.getch()
            if key == ord('e'):
                self.q1des = self.q1_curr + self.delta
            elif key == ord('d'):
                self.q1des = self.q1_curr - self.delta
            elif key == ord('r'):
                self.q2des = self.q2_curr + self.delta
            elif key == ord('f'):
                self.q2des = self.q2_curr - self.delta
            self.publish_message()
            self.display_position(stdscr)
            stdscr.refresh()
            rclpy.spin_once(self, timeout_sec=0.01)

    def display_position(self, stdscr):
        stdscr.clear()
        stdscr.addstr(0, 0, f"Current q1={self.q1_curr:.4f}, q2={self.q2_curr:.4f}")
        q1des_str = f"{self.q1des:.4f}" if self.q1des is not None else "----"
        q2des_str = f"{self.q2des:.4f}" if self.q2des is not None else "----"
        stdscr.addstr(1, 0, f"Desired q1={q1des_str}, q2={q2des_str}")
        stdscr.addstr(3, 0, "Controls: e(+q1), d(-q1), r(+q2), f(-q2)")

    def publish_message(self):
        if self.q1des is None or self.q2des is None:
            # 아직 상태를 받지 못한 경우 publish를 건너뛴다
            return
        msg = CMD()
        msg.xdes = 0.0
        msg.ydes = 0.0
        msg.q1des = self.q1des
        msg.q2des = self.q2des
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PosPublisher()
    try:
        curses.wrapper(node.update_position)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()