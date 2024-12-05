#!/usr/bin/env python3
import curses
import rclpy
from rclpy.node import Node
from twolink_msgs.msg import CMD
from geometry_msgs.msg import Point32

class PosPublisher(Node):
    def __init__(self):
        super().__init__('command_node') 
        # TODO

    def yolo_callback(self, msg):  
        # TODO
        print('TODO')
        
    def update_position(self, stdscr):
        stdscr.nodelay(True)
        self.display_controls(stdscr)

        def qclear(self):
            self.q1des = 0.0
            self.q2des = 0.0
        
        def xclear(self):
            self.xdes = 0.0
            self.ydes = 0.0
            
        while rclpy.ok():
            # stdscr.clear()
            key = stdscr.getch()
            if key == ord('e'):
                self.q1des += 0.1
                xclear(self)
            elif key == ord('d'):
                self.q1des -= 0.1
                xclear(self)
            elif key == ord('r'):
                self.q2des += 0.1
                xclear(self)
            elif key == ord('f'):
                self.q2des -= 0.1
                xclear(self)

            self.display_controls(stdscr)
            self.display_position(stdscr)
            stdscr.refresh()
            rclpy.spin_once(self, timeout_sec=10)

    def display_controls(self, stdscr):
        stdscr.addstr(0, 0, "=============Key Guide==+=========  ______  _____ ___  ___    _            ______ ")
        stdscr.addstr(1, 0, "|  DES  |  x  |  y  |  q1  |  q2 |  | ___ \/  __ \|  \/  |   | |           | ___ \\")
        stdscr.addstr(2, 0, "|_______|_____|_____|______|_____|  | |_/ /| /  \/| .  . |   | |      __ _ | |_/ /")
        stdscr.addstr(3, 0, "|   +   |     |     |  e   |  r  |  |    / | |    | |\/| |   | |     / _` || ___ \\")
        stdscr.addstr(4, 0, "|   -   |     |     |  d   |  f  |  | |\ \ | \__/\| |  | |   | |____| (_| || |_/ /")
        stdscr.addstr(5, 0, "|--------------------------------|  \_| \_| \____/\_|  |_/   \_____/ \__,_|\____/ ")
        stdscr.addstr(6, 0, "|       Exit - < Ctrl + C >      |")
        stdscr.addstr(7, 0, "|___________________ ____________|")
                      
    def display_position(self, stdscr):
        stdscr.addstr(8, 0,f'xpos={self.xdes:.4f}, ypos={self.ydes:.4f}, qpos1={self.q1des:.4f}, qpos2={self.q2des:.4f}')
   
    def publish_message(self):
        msg = CMD()
        msg.xdes = round(self.xdes,4)
        msg.ydes = round(self.ydes,4)
        msg.q1des = round(self.q1des,4)
        msg.q2des = round(self.q2des,4)
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    command_node = PosPublisher()

    try:
        curses.wrapper(command_node.update_position)
    except KeyboardInterrupt:
        pass

    command_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
