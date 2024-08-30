#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from twolink_act.action import Robotvalue
import curses
import os 

class posClient(Node):

    def __init__(self):
        super().__init__('clnt')
        self._action_client = ActionClient(self, Robotvalue, 'robotpos')
        self.xpos_des = 0.0 #
        self.ypos_des = 0.0 #
        self.feedback_msg = None
    ###############################################
    def update_position(self, stdscr):
        stdscr.nodelay(True)
        self.display_controls(stdscr)

        while rclpy.ok():
            key = stdscr.getch()
            if key == ord('w'):
                self.ypos_des += 0.01
            elif key == ord('s'):
                self.ypos_des -= 0.01
            elif key == ord('a'):
                self.xpos_des -= 0.01
            elif key == ord('d'):
                self.xpos_des += 0.01

            # self.publish_message(stdscr) # << 여기 수정
            self.send_goal(self.xpos_des, self.ypos_des, stdscr)
            # os.system('clear') 
            self.display_controls(stdscr)
            self.display_position(stdscr)
            stdscr.refresh()
            rclpy.spin_once(self, timeout_sec=0.01)

    def display_controls(self, stdscr):
        stdscr.addstr(0, 0, "==========Key Guide========== ______  _____ ___  ___    _            ______ ")
        stdscr.addstr(1, 0, "|  'w' - Desire ypos +0.01  | | ___ \/  __ \|  \/  |   | |           | ___ \\")
        stdscr.addstr(2, 0, "|  's' - Desire ypos -0.01  | | |_/ /| /  \/| .  . |   | |      __ _ | |_/ /")
        stdscr.addstr(3, 0, "|  'a' - Desire xpos +0.01  | |    / | |    | |\/| |   | |     / _` || ___ \\")
        stdscr.addstr(4, 0, "|  'd' - Desire xpos -0.01  | | |\ \ | \__/\| |  | |   | |____| (_| || |_/ /")
        stdscr.addstr(5, 0, "|                           | \_| \_| \____/\_|  |_/   \_____/ \__,_|\____/ ")
        stdscr.addstr(6, 0, "|    Exit - < Ctrl + C >    |")

    def display_position(self, stdscr):
        stdscr.addstr(8, 0, f"Desire position: x={self.xpos_des:.2f}, y={self.ypos_des:.2f}")
        stdscr.addstr(9, 0,f'Feedback :')
        
        if self.feedback_msg:
            fb = self.feedback_msg.feedback
            stdscr.addstr(10, 0,f'xpos={fb.xpos:.4f}, ypos={fb.ypos:.4f}, xvel={fb.xvel:.4f}, yvel={fb.yvel:.4f}')
            stdscr.addstr(11, 0,f'qpos1={fb.qpos1:.4f}, qpos2={fb.qpos2:.4f}, qvel1={fb.qvel1:.4f}, qvel2={fb.qvel2:.4f}')            
        else:
            stdscr.addstr(10, 0,f'No Feedback data...')

    #####################################    

    def send_goal(self, xpos_des, ypos_des, stdscr):
        goal_msg = Robotvalue.Goal()
        goal_msg.xpos_des = xpos_des
        goal_msg.ypos_des = ypos_des

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
            # feedback_callback=lambda feedback_msg: self.feedback_callback(feedback_msg))
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        # if not goal_handle.accepted:
        #     self.get_logger().info('Goal rejected')
        #     return
        # self.get_logger().info('Goal accepted')
        if not goal_handle.accepted:
            # Display rejection message directly on screen
            stdscr = curses.initscr()
            stdscr.addstr(14, 0, 'Goal rejected')
            return
        else:
            stdscr = curses.initscr()
            stdscr.addstr(14, 0, 'Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.feedback_msg = feedback_msg
        # self.get_logger().info(f'Received feedback: xpos={feedback_msg.feedback.xpos}, ypos={feedback_msg.feedback.ypos}')
        # self.get_logger().info(f'\nFeedback : xpos={feedback_msg.feedback.xpos}, ypos={feedback_msg.feedback.ypos}, xvel={feedback_msg.feedback.xvel}, yvel={feedback_msg.feedback.yvel}, qpos1={feedback_msg.feedback.qpos1}, qpos2={feedback_msg.feedback.qpos2}, qvel1={feedback_msg.feedback.qvel1}, qvel2={feedback_msg.feedback.qvel2}')
        # stdscr.addstr(9, 0,f'Feedback : xpos={feedback_msg.feedback.xpos}, ypos={feedback_msg.feedback.ypos}, xvel={feedback_msg.feedback.xvel}, yvel={feedback_msg.feedback.yvel}, qpos1={feedback_msg.feedback.qpos1}, qpos2={feedback_msg.feedback.qpos2}, qvel1={feedback_msg.feedback.qvel1}, qvel2={feedback_msg.feedback.qvel2}')

                                        
    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info(f'Goal result: {result.success}')
        if result.success:
            # Display goal result on screen
            stdscr = curses.initscr()
            stdscr.addstr(15, 0, 'Goal result: Success')
        else:
            stdscr = curses.initscr()
            stdscr.addstr(15, 0, 'Goal result: Failed')
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    pos_client = posClient()

    try:
        curses.wrapper(pos_client.update_position)
    except KeyboardInterrupt:
        pass
    
    # rclpy.spin(pos_client)
    pos_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()