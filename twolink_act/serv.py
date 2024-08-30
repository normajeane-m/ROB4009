#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from twolink_act.action import Robotvalue
import time
import serial
import struct
import sys

class posServer(Node):

    def __init__(self):
        super().__init__('serv')
        self._action_server = ActionServer(self, Robotvalue, 'robotpos', self.execute_callback)
        self.ser = None
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600)
        except serial.SerialException as e:
            self.get_logger().error(f"Line 18 : Failed to connect to serial port: {e}")
            # sys.exit(1)
            
    def send_data_to_arduino(self, goal_handle):
        send = [b"S", goal_handle.request.xpos_des, goal_handle.request.ypos_des, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.get_logger().info(f"Send < f > to arduino...")
        try:
            for f in send:
                if isinstance(f, float):    
                    data = struct.pack('f', f)
                    self.ser.write(data)
                    # self.get_logger().info(f"Send < data > to arduino...")
                else:
                    self.ser.write(f)  
        except serial.SerialException as e:
            self.get_logger().error(f"Line 31 : Failed to connect to serial port: {e}")
            goal_handle.abort()
            # sys.exit(1)
            
    def receive_data_from_arduino(self,goal_handle):
        # self.get_logger().info(f"Try to receive data from arduino...")
        try:
            self.ser.write(b"f")
            # self.get_logger().info(f"Send < f > to arduino...")
            floats = []
            while self.ser.in_waiting < 32:
                pass
            for _ in range(8):
                data = self.ser.read(4)
                received_float = struct.unpack('f', data)[0]
                floats.append(round(received_float,5))
                # self.get_logger().info(f'Received float : {received_float}')
            return floats
        except serial.SerialException as e:
            self.get_logger().error(f"Line 46 : Failed to connect to serial port: {e}")
            goal_handle.abort()
            # sys.exit(1)
            
    def goal_achievement(self, goal_handle, ret_data):
        err = 0.001
        Xd = goal_handle.request.xpos_des
        Yd = goal_handle.request.ypos_des
        if(Xd - err <= ret_data[0] <= Xd + err & Yd - err <= ret_data[1] <= Yd + err):
            goal_handle.succeed()
        else:
            goal_handle.execute()
        # goal_handle.  
        
    def execute_callback(self, goal_handle):
        if self.ser is None:
            self.get_logger().error("Serial port is not initialized.")
            goal_handle.abort()
            return Robotvalue.Result(success=False)

        self.get_logger().info('Executing goal...')

        feedback_msg = Robotvalue.Feedback()

        self.send_data_to_arduino(goal_handle)
        ret_data = self.receive_data_from_arduino(goal_handle)
        
        if not ret_data:
            goal_handle.abort()
            self.get_logger().info('No return data...')
            return Robotvalue.Result(success=False)
        
        #goal값과 피드백 값 비교해서 goal_handle 값 반환.. test아직
        #아두이노에 값 입력 아직 안넣음 >> 작동테스트 완료
        #
        feedback_msg.xpos = ret_data[0]
        feedback_msg.ypos = ret_data[1]
        feedback_msg.xvel = ret_data[2]
        feedback_msg.yvel = ret_data[3]
        feedback_msg.qpos1 = ret_data[4]
        feedback_msg.qpos2 = ret_data[5]
        feedback_msg.qvel1 = ret_data[6]
        feedback_msg.qvel2 = ret_data[7]
        goal_handle.execute()
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'Feedback : xpos={feedback_msg.xpos}, ypos={feedback_msg.ypos}, xvel={feedback_msg.xvel}, yvel={feedback_msg.yvel}, qpos1={feedback_msg.qpos1}, qpos2={feedback_msg.qpos2}, qvel1={feedback_msg.qvel1}, qvel2={feedback_msg.qvel2}')
                                        
        time.sleep(0.5)

        # goal_handle.succeed()
        self.goal_achievement(goal_handle,ret_data)
        
        result = Robotvalue.Result()
        result.success = True
        return result

    def destroy_node(self): 
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    pos_server = posServer()

    try:
        rclpy.spin(pos_server)
    except KeyboardInterrupt:
        pass

    pos_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    