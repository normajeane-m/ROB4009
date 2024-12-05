#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from twolink_msgs.msg import XYpos
from twolink_msgs.msg import CMD
import serial
import struct
import sys 
'''
subscribing desire value (x, y, q1, q2)
publishing current robot state (x, y, xv, yv, q1, q2, q1v, q2v)
'''
class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_subscriber')
        self.pub_robot_state = self.create_publisher(XYpos, "robot_state", 10)
        self.subscription = self.create_subscription(CMD, 'des_value', self.listener_callback, 10)
        self.ser = None
        self.ret_data = None
        self.xdes = 0.0 
        self.ydes = 0.0
        self.q1des = 0.0
        self.q2des = 0.0
        pub_period = 0.001
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            Serflag = True
        except serial.SerialException as e:
            self.get_logger().error(f"Line 18 : Failed to connect to serial port: {e}")
            sys.exit(1)
        if(Serflag):
           self.timer = self.create_timer(pub_period, self.publish_robot_state) 

    def Serial_Task(self): #Serial task with arduino
        send = [b"f", self.xdes, self.ydes, 0.0, 0.0, self.q1des, self.q2des, 0.0, 0.0]
        self.get_logger().info(f"Serial task with arduino...")
        try:
            floats = []
            for f in send:
                # self.get_logger().info(f"type of {f}: {type(f)}")
                if isinstance(f, float):    
                    data = struct.pack('f', f)
                    self.ser.write(data)
                else:
                    self.ser.write(f)   
            #Receive data from arduino        
            while self.ser.in_waiting < 32: 
                pass
            for _ in range(8):
                data = self.ser.read(4)
                received_float = struct.unpack('f', data)[0]
                floats.append(round(received_float,5))
            self.get_logger().info(f'Received float : {floats}')
            return floats        
        
        except serial.SerialException as e:
            self.get_logger().error(f"Line 39 : Failed to connect to serial port: {e}")
            sys.exit(1)

    def listener_callback(self, cmd):
        self.xdes = round(cmd.xdes,4)
        self.ydes = round(cmd.ydes,4)
        self.q1des = round(cmd.q1des,4)
        self.q2des = round(cmd.q2des,4)

    def publish_robot_state(self):
        self.ret_data = self.Serial_Task()
        if not self.ret_data:
            self.get_logger().info('No return data...')
            return
    
        msg = XYpos()
        msg.xpos = self.ret_data[0]
        msg.ypos = self.ret_data[1]
        msg.xvel = self.ret_data[2]
        msg.yvel = self.ret_data[3]
        msg.qpos1 = self.ret_data[4]
        msg.qpos2 = self.ret_data[5]
        msg.qvel1 = self.ret_data[6]
        msg.qvel2 = self.ret_data[7]
        self.pub_robot_state.publish(msg)
        return None

    def destroy_node(self): 
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_subscriber = SerialSubscriber()
    try:
        rclpy.spin(serial_subscriber)
    except KeyboardInterrupt:
        pass

    serial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
