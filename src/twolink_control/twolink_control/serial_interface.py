#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from twolink_msgs.msg import XYpos, CMD
import serial
import struct
import sys
import threading


class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_interface')
        self.pub_robot_state = self.create_publisher(XYpos, "robot_state", 10)
        self.subscription = self.create_subscription(
            CMD, 'des_value', self.listener_callback, 10)

        # 명령 값
        self.xdes = 0.0
        self.ydes = 0.0
        self.q1des = 0.0
        self.q2des = 0.0

        # 수신된 로봇 상태 (RX 스레드가 갱신)
        self.last_state = None
        self.state_lock = threading.Lock()

        pub_period = 0.001  # 1000 Hz

        try:
            # 아두이노와 반드시 동일한 baudrate로 맞추기 (115200 추천)
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
            # self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.01)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            sys.exit(1)

        # RX 스레드 시작
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # 타이머: 주기적으로 TX + publish
        self.timer = self.create_timer(pub_period, self.tx_and_publish)
    # ---------------- TX + Publish ----------------
    def tx_and_publish(self):
        # 1) 명령 패킷 송신
        try:
            packet = self.build_packet()
            # print packet bytes before sending for easier debugging
            self.get_logger().info(f"TX packet: {packet.hex(' ')}")
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"TX error: {e}")
            return

    # ---------------- CRC / Packet ----------------
    def crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def build_packet(self) -> bytes:
        # payload: 8 floats
        payload = struct.pack(
            '<8f',
            self.xdes, self.ydes, 0.0, 0.0,
            self.q1des, self.q2des, 0.0, 0.0
        )
        crc = self.crc16(payload)
        packet = bytes([0xAA, len(payload)]) + payload + struct.pack('<H', crc)
        return packet

    def parse_packet(self, data: bytes):
        # data: 36 bytes 예상
        if len(data) != 36:
            return None

        if data[0] != 0xAA:
            return None

        length = data[1]
        if length != 32:
            return None

        payload = data[2:34]
        recv_crc = struct.unpack('<H', data[34:36])[0]
        calc_crc = self.crc16(payload)
        if calc_crc != recv_crc:
            return None

        # 8 floats
        return struct.unpack('<8f', payload)

    # ---------------- RX 스레드 ----------------
    def rx_loop(self):
        """
        시리얼 수신 전용 스레드:
        - 계속해서 바이트 스트림을 읽고
        - STX(0xAA) 기준으로 36바이트 패킷을 파싱
        - 유효하면 last_state에 저장
        """
        buf = bytearray()

        while rclpy.ok():
            try:
                # 사용 가능한 만큼 읽기
                n = self.ser.in_waiting
                if n > 0:
                    chunk = self.ser.read(n)
                    buf.extend(chunk)

                # 패킷 파싱 루프
                while len(buf) >= 36:
                    # STX 맞출 때까지 앞에서 버림
                    if buf[0] != 0xAA:
                        buf.pop(0)
                        continue

                    length = buf[1]
                    if length != 32:
                        # 잘못된 헤더 → 한 바이트 버리고 다시
                        buf.pop(0)
                        continue

                    # 전체 패킷(36 bytes) 도착했는지 확인
                    if len(buf) < 36:
                        break  # 더 기다려야 함

                    packet = bytes(buf[:36])
                    del buf[:36]

                    vals = self.parse_packet(packet)
                    if vals is not None:
                        with self.state_lock:
                            self.last_state = vals

            except Exception as e:
                self.get_logger().error(f"RX loop error: {e}")

    # ---------------- TX + Publish ----------------
    def tx_and_publish(self):
        # 1) 명령 패킷 송신
        try:
            self.get_logger().info(f"TX 전 상태: xdes={self.xdes:.4f}, ydes={self.ydes:.4f}, q1des={self.q1des:.4f}, q2des={self.q2des:.4f}")
            packet = self.build_packet()
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"TX error: {e}")
            return

        # 2) 가장 최근 상태값을 읽어와서 publish
        with self.state_lock:
            state = self.last_state

        if state is None:
            # 아직 수신된 프레임이 없음
            # self.get_logger().info("No state yet")
            return

        x, y, xv, yv, q1, q2, q1v, q2v = state

        self.get_logger().info(
            f"x={x:.4f}, y={y:.4f}, xv={xv:.4f}, yv={yv:.4f}, "
            f"q1={q1:.4f}, q2={q2:.4f}, q1v={q1v:.4f}, q2v={q2v:.4f}"
        )

        msg = XYpos()
        msg.xpos = x
        msg.ypos = y
        msg.xvel = xv
        msg.yvel = yv
        msg.qpos1 = q1
        msg.qpos2 = q2
        msg.qvel1 = q1v
        msg.qvel2 = q2v
        self.pub_robot_state.publish(msg)

    # ---------------- ROS 콜백 ----------------
    def listener_callback(self, cmd: CMD):
        self.xdes = cmd.xdes
        self.ydes = cmd.ydes
        self.q1des = cmd.q1des
        self.q2des = cmd.q2des

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

