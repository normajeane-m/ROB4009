#!/usr/bin/env python3
"""
시리얼 통신 모듈
Arduino와의 통신을 담당합니다.
"""

import serial
import serial.tools.list_ports
import struct
import threading
import time
from typing import Optional, Dict

# ===============================
# [ 시리얼 통신 설정 ]
# ===============================
STX = 0xAA  # Start byte
DATA_SIZE = 8  # float 8개
PAYLOAD_FMT = "<8f"  # little-endian, 8 floats
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)  # 32 bytes
CRC_SIZE = 2
PACKET_SIZE = 2 + PAYLOAD_SIZE + CRC_SIZE  # STX(1) + LEN(1) + payload(32) + CRC(2) = 36 bytes

# ===============================
# [ 통신 속도 측정 ]
# ===============================
MAX_RX_TIMES = 20  # 최근 20개 데이터로 평균 계산


class SerialComm:
    """시리얼 통신 클래스"""
    
    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.data_lock = threading.Lock()
        self.send_lock = threading.Lock()
        
        # Arduino에서 받은 데이터
        self.current_data = {
            'x': 0.0,      # Cartesian x
            'y': 0.0,      # Cartesian y
            'xdot': 0.0,   # Cartesian x velocity
            'ydot': 0.0,   # Cartesian y velocity
            'q1': 0.0,     # Joint 1 angle (rad)
            'q2': 0.0,     # Joint 2 angle (rad)
            'q1dot': 0.0,  # Joint 1 velocity
            'q2dot': 0.0   # Joint 2 velocity
        }
        
        # 통신 속도 측정
        self.comm_rate = 0.0  # Hz
        self.last_rx_time = 0.0
        self.rx_times = []  # 최근 수신 시간들 (이동 평균 계산용)
        
        self.read_thread: Optional[threading.Thread] = None
    
    def crc16(self, data: bytes) -> int:
        """Modbus CRC16 계산"""
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF
    
    def connect(self, port_name: str, baudrate: int = 115200) -> bool:
        """시리얼 포트 연결"""
        try:
            self.serial_port = serial.Serial(port_name, baudrate, timeout=0.1, write_timeout=0.1)
            self.serial_port.reset_input_buffer()  # 입력 버퍼 초기화
            self.serial_port.reset_output_buffer()  # 출력 버퍼 초기화
            time.sleep(2)  # 안정화 대기
            print("Connected!")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """시리얼 포트 연결 해제"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def start_read_thread(self):
        """시리얼 수신 스레드 시작"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        
        self.running = True
        self.read_thread = threading.Thread(target=self._serial_read_thread, daemon=True)
        self.read_thread.start()
        return True
    

    def send_joint_command(self, q1: float, q2: float) -> bool:
        """Joint space 명령 전송"""
        if not (self.serial_port and self.serial_port.is_open):
            return False
        try:
            # recv_data[0], [1]: Cartesian (0으로 설정)
            # recv_data[4], [5]: Joint (q1, q2)
            vals = [0.0, 0.0, 0.0, 0.0, q1, q2, 0.0, 0.0]
            payload = struct.pack(PAYLOAD_FMT, *vals)
            crc = self.crc16(payload)
            
            packet = bytes([STX, PAYLOAD_SIZE]) + payload + struct.pack("<H", crc)
            with self.send_lock:
                self.serial_port.write(packet)
                self.serial_port.flush()
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False

    def send_cartesian_command(self, x: float, y: float, xdot: float = 0.0, ydot: float = 0.0) -> bool:
        """Cartesian space 명령 전송"""
        if not (self.serial_port and self.serial_port.is_open):
            return False
        try:
            # recv_data[0], [1]: Cartesian (x, y)
            # recv_data[4], [5]: Joint (0으로 설정)
            vals = [x, y, xdot, ydot, 0.0, 0.0, 0.0, 0.0]
            payload = struct.pack(PAYLOAD_FMT, *vals)
            crc = self.crc16(payload)
            
            packet = bytes([STX, PAYLOAD_SIZE]) + payload + struct.pack("<H", crc)
            with self.send_lock:
                self.serial_port.write(packet)
                self.serial_port.flush()
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def get_data(self) -> Dict[str, float]:
        """현재 데이터 복사본 반환 (스레드 안전)"""
        with self.data_lock:
            return self.current_data.copy()
    
    def get_comm_rate(self) -> float:
        """통신 속도 반환 (Hz)"""
        return self.comm_rate

    def _serial_read_thread(self):
        """Arduino → Python 패킷 수신 스레드 (STX + LEN + payload + CRC)"""
        
        while self.running:
            try:
                # 1) STX 찾기
                if self.serial_port.in_waiting < 2:
                    time.sleep(0.001)
                    continue
                
                b = self.serial_port.read(1)
                if b[0] != STX:
                    continue  # sync 맞출 때까지 버림
                
                # 2) LEN 읽기
                len_byte = self.serial_port.read(1)
                if len(len_byte) != 1:
                    continue
                
                expected_len = len_byte[0]
                if expected_len != PAYLOAD_SIZE:
                    continue  # 잘못된 길이
                
                # 3) payload + CRC 읽기
                need_bytes = expected_len + CRC_SIZE
                if self.serial_port.in_waiting < need_bytes:
                    time.sleep(0.001)
                    continue
                
                rx_buffer = self.serial_port.read(need_bytes)
                if len(rx_buffer) != need_bytes:
                    continue  # 불완전한 패킷 → 버림
                
                # 4) CRC 검사
                payload = rx_buffer[:expected_len]
                recv_crc = struct.unpack("<H", rx_buffer[expected_len:expected_len+CRC_SIZE])[0]
                calc_crc = self.crc16(payload)
                
                if calc_crc != recv_crc:
                    continue  # CRC 오류 → 버림
                
                # 5) float 8개 언팩
                vals = struct.unpack(PAYLOAD_FMT, payload)

                # -----------------------------
                # 저장 (스레드 안전)
                # Arduino send_data 순서:
                # [0]: xpos[0] (x)
                # [1]: xpos[1] (y)
                # [2]: xvel[0] (vx)
                # [3]: xvel[1] (vy)
                # [4]: qpos[0] (q1)
                # [5]: qpos[1] (q2)
                # [6]: qvel[0] (q1dot)
                # [7]: qvel[1] (q2dot)
                # -----------------------------
                with self.data_lock:
                    self.current_data['x']     = vals[0]
                    self.current_data['y']     = vals[1]
                    self.current_data['xdot']  = vals[2]
                    self.current_data['ydot']  = vals[3]
                    self.current_data['q1']    = vals[4]
                    self.current_data['q2']    = vals[5]
                    self.current_data['q1dot'] = vals[6]
                    self.current_data['q2dot'] = vals[7]

                # -----------------------------
                # 통신 속도 계산
                # -----------------------------
                now = time.time()
                if self.last_rx_time > 0:
                    dt = now - self.last_rx_time
                    if dt > 0:
                        self.rx_times.append(dt)
                        if len(self.rx_times) > MAX_RX_TIMES:
                            self.rx_times.pop(0)

                        avg = sum(self.rx_times) / len(self.rx_times)
                        self.comm_rate = 1.0 / avg if avg > 0 else 0.0

                self.last_rx_time = now

            except Exception as e:
                print(f"[Serial Read Error] {e}")
                time.sleep(0.01)

def list_serial_ports():
    """사용 가능한 시리얼 포트 목록 반환"""
    ports = serial.tools.list_ports.comports()
    return ports

