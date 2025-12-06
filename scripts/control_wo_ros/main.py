#!/usr/bin/env python3
import pygame
import sys
import numpy as np
import time

from robot_params import step_size, cartesian_step, REQUEST_INTERVAL, MAX_GRAPH_POINTS
from serial_comm import SerialComm, list_serial_ports
from kinematics import compute_manipulability, joint_to_screen_coords
from visualization import Visualizer, GraphData


def main():
    """메인 함수"""
    # 시리얼 통신 초기화
    serial_comm = SerialComm()
    
    # 시리얼 포트 선택
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found!")
        return
    
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"{i}: {p.device}")
    
    port_idx = 0
    if len(ports) > 1:
        try:
            port_idx = int(input("Select port index (default 0): ") or "0")
        except:
            port_idx = 0
    
    port_name = ports[port_idx].device
    print(f"Connecting to {port_name}...")
    
    if not serial_comm.connect(port_name, 115200):
        return
    
    # 시리얼 수신 스레드 시작
    serial_comm.start_read_thread()
    
    # 시각화 초기화
    visualizer = Visualizer(width=1200, height=600)
    
    # 그래프 데이터 초기화
    graph_data = GraphData(max_points=MAX_GRAPH_POINTS)
    graph_start_time = time.time()
    
    print("Visualization started.")
    print("Controls:")
    print("  C: Switch to Cartesian mode (reset q1_desired, q2_desired to 0)")
    print("  J: Switch to Joint mode (reset x_desired, y_desired to 0)")
    print("  UP/DOWN arrows: Joint mode (q1) / Cartesian mode (y)")
    print("  LEFT/RIGHT arrows: Joint mode (q2) / Cartesian mode (x)")
    print("  Q: Quit")
    
    # 시리얼 데이터 준비될 때까지 기다림

    # 첫 패킷 올 때까지 대기# Arduino가 sendState를 보내도록 깨우기 위한 더미 명령
    serial_comm.send_joint_command(0.0, 0.0)
    time.sleep(0.05)
    while serial_comm.last_rx_time == 0.0:
        current_data = serial_comm.get_data()
        q1_desired = current_data['q1']
        q2_desired = current_data['q2']
        print(f"Waiting for first packet: q1_desired={np.rad2deg(q1_desired):.2f}°, q2_desired={np.rad2deg(q2_desired):.2f}°")
        time.sleep(0.005)
    current_data = serial_comm.get_data()
    q1_desired = current_data['q1']
    q2_desired = current_data['q2']
    print(f"Initialized: q1_desired={np.rad2deg(q1_desired):.2f}°, q2_desired={np.rad2deg(q2_desired):.2f}°")
    # x_desired = current_data['x']
    # y_desired = current_data['y']
    x_desired = 0.0
    y_desired = 0.0
    xdot_desired = 0.0
    ydot_desired = 0.0

    # 메인 루프
    prev_point = None
    last_request_time = 0.0
    running = True
    
    # Default mode: Joint mode
    is_cartesian_mode = False
    
    while running:
        
        # 현재 데이터 읽기 (키보드 입력 처리 전에)
        current_data = serial_comm.get_data()
        current_x = current_data['x']
        current_y = current_data['y']
        current_q1 = current_data['q1']
        current_q2 = current_data['q2']
        
        # 이벤트 처리
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False
                # 모드 전환
                elif event.key == pygame.K_c:  # c 키: Cartesian 모드로 전환
                    is_cartesian_mode = True
                    q1_desired = 0.0
                    q2_desired = 0.0
                    x_desired = current_x
                    y_desired = current_y
                    xdot_desired = 0.0
                    ydot_desired = 0.0
                    print(f"Switched to Cartesian mode (x_desired={x_desired:.3f}, y_desired={y_desired:.3f})")
                elif event.key == pygame.K_j:  # j 키: Joint 모드로 전환
                    is_cartesian_mode = False
                    x_desired = 0.0
                    y_desired = 0.0
                    q1_desired = current_q1
                    q2_desired = current_q2
                    print("Switched to Joint mode")
                # 방향키 제어 (모드에 따라 다름)
                elif event.key == pygame.K_UP:
                    if is_cartesian_mode:
                        # Cartesian 모드: y 증가
                        # y_desired += cartesian_step
                        y_desired = current_y + cartesian_step
                        xdot_desired = 0.0  # y 방향 이동 시 x 속도 0
                        ydot_desired = 1.0  # y 방향 이동
                        print(f"x_desired: {x_desired:.3f}m, y_desired: {y_desired:.3f}m (현재 위치: x={current_x:.3f}m, y={current_y:.3f}m)")
                    else:
                        # Joint 모드: q1 증가
                        q1_desired += step_size
                        print(f"q1_desired: {np.rad2deg(q1_desired):.1f}°")
                elif event.key == pygame.K_DOWN:
                    if is_cartesian_mode:
                        # Cartesian 모드: y 감소
                        # y_desired -= cartesian_step
                        y_desired = current_y - cartesian_step
                        xdot_desired = 0.0  # y 방향 이동 시 x 속도 0
                        ydot_desired = -1.0  # y 방향 이동 (음수)
                        print(f"x_desired: {x_desired:.3f}m, y_desired: {y_desired:.3f}m (현재 위치: x={current_x:.3f}m, y={current_y:.3f}m)")
                    else:
                        # Joint 모드: q1 감소
                        q1_desired -= step_size
                        print(f"q1_desired: {np.rad2deg(q1_desired):.1f}°")
                elif event.key == pygame.K_LEFT:
                    if is_cartesian_mode:
                        # Cartesian 모드: x 감소
                        # x_desired -= cartesian_step
                        x_desired = current_x - cartesian_step
                        xdot_desired = -1.0  # x 방향 이동 (음수)
                        ydot_desired = 0.0  # x 방향 이동 시 y 속도 0
                        print(f"x_desired: {x_desired:.3f}m, y_desired: {y_desired:.3f}m (현재 위치: x={current_x:.3f}m, y={current_y:.3f}m)")
                    else:
                        # Joint 모드: q2 감소
                        q2_desired -= step_size
                        print(f"q2_desired: {np.rad2deg(q2_desired):.1f}°")
                elif event.key == pygame.K_RIGHT:
                    if is_cartesian_mode:
                        # Cartesian 모드: x 증가
                        x_desired += cartesian_step
                        xdot_desired = 1.0  # x 방향 이동
                        ydot_desired = 0.0  # x 방향 이동 시 y 속도 0
                        print(f"x_desired: {x_desired:.3f}m, y_desired: {y_desired:.3f}m (현재 위치: x={current_x:.3f}m, y={current_y:.3f}m)")
                    else:
                        # Joint 모드: q2 증가
                        q2_desired += step_size
                        print(f"q2_desired: {np.rad2deg(q2_desired):.1f}°")
        
        # 주기적으로 데이터 요청
        current_time = time.time()
        if current_time - last_request_time >= REQUEST_INTERVAL:
            if is_cartesian_mode:
                serial_comm.send_cartesian_command(x_desired, y_desired, xdot_desired, ydot_desired)
            else:
                serial_comm.send_joint_command(q1_desired, q2_desired)
                # print(f"[Joint Cmd] q1_desired={np.rad2deg(q1_desired):.2f}°, q2_desired={np.rad2deg(q2_desired):.2f}°")
            last_request_time = current_time
        
        # 현재 데이터 읽기
        current_data = serial_comm.get_data()
        q1 = current_data['q1']
        q2 = current_data['q2']
        x = current_data['x']
        y = current_data['y']
        
        # 그래프 데이터 업데이트
        current_graph_time = time.time() - graph_start_time
        major_axis, minor_axis, manip_measure, major_axis_direction = compute_manipulability(q1, q2)
        graph_data.add_point(
            time=current_graph_time,
            q1=np.rad2deg(q1),
            q2=np.rad2deg(q2),
            q1_des=np.rad2deg(q1_desired),
            q2_des=np.rad2deg(q2_desired),
            x=x,
            y=y,
            x_des=x_desired,
            y_des=y_desired,
            manipulability=manip_measure
        )
        
        # 좌표 계산
        point1, point2 = joint_to_screen_coords(q1, q2)
        
        # 현재 모드 결정
        # is_cartesian_mode = abs(x_desired) > 0.002 or abs(y_desired) > 0.002
        current_mode = "Cartesian" if is_cartesian_mode else "Joint"
        
        # 렌더링
        comm_rate = serial_comm.get_comm_rate()
        prev_point = visualizer.render(point1, point2, prev_point, comm_rate, graph_data, major_axis, minor_axis, major_axis_direction, current_mode)
        
        visualizer.update()
    
    # 정리
    serial_comm.disconnect()
    visualizer.quit()
    sys.exit()


if __name__ == "__main__":
    main()
