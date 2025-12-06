#!/usr/bin/env python3
"""
시각화 모듈
Pygame을 사용한 로봇 시각화 및 그래프 그리기를 담당합니다.
"""

import pygame
import numpy as np
from math import cos, sin, atan2, pi
from typing import Dict, Tuple, Optional
from robot_params import m1, m2, VISUAL_OFFSET


class GraphData:
    """그래프 데이터 관리 클래스"""
    
    def __init__(self, max_points: int = 300):
        self.max_points = max_points
        self.data = {
            'time': [],
            'q1': [],
            'q2': [],
            'q1_des': [],
            'q2_des': [],
            'x': [],
            'y': [],
            'x_des': [],
            'y_des': [],
            'manipulability': []
        }
    
    def add_point(self, time: float, q1: float, q2: float, q1_des: float, q2_des: float,
                  x: float, y: float, x_des: float, y_des: float, manipulability: float):
        """데이터 포인트 추가"""
        self.data['time'].append(time)
        self.data['q1'].append(q1)
        self.data['q2'].append(q2)
        self.data['q1_des'].append(q1_des)
        self.data['q2_des'].append(q2_des)
        self.data['x'].append(x)
        self.data['y'].append(y)
        self.data['x_des'].append(x_des)
        self.data['y_des'].append(y_des)
        self.data['manipulability'].append(manipulability)
        
        # 최대 포인트 수 제한
        if len(self.data['time']) > self.max_points:
            for key in self.data:
                self.data[key].pop(0)
    
    def get_data(self) -> Dict:
        """데이터 딕셔너리 반환"""
        return self.data


class Visualizer:
    """시각화 클래스"""
    
    def __init__(self, width: int = 1200, height: int = 600):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        self.screen.fill((255, 255, 255))
        self.trace = self.screen.copy()
        self.clock = pygame.time.Clock()
        
        pygame.font.init()
        self.font = pygame.font.SysFont('Comic Sans MS', 30)
        self.small_font = pygame.font.SysFont('Comic Sans MS', 20)
        
        self.width = width
        self.height = height
    
    def update(self):
        """화면 업데이트"""
        pygame.display.update()
        self.clock.tick(60)  # 60 FPS
    
    def render(self, point1: Tuple[float, float], point2: Tuple[float, float],
               prev_point: Optional[Tuple[float, float]], comm_rate: float,
               graph_data: GraphData, major_axis: float = 0.0, minor_axis: float = 0.0,
               major_axis_direction = None, current_mode: str = "Joint") -> Tuple[float, float]:
        """화면에 로봇과 정보를 그리기"""
        # 궤적 그리기
        x2, y2 = int(point2[0]), int(point2[1])
        if prev_point:
            xp, yp = prev_point[0], prev_point[1]
            pygame.draw.line(self.trace, (230, 230, 255), (xp, yp), (x2, y2), 3)
        
        # 배경
        self.screen.fill((255, 255, 255))
        self.screen.blit(self.trace, (0, 0))
        
        # 링크 그리기
        offset = VISUAL_OFFSET
        x1, y1 = int(point1[0]), int(point1[1])
        x2, y2 = int(point2[0]), int(point2[1])
        
        # 좌표축 그리기
        axis_length = 80
        arrow_size = 8
        
        # +x축 (오른쪽, 빨간색)
        x_axis_end = (offset[0] + axis_length, offset[1])
        pygame.draw.line(self.screen, (255, 0, 0), offset, x_axis_end, 3)
        pygame.draw.polygon(self.screen, (255, 0, 0), [
            (x_axis_end[0], x_axis_end[1]),
            (x_axis_end[0] - arrow_size, x_axis_end[1] - arrow_size//2),
            (x_axis_end[0] - arrow_size, x_axis_end[1] + arrow_size//2)
        ])
        x_label = self.small_font.render('+x', True, (255, 0, 0))
        self.screen.blit(x_label, (x_axis_end[0] + 5, x_axis_end[1] - 10))
        
        # +y축 (위쪽, 파란색)
        y_axis_end = (offset[0], offset[1] - axis_length)
        pygame.draw.line(self.screen, (0, 0, 255), offset, y_axis_end, 3)
        pygame.draw.polygon(self.screen, (0, 0, 255), [
            (y_axis_end[0], y_axis_end[1]),
            (y_axis_end[0] - arrow_size//2, y_axis_end[1] + arrow_size),
            (y_axis_end[0] + arrow_size//2, y_axis_end[1] + arrow_size)
        ])
        y_label = self.small_font.render('+y', True, (0, 0, 255))
        self.screen.blit(y_label, (y_axis_end[0] + 5, y_axis_end[1] - 15))
        
        # 원점 표시
        origin_label = self.small_font.render('O', True, (0, 0, 0))
        self.screen.blit(origin_label, (offset[0] - 15, offset[1] + 5))
        
        pygame.draw.line(self.screen, (0, 0, 0), offset, (x1, y1), 5)
        pygame.draw.line(self.screen, (0, 0, 0), (x1, y1), (x2, y2), 5)
        
        # 관절 그리기
        pygame.draw.circle(self.screen, (0, 0, 0), offset, 8)
        pygame.draw.circle(self.screen, (255, 0, 0), (x1, y1), int(m1 * 10))
        pygame.draw.circle(self.screen, (0, 0, 255), (x2, y2), int(m2 * 10))
        
        # Manipulability 타원 그리기 (point2 위치에)
        if major_axis > 0 and minor_axis > 0 and major_axis_direction is not None:
            # 타원의 크기를 화면에 맞게 스케일링 (VISUAL_SCALE 사용)
            from robot_params import VISUAL_SCALE
            ellipse_a = major_axis * VISUAL_SCALE * 0.5  # 장축 반지름 (픽셀)
            ellipse_b = minor_axis * VISUAL_SCALE * 0.5  # 단축 반지름 (픽셀)
            
            # Major axis 방향 벡터로부터 회전 각도 계산
            # 화면 좌표계: y축이 아래로 향하므로 atan2의 부호를 조정
            dir_vec = major_axis_direction
            angle = atan2(-dir_vec[1], dir_vec[0])  # y축 반전
            
            # 타원의 각 점을 계산하여 polygon으로 그리기
            num_points = 64  # 타원의 부드러움을 위한 점 개수
            ellipse_points = []
            for i in range(num_points):
                # 타원의 파라미터 방정식 (0도에서 시작)
                theta = 2 * pi * i / num_points
                # 타원의 로컬 좌표
                local_x = ellipse_a * cos(theta)
                local_y = ellipse_b * sin(theta)
                # 회전 변환
                rotated_x = local_x * cos(angle) - local_y * sin(angle)
                rotated_y = local_x * sin(angle) + local_y * cos(angle)
                # 화면 좌표로 변환 (point2가 중심)
                screen_x = int(x2 + rotated_x)
                screen_y = int(y2 + rotated_y)
                ellipse_points.append((screen_x, screen_y))
            
            # 타원 그리기 (반투명 채우기와 테두리)
            if len(ellipse_points) > 2:
                # 반투명 채우기를 위한 surface 생성
                # 타원의 경계 상자 계산
                min_x = min(p[0] for p in ellipse_points)
                max_x = max(p[0] for p in ellipse_points)
                min_y = min(p[1] for p in ellipse_points)
                max_y = max(p[1] for p in ellipse_points)
                width = max_x - min_x + 4
                height = max_y - min_y + 4
                
                ellipse_surface = pygame.Surface((width, height), pygame.SRCALPHA)
                # 상대 좌표로 변환
                relative_points = [(p[0] - min_x + 2, p[1] - min_y + 2) for p in ellipse_points]
                pygame.draw.polygon(ellipse_surface, (255, 0, 255, 100), relative_points)
                pygame.draw.polygon(ellipse_surface, (255, 0, 255, 255), relative_points, 2)
                self.screen.blit(ellipse_surface, (min_x - 2, min_y - 2))
        
        # 통신 속도 표시 (우하단)
        comm_rate_string = f'{comm_rate:.1f} Hz'
        text_surface = self.font.render(comm_rate_string, False, (0, 0, 0))
        text_x = self.width - text_surface.get_width() - 10
        text_y = self.height - text_surface.get_height() - 10
        self.screen.blit(text_surface, (text_x, text_y))
        
        # 모드 표시 (우하단, 통신 속도 위)
        mode_color = (0, 150, 0) if current_mode == "Cartesian" else (0, 0, 150)
        mode_string = f'Mode: {current_mode}'
        mode_surface = self.font.render(mode_string, False, mode_color)
        mode_x = self.width - mode_surface.get_width() - 10
        mode_y = text_y - mode_surface.get_height() - 5
        self.screen.blit(mode_surface, (mode_x, mode_y))
        
        # 그래프 그리기
        self._draw_all_graphs(graph_data.get_data())
        
        return (x2, y2)
    
    def _draw_all_graphs(self, graph_data: Dict):
        """모든 그래프 그리기"""
        graph_width = 290
        graph_height = 145
        graph_spacing = 10
        
        top_y = self.height - graph_height * 3 - graph_spacing - 10
        bottom_y = self.height - graph_height * 2 - graph_spacing - 10
        middle_y = self.height - graph_height * 1 - graph_spacing - 10

        right_x = self.width - graph_width * 2 - graph_spacing * 1 - 10
        middle_x = self.width - graph_width * 1 - graph_spacing - 10
        left_x = self.width - graph_width * 2 - graph_spacing * 1 - 10
        
        # q1 그래프 (위쪽 왼쪽)
        self._draw_q1_graph(graph_data, right_x, top_y, graph_width, graph_height)
        
        # q2 그래프 (위쪽 오른쪽)
        self._draw_q2_graph(graph_data, middle_x, top_y, graph_width, graph_height)
        
        # Manipulability 그래프 (아래 왼쪽)
        self._draw_manipulability_graph(graph_data, left_x, middle_y, graph_width, graph_height)
        
        # x 그래프 (중간 왼쪽)
        self._draw_x_graph(graph_data, right_x, bottom_y, graph_width, graph_height)
        
        # y 그래프 (중간 오른쪽)
        self._draw_y_graph(graph_data, middle_x, bottom_y, graph_width, graph_height)
    
    def _draw_q1_graph(self, graph_data: Dict, x_offset: int, y_offset: int, width: int, height: int):
        """q1 그래프 그리기"""
        if len(graph_data['time']) < 2:
            return
        
        graph_rect = pygame.Rect(x_offset, y_offset, width, height)
        pygame.draw.rect(self.screen, (240, 240, 240), graph_rect)
        pygame.draw.rect(self.screen, (0, 0, 0), graph_rect, 2)
        
        all_q = graph_data['q1'] + graph_data['q1_des']
        if len(all_q) == 0:
            return
        
        q_min = min(all_q)
        q_max = max(all_q)
        q_range = q_max - q_min
        if q_range < 1.0:
            q_center = (q_min + q_max) / 2
            q_min = q_center - 10.0
            q_max = q_center + 10.0
            q_range = 20.0
        
        time_min = graph_data['time'][0] if len(graph_data['time']) > 0 else 0
        time_max = graph_data['time'][-1] if len(graph_data['time']) > 0 else 1
        time_range = time_max - time_min if time_max > time_min else 1.0
        
        def time_to_x(t):
            return x_offset + 40 + int((t - time_min) / time_range * (width - 80))
        
        def q_to_y(q):
            return y_offset + height - 30 - int((q - q_min) / q_range * (height - 60))
        
        # 격자 그리기
        for i in range(5):
            y = y_offset + 30 + int(i * (height - 60) / 4)
            pygame.draw.line(self.screen, (200, 200, 200),
                           (x_offset + 40, y), (x_offset + width - 40, y), 1)
            q_val = q_max - i * q_range / 4
            label = self.small_font.render(f'{q_val:.0f}°', True, (100, 100, 100))
            self.screen.blit(label, (x_offset + 5, y - 10))
        
        if time_range > 0:
            for i in range(5):
                t = time_min + i * time_range / 4
                x = time_to_x(t)
                pygame.draw.line(self.screen, (200, 200, 200),
                               (x, y_offset + 30), (x, y_offset + height - 30), 1)
                label = self.small_font.render(f'{t:.1f}s', True, (100, 100, 100))
                self.screen.blit(label, (x - 20, y_offset + height - 25))
        
        # q1 그래프 (빨간색)
        if len(graph_data['q1']) > 1:
            points = [(time_to_x(graph_data['time'][i]), q_to_y(graph_data['q1'][i]))
                     for i in range(len(graph_data['q1']))]
            if len(points) > 1:
                pygame.draw.lines(self.screen, (255, 0, 0), False, points, 2)
        
        # q1_des 그래프 (빨간 점선)
        if len(graph_data['q1_des']) > 1:
            points = [(time_to_x(graph_data['time'][i]), q_to_y(graph_data['q1_des'][i]))
                     for i in range(len(graph_data['q1_des']))]
            if len(points) > 1:
                for i in range(len(points) - 1):
                    if i % 2 == 0:
                        pygame.draw.line(self.screen, (255, 100, 100), points[i], points[i+1], 1)
        
        # 범례
        legend_y = y_offset + 10
        legend_x = x_offset + width - 80
        pygame.draw.line(self.screen, (255, 0, 0), (legend_x, legend_y),
                        (legend_x + 30, legend_y), 2)
        label = self.small_font.render('q1', True, (0, 0, 0))
        self.screen.blit(label, (legend_x + 35, legend_y - 8))
    
    def _draw_q2_graph(self, graph_data: Dict, x_offset: int, y_offset: int, width: int, height: int):
        """q2 그래프 그리기"""
        if len(graph_data['time']) < 2:
            return
        
        graph_rect = pygame.Rect(x_offset, y_offset, width, height)
        pygame.draw.rect(self.screen, (240, 240, 240), graph_rect)
        pygame.draw.rect(self.screen, (0, 0, 0), graph_rect, 2)
        
        all_q = graph_data['q2'] + graph_data['q2_des']
        if len(all_q) == 0:
            return
        
        q_min = min(all_q)
        q_max = max(all_q)
        q_range = q_max - q_min
        if q_range < 1.0:
            q_center = (q_min + q_max) / 2
            q_min = q_center - 10.0
            q_max = q_center + 10.0
            q_range = 20.0
        
        time_min = graph_data['time'][0] if len(graph_data['time']) > 0 else 0
        time_max = graph_data['time'][-1] if len(graph_data['time']) > 0 else 1
        time_range = time_max - time_min if time_max > time_min else 1.0
        
        def time_to_x(t):
            return x_offset + 40 + int((t - time_min) / time_range * (width - 80))
        
        def q_to_y(q):
            return y_offset + height - 30 - int((q - q_min) / q_range * (height - 60))
        
        # 격자 그리기
        for i in range(5):
            y = y_offset + 30 + int(i * (height - 60) / 4)
            pygame.draw.line(self.screen, (200, 200, 200),
                           (x_offset + 40, y), (x_offset + width - 40, y), 1)
            q_val = q_max - i * q_range / 4
            label = self.small_font.render(f'{q_val:.0f}°', True, (100, 100, 100))
            self.screen.blit(label, (x_offset + 5, y - 10))
        
        if time_range > 0:
            for i in range(5):
                t = time_min + i * time_range / 4
                x = time_to_x(t)
                pygame.draw.line(self.screen, (200, 200, 200),
                               (x, y_offset + 30), (x, y_offset + height - 30), 1)
                label = self.small_font.render(f'{t:.1f}s', True, (100, 100, 100))
                self.screen.blit(label, (x - 20, y_offset + height - 25))
        
        # q2 그래프 (파란색)
        if len(graph_data['q2']) > 1:
            points = [(time_to_x(graph_data['time'][i]), q_to_y(graph_data['q2'][i]))
                     for i in range(len(graph_data['q2']))]
            if len(points) > 1:
                pygame.draw.lines(self.screen, (0, 0, 255), False, points, 2)
        
        # q2_des 그래프 (파란 점선)
        if len(graph_data['q2_des']) > 1:
            points = [(time_to_x(graph_data['time'][i]), q_to_y(graph_data['q2_des'][i]))
                     for i in range(len(graph_data['q2_des']))]
            if len(points) > 1:
                for i in range(len(points) - 1):
                    if i % 2 == 0:
                        pygame.draw.line(self.screen, (100, 100, 255), points[i], points[i+1], 1)
        
        # 범례
        legend_y = y_offset + 10
        legend_x = x_offset + width - 80
        pygame.draw.line(self.screen, (0, 0, 255), (legend_x, legend_y),
                        (legend_x + 30, legend_y), 2)
        label = self.small_font.render('q2', True, (0, 0, 0))
        self.screen.blit(label, (legend_x + 35, legend_y - 8))
    
    def _draw_x_graph(self, graph_data: Dict, x_offset: int, y_offset: int, width: int, height: int):
        """x 그래프 그리기"""
        if len(graph_data['time']) < 2:
            return
        
        graph_rect = pygame.Rect(x_offset, y_offset, width, height)
        pygame.draw.rect(self.screen, (240, 240, 240), graph_rect)
        pygame.draw.rect(self.screen, (0, 0, 0), graph_rect, 2)
        
        all_x = graph_data['x'] + graph_data['x_des']
        if len(all_x) == 0:
            return
        
        xy_min = min(all_x)
        xy_max = max(all_x)
        xy_range = xy_max - xy_min
        if xy_range < 0.01:
            xy_center = (xy_min + xy_max) / 2
            xy_min = xy_center - 0.1
            xy_max = xy_center + 0.1
            xy_range = 0.2
        
        time_min = graph_data['time'][0] if len(graph_data['time']) > 0 else 0
        time_max = graph_data['time'][-1] if len(graph_data['time']) > 0 else 1
        time_range = time_max - time_min if time_max > time_min else 1.0
        
        def time_to_x(t):
            return x_offset + 40 + int((t - time_min) / time_range * (width - 80))
        
        def xy_to_y(val):
            return y_offset + height - 30 - int((val - xy_min) / xy_range * (height - 60))
        
        # 격자 그리기
        for i in range(5):
            y = y_offset + 30 + int(i * (height - 60) / 4)
            pygame.draw.line(self.screen, (200, 200, 200),
                           (x_offset + 40, y), (x_offset + width - 40, y), 1)
            xy_val = xy_max - i * xy_range / 4
            label = self.small_font.render(f'{xy_val:.3f}m', True, (100, 100, 100))
            self.screen.blit(label, (x_offset + 5, y - 10))
        
        if time_range > 0:
            for i in range(5):
                t = time_min + i * time_range / 4
                x = time_to_x(t)
                pygame.draw.line(self.screen, (200, 200, 200),
                               (x, y_offset + 30), (x, y_offset + height - 30), 1)
                label = self.small_font.render(f'{t:.1f}s', True, (100, 100, 100))
                self.screen.blit(label, (x - 20, y_offset + height - 25))
        
        # x 그래프 (초록색)
        if len(graph_data['x']) > 1:
            points = [(time_to_x(graph_data['time'][i]), xy_to_y(graph_data['x'][i]))
                     for i in range(len(graph_data['x']))]
            if len(points) > 1:
                pygame.draw.lines(self.screen, (0, 255, 0), False, points, 2)
        
        # x_des 그래프 (초록 점선)
        if len(graph_data['x_des']) > 1:
            points = [(time_to_x(graph_data['time'][i]), xy_to_y(graph_data['x_des'][i]))
                     for i in range(len(graph_data['x_des']))]
            if len(points) > 1:
                for i in range(len(points) - 1):
                    if i % 2 == 0:
                        pygame.draw.line(self.screen, (100, 255, 100), points[i], points[i+1], 1)
        
        # 범례
        legend_y = y_offset + 10
        legend_x = x_offset + width - 60
        pygame.draw.line(self.screen, (0, 255, 0), (legend_x, legend_y),
                        (legend_x + 30, legend_y), 2)
        label = self.small_font.render('x', True, (0, 0, 0))
        self.screen.blit(label, (legend_x + 35, legend_y - 8))
    
    def _draw_y_graph(self, graph_data: Dict, x_offset: int, y_offset: int, width: int, height: int):
        """y 그래프 그리기"""
        if len(graph_data['time']) < 2:
            return
        
        graph_rect = pygame.Rect(x_offset, y_offset, width, height)
        pygame.draw.rect(self.screen, (240, 240, 240), graph_rect)
        pygame.draw.rect(self.screen, (0, 0, 0), graph_rect, 2)
        
        all_y = graph_data['y'] + graph_data['y_des']
        if len(all_y) == 0:
            return
        
        xy_min = min(all_y)
        xy_max = max(all_y)
        xy_range = xy_max - xy_min
        if xy_range < 0.01:
            xy_center = (xy_min + xy_max) / 2
            xy_min = xy_center - 0.1
            xy_max = xy_center + 0.1
            xy_range = 0.2
        
        time_min = graph_data['time'][0] if len(graph_data['time']) > 0 else 0
        time_max = graph_data['time'][-1] if len(graph_data['time']) > 0 else 1
        time_range = time_max - time_min if time_max > time_min else 1.0
        
        def time_to_x(t):
            return x_offset + 40 + int((t - time_min) / time_range * (width - 80))
        
        def xy_to_y(val):
            return y_offset + height - 30 - int((val - xy_min) / xy_range * (height - 60))
        
        # 격자 그리기
        for i in range(5):
            y = y_offset + 30 + int(i * (height - 60) / 4)
            pygame.draw.line(self.screen, (200, 200, 200),
                           (x_offset + 40, y), (x_offset + width - 40, y), 1)
            xy_val = xy_max - i * xy_range / 4
            label = self.small_font.render(f'{xy_val:.3f}m', True, (100, 100, 100))
            self.screen.blit(label, (x_offset + 5, y - 10))
        
        if time_range > 0:
            for i in range(5):
                t = time_min + i * time_range / 4
                x = time_to_x(t)
                pygame.draw.line(self.screen, (200, 200, 200),
                               (x, y_offset + 30), (x, y_offset + height - 30), 1)
                label = self.small_font.render(f'{t:.1f}s', True, (100, 100, 100))
                self.screen.blit(label, (x - 20, y_offset + height - 25))
        
        # y 그래프 (주황색)
        if len(graph_data['y']) > 1:
            points = [(time_to_x(graph_data['time'][i]), xy_to_y(graph_data['y'][i]))
                     for i in range(len(graph_data['y']))]
            if len(points) > 1:
                pygame.draw.lines(self.screen, (255, 165, 0), False, points, 2)
        
        # y_des 그래프 (주황 점선)
        if len(graph_data['y_des']) > 1:
            points = [(time_to_x(graph_data['time'][i]), xy_to_y(graph_data['y_des'][i]))
                     for i in range(len(graph_data['y_des']))]
            if len(points) > 1:
                for i in range(len(points) - 1):
                    if i % 2 == 0:
                        pygame.draw.line(self.screen, (255, 200, 100), points[i], points[i+1], 1)
        
        # 범례
        legend_y = y_offset + 10
        legend_x = x_offset + width - 60
        pygame.draw.line(self.screen, (255, 165, 0), (legend_x, legend_y),
                        (legend_x + 30, legend_y), 2)
        label = self.small_font.render('y', True, (0, 0, 0))
        self.screen.blit(label, (legend_x + 35, legend_y - 8))
    
    def _draw_manipulability_graph(self, graph_data: Dict, x_offset: int, y_offset: int, width: int, height: int):
        """Manipulability 그래프 그리기"""
        if len(graph_data['time']) < 2:
            return
        
        graph_rect = pygame.Rect(x_offset, y_offset, width, height)
        pygame.draw.rect(self.screen, (240, 240, 240), graph_rect)
        pygame.draw.rect(self.screen, (0, 0, 0), graph_rect, 2)
        
        title = self.small_font.render('Manipulability', True, (0, 0, 0))
        self.screen.blit(title, (x_offset + 5, y_offset + 5))
        
        all_manip = graph_data['manipulability']
        if len(all_manip) == 0:
            return
        
        manip_min = min(all_manip) if all_manip else 0
        manip_max = max(all_manip) if all_manip else 1
        manip_range = manip_max - manip_min if manip_max > manip_min else 1
        
        def time_to_x(t):
            time_min = graph_data['time'][0] if len(graph_data['time']) > 0 else 0
            time_max = graph_data['time'][-1] if len(graph_data['time']) > 0 else 1
            time_range = time_max - time_min if time_max > time_min else 1
            return x_offset + 50 + int((t - time_min) / time_range * (width - 60))
        
        def manip_to_y(m):
            return y_offset + height - 30 - int((m - manip_min) / manip_range * (height - 50))
        
        # 그리드 그리기
        num_y_ticks = 5
        for i in range(num_y_ticks + 1):
            manip_val = manip_min + (manip_max - manip_min) * i / num_y_ticks
            y_pos = y_offset + height - 30 - int(i / num_y_ticks * (height - 50))
            pygame.draw.line(self.screen, (200, 200, 200),
                           (x_offset + 50, y_pos),
                           (x_offset + width - 10, y_pos), 1)
            label = self.small_font.render(f'{manip_val:.3f}', True, (100, 100, 100))
            self.screen.blit(label, (x_offset + 5, y_pos - 8))
        
        time_min = graph_data['time'][0] if len(graph_data['time']) > 0 else 0
        time_max = graph_data['time'][-1] if len(graph_data['time']) > 0 else 1
        num_x_ticks = 5
        for i in range(num_x_ticks + 1):
            t = time_min + (time_max - time_min) * i / num_x_ticks
            x_pos = time_to_x(t)
            pygame.draw.line(self.screen, (200, 200, 200),
                           (x_pos, y_offset + 25),
                           (x_pos, y_offset + height - 30), 1)
            label = self.small_font.render(f'{t:.1f}s', True, (100, 100, 100))
            self.screen.blit(label, (x_pos - 15, y_offset + height - 25))
        
        # 데이터 그리기
        if len(graph_data['manipulability']) > 1:
            points = [(time_to_x(graph_data['time'][i]), manip_to_y(graph_data['manipulability'][i]))
                     for i in range(len(graph_data['manipulability']))]
            if len(points) > 1:
                pygame.draw.lines(self.screen, (128, 0, 128), False, points, 2)
        
        # 범례
        legend_y = y_offset + 10
        legend_x = x_offset + width - 60
        pygame.draw.line(self.screen, (128, 0, 128), (legend_x, legend_y),
                        (legend_x + 30, legend_y), 2)
        label = self.small_font.render('μ', True, (0, 0, 0))
        self.screen.blit(label, (legend_x + 35, legend_y - 8))
    
    def quit(self):
        """Pygame 종료"""
        pygame.quit()

