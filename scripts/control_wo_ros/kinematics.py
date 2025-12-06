#!/usr/bin/env python3
"""
기구학 계산 모듈
2-link 로봇의 기구학 계산을 담당합니다.
"""

from math import sin, cos
import numpy as np
from robot_params import l1, l2, VISUAL_SCALE, VISUAL_OFFSET


def compute_manipulability(q1: float, q2: float) -> tuple:
    """2-link 로봇의 manipulability ellipsoid 계산
    Jacobian의 singular value를 계산하여 장축/단축을 반환
    
    Args:
        q1: Joint 1 angle (rad)
        q2: Joint 2 angle (rad)
    
    Returns:
        (major_axis, minor_axis, manipulability_measure, major_axis_direction)
        major_axis_direction: major axis의 방향 벡터 (2D numpy array)
    """
    # Jacobian 행렬 계산
    # J = [[-l1*sin(q1) - l2*sin(q1+q2), -l2*sin(q1+q2)],
    #      [l1*cos(q1) + l2*cos(q1+q2), l2*cos(q1+q2)]]
    J = np.array([
        [-l1*sin(q1) - l2*sin(q1+q2), -l2*sin(q1+q2)],
        [l1*cos(q1) + l2*cos(q1+q2), l2*cos(q1+q2)]
    ])
    
    # Singular value decomposition
    U, s, Vt = np.linalg.svd(J)
    
    # Singular values가 장축/단축 (또는 반지름)
    major_axis = s[0] if len(s) > 0 else 0.0
    minor_axis = s[1] if len(s) > 1 else 0.0
    
    # Manipulability measure: sqrt(det(J * J^T)) = s1 * s2
    manipulability_measure = np.prod(s) if len(s) > 0 else 0.0
    
    # Major axis 방향 (U 행렬의 첫 번째 열)
    major_axis_direction = U[:, 0] if U.shape[1] > 0 else np.array([1.0, 0.0])
    
    return major_axis, minor_axis, manipulability_measure, major_axis_direction


def joint_to_cartesian(q1: float, q2: float) -> tuple:
    """Joint angles를 Cartesian 좌표로 변환
    
    Args:
        q1: Joint 1 angle (rad)
        q2: Joint 2 angle (rad)
    
    Returns:
        (x, y) - Cartesian 좌표 (m)
    """
    x = l1 * cos(q1) + l2 * cos(q1 + q2)
    y = l1 * sin(q1) + l2 * sin(q1 + q2)
    return x, y


def joint_to_screen_coords(q1: float, q2: float) -> tuple:
    """Joint angles를 화면 좌표로 변환
    
    Args:
        q1: Joint 1 angle (rad)
        q2: Joint 2 angle (rad)
    
    Returns:
        ((x1, y1), (x2, y2)) - 첫 번째 링크 끝점, 두 번째 링크 끝점 (픽셀)
    """
    a1 = -q1
    a2 = -q2
    
    # 첫 번째 링크 끝점
    x1 = l1 * VISUAL_SCALE * cos(a1) + VISUAL_OFFSET[0]
    y1 = l1 * VISUAL_SCALE * sin(a1) + VISUAL_OFFSET[1]
    
    # 두 번째 링크 끝점
    x2 = x1 + l2 * VISUAL_SCALE * cos(a1 + a2)
    y2 = y1 + l2 * VISUAL_SCALE * sin(a1 + a2)
    
    return (x1, y1), (x2, y2)

