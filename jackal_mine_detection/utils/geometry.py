"""
geometry.py
간단한 거리/각도/PoseStamped 헬퍼.
ROS 메세지 타입을 직접 다루기 보다는 (x,y,z) tuple 단위로 처리한다.
"""
from __future__ import annotations
import math
from typing import Iterable, Tuple


XYZ = Tuple[float, float, float]


def dist_xy(a: XYZ, b: XYZ) -> float:
    """2D (x,y) 평면 거리. z는 무시."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist_xyz(a: XYZ, b: XYZ) -> float:
    """3D 거리."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


def centroid(points: Iterable[XYZ]) -> XYZ:
    """점들의 평균 좌표. 빈 입력은 (0,0,0) 리턴."""
    pts = list(points)
    if not pts:
        return (0.0, 0.0, 0.0)
    n = float(len(pts))
    sx = sum(p[0] for p in pts) / n
    sy = sum(p[1] for p in pts) / n
    sz = sum(p[2] for p in pts) / n
    return (sx, sy, sz)


def yaw_from_to(src: XYZ, dst: XYZ) -> float:
    """src 위치에서 dst를 바라보는 yaw [rad]."""
    return math.atan2(dst[1] - src[1], dst[0] - src[0])


def quat_from_yaw(yaw: float):
    """yaw -> quaternion (x,y,z,w). tf 의존 없이 단순 계산."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))
