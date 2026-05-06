"""
clustering.py
반경 기반 단순 군집(radius_count) + 옵션 DBSCAN.

학부 프로젝트 수준에서 sklearn 의존성을 피하기 위해 numpy만 사용.
"""
from __future__ import annotations
from typing import List, Optional, Tuple

XYZ = Tuple[float, float, float]


def radius_count_cluster(
    points: List[XYZ],
    radius: float,
    min_count: int = 1,
) -> Tuple[Optional[XYZ], List[XYZ]]:
    """
    각 점을 중심으로 reach 안의 이웃 수를 세고, 가장 많은 이웃을 가진 점 그룹의
    centroid 를 반환한다. min_count 미만이면 None.

    반환: (cluster_center_xyz_or_None, member_points_list)
    """
    if not points:
        return None, []

    n = len(points)
    best_idx = -1
    best_count = -1
    best_members: List[int] = []

    for i in range(n):
        members = []
        for j in range(n):
            dx = points[i][0] - points[j][0]
            dy = points[i][1] - points[j][1]
            if (dx * dx + dy * dy) ** 0.5 <= radius:
                members.append(j)
        if len(members) > best_count:
            best_count = len(members)
            best_idx = i
            best_members = members

    if best_count < min_count:
        return None, []

    member_pts = [points[k] for k in best_members]
    sx = sum(p[0] for p in member_pts) / len(member_pts)
    sy = sum(p[1] for p in member_pts) / len(member_pts)
    sz = sum(p[2] for p in member_pts) / len(member_pts)
    return (sx, sy, sz), member_pts


def dbscan_cluster(
    points: List[XYZ],
    eps: float,
    min_samples: int = 2,
) -> Tuple[Optional[XYZ], List[XYZ]]:
    """
    아주 단순한 DBSCAN. 가장 큰 cluster의 centroid를 반환.
    sklearn 없이 O(N^2) 구현. (N이 수백 이하면 충분히 빠름)
    """
    if not points:
        return None, []

    n = len(points)
    visited = [False] * n
    cluster_id = [-1] * n  # -1 = noise

    def neighbors(idx: int) -> List[int]:
        out = []
        for k in range(n):
            dx = points[idx][0] - points[k][0]
            dy = points[idx][1] - points[k][1]
            if (dx * dx + dy * dy) ** 0.5 <= eps:
                out.append(k)
        return out

    cid = 0
    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        nbrs = neighbors(i)
        if len(nbrs) < min_samples:
            cluster_id[i] = -1
            continue
        cluster_id[i] = cid
        seeds = list(nbrs)
        while seeds:
            j = seeds.pop()
            if not visited[j]:
                visited[j] = True
                jn = neighbors(j)
                if len(jn) >= min_samples:
                    seeds.extend(jn)
            if cluster_id[j] == -1 or cluster_id[j] == -2:
                cluster_id[j] = cid
        cid += 1

    if cid == 0:
        return None, []

    # 가장 큰 cluster id 찾기
    counts = {}
    for c in cluster_id:
        if c < 0:
            continue
        counts[c] = counts.get(c, 0) + 1
    if not counts:
        return None, []
    biggest = max(counts, key=counts.get)
    members = [points[i] for i in range(n) if cluster_id[i] == biggest]
    sx = sum(p[0] for p in members) / len(members)
    sy = sum(p[1] for p in members) / len(members)
    sz = sum(p[2] for p in members) / len(members)
    return (sx, sy, sz), members
