# =====================================================================================
# [실행 명령어]
#   python analyze_group_pipes.py ./data-v10 --mode equipment --tol_z 150 --max_spacing 500
#   python analyze_group_pipes.py ./data-v10 --mode equipment --tol_z 150 --max_spacing 500 --min_group_count 2
#   python analyze_group_pipes.py ./data-v11 --mode equipment --tol_z 150 --max_spacing 500 --min_group_count 1
# =====================================================================================
#
# analyze_group_pipes.py  — 3D 배관 그룹 분석 및 클러스터링
# ==========================================================
#
# [프로그램 개요]
#   3D 배관 설계 데이터(JSON)에서 인접한 배관들을 찾아 그룹(Group)으로 묶고
#   그룹별 통계(바운더리, 간격, 고도 등)를 산출하여 CSV로 저장합니다.
#
# [전체 흐름도]
#   ┌───────────────────────────────────────────────────────────────┐
#   │  1. argparse로 분석 디렉토리/모드 설정                        │
#   │  2. 디렉토리 내 *.json 파일 검색                              │
#   │  3. 데이터 파싱 (analyze_json):                               │
#   │     ├─ Nodes: UUID→3D좌표 매핑                                │
#   │     ├─ Edges: 유효 배관 → PipeData 객체 생성                   │
#   │     └─ Equipment: 장비 목록 + POC 추출                         │
#   │  4. BFS 기반 배관 추출 및 그래프 탐색                          │
#   │  5. 배관 분류: 방향벡터 분석 → 수직/수평 분류                   │
#   │  6. 3단계 클러스터링:                                          │
#   │     ├─ 1차: Z축 공차 기반 초기 묶음 (group_by_tolerance)       │
#   │     ├─ 2차: 방향 벡터 평행성 검증 (group_by_direction)         │
#   │     └─ 3차: BFS 밀집 공간 클러스터 (cluster_*_pipes)           │
#   │  7. 그룹 통계 산출 + CSV 저장                                  │
#   └───────────────────────────────────────────────────────────────┘
#
# [주요 함수]
#   - dot_product(v1, v2)          : 두 3D 벡터의 내적
#   - normalize(v)                 : 3D 벡터를 단위 방향 벡터로 변환
#   - group_by_tolerance()         : 공차 이내 근접 파이프 1차 묶음
#   - group_by_direction()         : 방향 벡터 내적 기반 평행성 2차 필터링
#   - cluster_vertical_pipes()     : 수직 배관 BFS 밀집 공간 3차 클러스터링
#   - cluster_horizontal_pipes()   : 수평 배관 BFS 밀집 공간 3차 클러스터링
#   - create_pipe_group()          : 군집→통계 레코드 포맷팅 (바운딩박스, BOP 등)
#   - analyze_json(file_path, mode): 전체 파이프라인 실행 (파싱→분류→클러스터링)
#
# [주요 전역 변수]
#   - TOL_Z_ELEVATION (100mm)      : 수평 배관 판단 Z축 허용 오차
#   - TOL_ANGLE_DEG (3°)           : 평행 판단 최대 허용 각도
#   - MAX_SPACING (300mm)          : 그룹 내 인접 배관 최대 허용 간격
#   - MAX_LONGITUDINAL_GAP (1000mm): 길이 방향 겹침/이격 한계치
#   - TOL_ALIGNMENT (100mm)        : 정렬 축 이탈 오차 범위
#   - MIN_GROUP_SIZE               : 유의미한 그룹 최소 배관 수
# =====================================================================================


import json
import os
import math
import sys
import glob
import argparse
from collections import deque
from datetime import datetime
import csv
from collections import defaultdict

# ==============================================================================
# 전역 변수 및 설정 (Algorithm Constants)
# ==============================================================================
TOL_Z_ELEVATION = 100.0         # 수평 배관 판단 시 Z축(높이) 허용 오차 (mm)
TOL_ANGLE_DEG = 3.0             # 두 배관이 평행하다고 판단할 최대 허용 각도 (Degree)
MAX_SPACING = 300.0             # 배관 그룹 내 인접 배관 간의 최대 허용 간격 (mm)
MAX_LONGITUDINAL_GAP = 1000.0   # 배관 길이 방향으로 겹치거나 떨어져도 같은 그룹으로 볼 수 있는 한계치 (mm)
TOL_ALIGNMENT = 100.0           # 정렬 축에서 벗어날 수 있는 오차 범위 (mm)
MIN_GROUP_SIZE = 1              # 유의미한 '그룹'으로 인정하기 위한 최소 배관 개수

class PipeData:
    """
    배관의 기하학적 정보와 속성을 관리하는 클래스
    변수:
        - edge: JSON에서 읽어온 원본 데이터 (dict)
        - p1, p2: 배관의 양 끝점 좌표 (list of float)
        - midpoint: 배관의 중심점 좌표 (list of float)
        - length: 배관의 길이 (float)
        - direction: 배관의 방향 벡터 (list of float, normalized)
        - utility: 배관에 지정된 유틸리티 이름 (str)
    """
    def __init__(self, edge, p1, p2):
        self.edge = edge
        self.p1 = p1
        self.p2 = p2
        self.midpoint = [(p1[0]+p2[0])/2.0, (p1[1]+p2[1])/2.0, (p1[2]+p2[2])/2.0]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        self.length = math.sqrt(dx*dx + dy*dy + dz*dz)
        if self.length > 0:
            self.direction = [dx/self.length, dy/self.length, dz/self.length]
        else:
            self.direction = [0,0,0]
            
        # Unify direction
        d0, d1, d2 = self.direction
        if d0 < 0 or (abs(d0) < 1e-4 and d1 < 0) or (abs(d0) < 1e-4 and abs(d1) < 1e-4 and d2 < 0):
            self.direction = [-d0, -d1, -d2]
            self.p1, self.p2 = self.p2, self.p1
            
        self.utility = edge.get("utility", "Unknown") or "Unknown"
        self.bay = edge.get("bay") or "N/A"

# ==============================================================================
# 수학 및 벡터 헬퍼 함수
# ==============================================================================
def dot_product(v1, v2):
    """ 두 3D 벡터 v1, v2의 내적(Dot Product)을 반환합니다. """
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

def normalize(v):
    """ 정규화된(길이가 1인) 방향 벡터를 반환합니다. """
    mag = math.sqrt(dot_product(v, v))
    if mag > 0:
        return [v[0]/mag, v[1]/mag, v[2]/mag]
    return [0,0,0]

def group_by_tolerance(items, selector, tolerance):
    """
    단일 축(1D) 값들에 대한 허용 오차(Tolerance) 그룹핑 함수
    :param items: 원본 배관 리스트
    :param selector: 비교할 값을 추출하는 람다 함수 (예: p.midpoint[0] X값 기준)
    :param tolerance: 허용하는 최대 오차. (정확히는 tolerance*2 범위로 판정)
    """
    if not items:
        return []
    sorted_items = sorted(items, key=selector)
    results = []
    current_group = [sorted_items[0]]
    base_value = selector(sorted_items[0])
    
    for i in range(1, len(sorted_items)):
        val = selector(sorted_items[i])
        if abs(val - base_value) <= tolerance * 2.0:
            current_group.append(sorted_items[i])
        else:
            results.append(current_group)
            current_group = [sorted_items[i]]
            base_value = val
    results.append(current_group)
    return results

def group_by_direction(items, angle_tolerance_deg):
    """
    배관들을 방향 벡터(Direction)가 유사한(평행한) 것끼리 그룹핑합니다.
    알고리즘:
        - 기준 배관과 다른 배관들 사이의 내적(Dot Product)을 구함
        - 내적 값이 Cos(허용각도) 이상이면 동일 방향으로 판단
    """
    if not items:
        return []
    cos_tol = math.cos(math.radians(angle_tolerance_deg))
    results = []
    remaining = list(items)
    
    while remaining:
        base_pipe = remaining[0]
        current_group = []
        next_remaining = []
        
        for pipe in remaining:
            # 내적이 cos(각도공차) 이상이면 임계치보다 두 벡터의 각이 작다는 뜻 (평행)
            if dot_product(base_pipe.direction, pipe.direction) >= cos_tol:
                current_group.append(pipe)
            else:
                next_remaining.append(pipe)
        
        results.append(current_group)
        remaining = next_remaining
        
    return results

def cluster_vertical_pipes(items, normal, max_lateral_gap, max_longitudinal_gap):
    """
    입상(수직) 배관 클러스터링 알고리즘
    매개변수:
        - items: 분석 대상 배관 리스트
        - normal: 정렬 기준 평면의 법선 벡터
        - max_lateral_gap: 허용되는 최대 측면 이격 거리
        - max_longitudinal_gap: 허용되는 최대 상하 겹침/이격 거리
    """
    clusters = []
    remaining = list(items)
    
    while remaining:
        cluster = []
        queue = deque([remaining.pop(0)])
        
        while queue:
            current = queue.popleft()
            cluster.append(current)
            
            # 현재 배관의 상하(Z) 범위와 정렬 오차 계산
            c_min_proj = min(current.p1[2], current.p2[2])
            c_max_proj = max(current.p1[2], current.p2[2])
            c_offset = dot_product(current.midpoint, normal)
            
            i = len(remaining) - 1
            while i >= 0:
                candidate = remaining[i]
                t_min_proj = min(candidate.p1[2], candidate.p2[2])
                t_max_proj = max(candidate.p1[2], candidate.p2[2])
                t_offset = dot_product(candidate.midpoint, normal)
                
                # 간격 및 겹침 조건 확인
                lateral_gap = abs(c_offset - t_offset)
                overlap_gap = max(c_min_proj, t_min_proj) - min(c_max_proj, t_max_proj)
                
                # 다른 축(X 또는 Y)에서의 정렬 오차 확인
                c_other = current.midpoint[1] if abs(normal[0]) > 0.5 else current.midpoint[0]
                t_other = candidate.midpoint[1] if abs(normal[0]) > 0.5 else candidate.midpoint[0]
                other_gap = abs(c_other - t_other)
                
                # 모든 조건 만족 시 같은 그룹으로 병합
                if lateral_gap <= max_lateral_gap and overlap_gap <= max_longitudinal_gap and other_gap <= TOL_ALIGNMENT * 2.0:
                    queue.append(candidate)
                    remaining.pop(i)
                i -= 1
        clusters.append(cluster)
    return clusters

def cluster_horizontal_pipes(items, directory, normal, max_lateral_gap, max_longitudinal_gap):
    """
    수평 배관들에 대한 밀집도 기반 클러스터링 (공간 군집화)
    알고리즘:
        - BFS(Queue)를 사용하여 인접 조건을 만족하는 배관을 계속 확장 탐색
        - 조건: 측면 간격 <= max_lateral_gap AND 길이 겹침 <= max_longitudinal_gap AND 높이 차이 <= 공차
    """
    clusters = []
    remaining = list(items)
    
    while remaining:
        cluster = []
        queue = deque([remaining.pop(0)])
        
        while queue:
            current = queue.popleft()
            cluster.append(current)
            
            c_min_proj = min(dot_product(current.p1, directory), dot_product(current.p2, directory))
            c_max_proj = max(dot_product(current.p1, directory), dot_product(current.p2, directory))
            c_offset = dot_product(current.midpoint, normal)
            
            i = len(remaining) - 1
            while i >= 0:
                candidate = remaining[i]
                t_min_proj = min(dot_product(candidate.p1, directory), dot_product(candidate.p2, directory))
                t_max_proj = max(dot_product(candidate.p1, directory), dot_product(candidate.p2, directory))
                t_offset = dot_product(candidate.midpoint, normal)
                
                lateral_gap = abs(c_offset - t_offset)
                overlap_gap = max(c_min_proj, t_min_proj) - min(c_max_proj, t_max_proj)
                vertical_gap = abs(current.midpoint[2] - candidate.midpoint[2])
                
                if lateral_gap <= max_lateral_gap and overlap_gap <= max_longitudinal_gap and vertical_gap <= TOL_ALIGNMENT * 2.0:
                    queue.append(candidate)
                    remaining.pop(i)
                i -= 1
        clusters.append(cluster)
    return clusters

def create_pipe_group(utility, g_type, axis, chunk, space_info):
    """
    클러스터링된 배관 묶음을 분석하여 최종 통계 데이터를 생성합니다.
    계산 항목:
        - 배관 수 (PipeCount): 고유 트랙(Track) 수 기준
        - 평균 간격 (Spacing)
        - 3D 바운더리 크기 (BoundWidth/Depth/Height)
        - 3D 바운더리 좌표 (MinX/Y/Z, MaxX/Y/Z)
        - 공간 정보 (Bay, Level, BOP)
    """
    first = chunk[0]
    spacing = 0.0
    pipe_count = 1
    
    if len(chunk) > 1:
        if g_type == "입상":
            if axis == "X축 정렬":
                offsets = [p.midpoint[0] for p in chunk]
            else:
                offsets = [p.midpoint[1] for p in chunk]
        else:
            c_dir = chunk[0].direction
            normal = normalize([-c_dir[1], c_dir[0], 0])
            offsets = [dot_product(p.midpoint, normal) for p in chunk]
            
        unique_offsets = sorted(list(set([round(off / 10.0) * 10.0 for off in offsets])))
        pipe_count = len(unique_offsets) if len(unique_offsets) > 0 else 1
        
        if len(unique_offsets) > 1:
            spacing = (unique_offsets[-1] - unique_offsets[0]) / (len(unique_offsets) - 1)
            
    min_x = min(min(p.p1[0], p.p2[0]) for p in chunk)
    max_x = max(max(p.p1[0], p.p2[0]) for p in chunk)
    min_y = min(min(p.p1[1], p.p2[1]) for p in chunk)
    max_y = max(max(p.p1[1], p.p2[1]) for p in chunk)
    min_z = min(min(p.p1[2], p.p2[2]) for p in chunk)
    max_z = max(max(p.p1[2], p.p2[2]) for p in chunk)
    
    # Calculate Elevation (높이)
    elevation = round(min(first.p1[2], first.p2[2]) if g_type == "입상" else first.midpoint[2], 2)
    
    # Determine Level (LevelName) and BOP from SpaceInfo
    level_name = "N/A"
    bop_value = 0.0
    
    if space_info:
        for space in space_info:
            boundary = space.get("boundary", {})
            lvl_min_z = boundary.get("min", {}).get("z")
            lvl_max_z = boundary.get("max", {}).get("z")
            
            if lvl_min_z is not None and lvl_max_z is not None:
                if lvl_min_z <= elevation <= lvl_max_z:
                    level_name = space.get("levelName") or "N/A"
                    bop_value = round(elevation - lvl_min_z, 2)
                    break

    # spatial data fallback if not found in SpaceInfo (but prioritize SpaceInfo names)
    bay = first.bay if hasattr(first, 'bay') and first.bay != "N/A" else (first.edge.get("bay") or "N/A")

    return {
        "Utility": utility,
        "GroupType": g_type,
        "AlignmentAxis": axis,
        "PipeCount": pipe_count,
        "Spacing": round(spacing, 2),
        "Length": round(first.length, 2),
        "Elevation": elevation,
        "BoundWidthX": round(max_x - min_x, 2),
        "BoundDepthY": round(max_y - min_y, 2),
        "BoundHeightZ": round(max_z - min_z, 2),
        "MinX": round(min_x, 2),
        "MaxX": round(max_x, 2),
        "MinY": round(min_y, 2),
        "MaxY": round(max_y, 2),
        "MinZ": round(min_z, 2),
        "MaxZ": round(max_z, 2),
        "Bay": bay,
        "Level": level_name,
        "BOP": bop_value,
        "EdgeIds": [p.edge.get("id") or p.edge.get("guid") for p in chunk]
    }

def group_vertical_pipes(pipes, utility, space_info):
    """ 입상(수직) 배관 전체를 정렬 방식에 따라 클러스터로 분리하고 최종 Group Dict 리스트로 반환 """
    results = []
    if len(pipes) < MIN_GROUP_SIZE:
        return results
        
    # X 좌표 기준 (Y축 정렬 도출)
    x_groups = group_by_tolerance(pipes, lambda p: p.midpoint[0], TOL_ALIGNMENT)
    for xG in x_groups:
        if len(xG) < MIN_GROUP_SIZE:
            continue
        clusters = cluster_vertical_pipes(xG, [0, 1, 0], MAX_SPACING, MAX_LONGITUDINAL_GAP)
        for chunk in clusters:
            unique_tracks = set([round(p.midpoint[1] / 10.0) * 10.0 for p in chunk])
            if len(unique_tracks) >= MIN_GROUP_SIZE:
                sorted_chunk = sorted(chunk, key=lambda p: p.midpoint[1])
                results.append(create_pipe_group(utility, "입상", "Y축 정렬", sorted_chunk, space_info))
                
    # Y 좌표 기준 (X축 정렬 도출)
    y_groups = group_by_tolerance(pipes, lambda p: p.midpoint[1], TOL_ALIGNMENT)
    for yG in y_groups:
        if len(yG) < MIN_GROUP_SIZE:
            continue
        clusters = cluster_vertical_pipes(yG, [1, 0, 0], MAX_SPACING, MAX_LONGITUDINAL_GAP)
        for chunk in clusters:
            unique_tracks = set([round(p.midpoint[0] / 10.0) * 10.0 for p in chunk])
            if len(unique_tracks) >= MIN_GROUP_SIZE:
                sorted_chunk = sorted(chunk, key=lambda p: p.midpoint[0])
                results.append(create_pipe_group(utility, "입상", "X축 정렬", sorted_chunk, space_info))
                
    return results

def group_horizontal_pipes(pipes, utility, space_info):
    """ 수평(Horizontal) 배관들을 높이 고도(z) 및 방향 벡터에 따라 묶고 최종 Group Dict 리스트로 반환 """
    results = []
    if len(pipes) < MIN_GROUP_SIZE:
        return results
        
    # 동일 높이의 배관들을 묶어 Z축(입면도) 차원을 해소
    z_groups = group_by_tolerance(pipes, lambda p: p.midpoint[2], TOL_Z_ELEVATION)
    
    for zG in z_groups:
        # 내적 공차 기준을 이용한 방향 벡터 그룹핑
        dir_groups = group_by_direction(zG, TOL_ANGLE_DEG)
        for dG in dir_groups:
            if len(dG) < MIN_GROUP_SIZE:
                continue
            
            c_dir = dG[0].direction
            normal = [-c_dir[1], c_dir[0], 0]
            mag_sq = normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]
            if mag_sq < 1e-6:
                continue
            normal = normalize(normal)
            
            clusters = cluster_horizontal_pipes(dG, c_dir, normal, MAX_SPACING, MAX_LONGITUDINAL_GAP)
            for chunk in clusters:
                # 측면으로 산개한 트랙이 지정한 개수 이상인지 필터링
                unique_tracks = set([round(dot_product(p.midpoint, normal) / 10.0) * 10.0 for p in chunk])
                if len(unique_tracks) >= MIN_GROUP_SIZE:
                    sorted_chunk = sorted(chunk, key=lambda p: dot_product(p.midpoint, normal))
                    axis = "Y축 정렬" if abs(c_dir[0]) > abs(c_dir[1]) else "X축 정렬"
                    results.append(create_pipe_group(utility, "수평", axis, sorted_chunk, space_info))
                    
    return results

def analyze_json(file_path, mode="equipment"):
    """
    JSON 파일을 읽어서 배관 그룹을 분석하는 핵심 로직
    매개변수:
        - file_path: JSON 파일 경로
        - mode: "equipment" (장비 기준 탐색) 또는 "utility" (전체 배관 대상)
    반환값:
        - 분석된 장비별/유틸리티별 그룹 리스트
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
        
    nodes = data.get("Nodes", data.get("nodes", []))
    edges = data.get("Edges", data.get("edges", []))
    equipments = data.get("Equipment", data.get("equipment", []))
    
    node_positions = {}
    node_dict = {}
    for n in nodes:
        guid = n.get("guid") or n.get("Guid")
        pos = n.get("position") or n.get("Position")
        if guid:
            node_dict[guid] = n
            if pos:
                if isinstance(pos, list) and len(pos) >= 3:
                    node_positions[guid] = [pos[0], pos[1], pos[2]]
                elif isinstance(pos, dict):
                    node_positions[guid] = [pos.get("x") or 0, pos.get("y") or 0, pos.get("z") or 0]
            
    all_pipes = []
    
    # Extract SpaceInfo for Level/BOP calculation
    space_info = data.get("FileInfo", {}).get("SpaceInfo", [])
    
    for edge in edges:
        guids = edge.get("connectionGuidList", [])
        if not guids or len(guids) < 2:
            continue
        guid1, guid2 = guids[0], guids[1]
        if guid1 in node_positions and guid2 in node_positions:
            p1 = node_positions[guid1]
            p2 = node_positions[guid2]
            pipe_data = PipeData(edge, p1, p2)
            if pipe_data.length >= 100.0:
                all_pipes.append(pipe_data)

    vertical_cos_threshold = math.cos(math.radians(TOL_ANGLE_DEG))
    horizontal_sin_threshold = math.sin(math.radians(TOL_ANGLE_DEG))
    
    equipment_results = []

    # 전체 장비(Equipment) 목록을 순회
    if isinstance(equipments, dict):
        equipments = [equipments]

    if mode == "equipment":
        for eq in equipments:
            eq_name = eq.get("name") or eq.get("Name") or "Unknown_Equipment"
            eq_guid = eq.get("guid") or eq.get("Guid") or eq.get("id") or eq.get("Id")
            eq_id = eq.get("id") or eq.get("Id") or "N/A"
            eq_process = eq.get("process") or eq.get("Process") or "N/A"
            eq_maker = eq.get("maker") or eq.get("Maker") or "N/A"
            eq_type = eq.get("type") or eq.get("Type") or "N/A"
            
            if not eq_guid:
                continue

            eq_poc_guids = set()
            for p in eq.get("pocList", []):
                p_guid = p.get("guid") or p.get("id") or p.get("Id")
                if p_guid:
                    eq_poc_guids.add(p_guid)

            for edge in edges:
                guids = edge.get("connectionGuidList", [])
                if len(guids) < 2: continue
                edge_type = edge.get("type", "").upper()
                if edge_type == "DIRECT" and eq_guid in guids:
                    other_guid = guids[0] if guids[1] == eq_guid else guids[1]
                    eq_poc_guids.add(other_guid)

            # --- 장비와 연결된 모든 노드/배관을 찾는 BFS 탐색 시작 ---
            visited_nodes = set([eq_guid]) | eq_poc_guids
            visited_edges = set()
            queue = deque(list(visited_nodes))
            
            while queue:
                curr = queue.popleft()
                for edge in edges:
                    guids = edge.get("connectionGuidList", [])
                    if len(guids) < 2: continue
                    e_id = edge.get("id") or edge.get("guid")
                    
                    if e_id in visited_edges: continue
                    
                    # 현재 노드가 배관의 한쪽 끝이면 해당 배관을 추적 대상에 추가
                    if curr in guids:
                        visited_edges.add(e_id)
                        nxt = guids[0] if guids[1] == curr else guids[1]
                        if nxt not in visited_nodes:
                            visited_nodes.add(nxt)
                            queue.append(nxt)

            # 탐색된 Edge ID들을 기반으로 PipeData 객체 필터링
            eq_pipes = [p for p in all_pipes if (p.edge.get("id") or p.edge.get("guid")) in visited_edges]
            
            if not eq_pipes:
                continue

            vertical_pipes = []
            horizontal_pipes = []

            for pipe in eq_pipes:
                z_dot = abs(pipe.direction[2])
                if z_dot >= vertical_cos_threshold:
                    vertical_pipes.append(pipe)
                elif z_dot <= horizontal_sin_threshold:
                    horizontal_pipes.append(pipe)
                    
            vert_by_utility = defaultdict(list)
            for p in vertical_pipes:
                vert_by_utility[p.utility].append(p)
                
            horiz_by_utility = defaultdict(list)
            for p in horizontal_pipes:
                horiz_by_utility[p.utility].append(p)
                
            groups = []
            for u, pipes in vert_by_utility.items():
                groups.extend(group_vertical_pipes(pipes, u, space_info))
                
            for u, pipes in horiz_by_utility.items():
                groups.extend(group_horizontal_pipes(pipes, u, space_info))
                
            # 중복 그룹 제거 (다른 그룹의 부분집합인 경우 제외)
            distinct_groups = []
            for g in sorted(groups, key=lambda x: x["PipeCount"], reverse=True):
                is_subset = False
                for existing in distinct_groups:
                    if existing["Utility"] == g["Utility"] and existing["GroupType"] == g["GroupType"]:
                        if set(g["EdgeIds"]).issubset(set(existing["EdgeIds"])):
                            is_subset = True
                            break
                if not is_subset:
                    distinct_groups.append(g)

            if distinct_groups:
                equipment_results.append({
                    "EquipmentName": eq_name,
                    "EquipmentId": eq_id,
                    "Process": eq_process,
                    "Maker": eq_maker,
                    "Type": eq_type,
                    "Groups": distinct_groups
                })
    else:  # mode == "utility"
        vertical_pipes = []
        horizontal_pipes = []
        for pipe in all_pipes:
            z_dot = abs(pipe.direction[2])
            if z_dot >= vertical_cos_threshold:
                vertical_pipes.append(pipe)
            elif z_dot <= horizontal_sin_threshold:
                horizontal_pipes.append(pipe)
                
        vert_by_utility = defaultdict(list)
        for p in vertical_pipes:
            vert_by_utility[p.utility].append(p)
            
        horiz_by_utility = defaultdict(list)
        for p in horizontal_pipes:
            horiz_by_utility[p.utility].append(p)
            
        groups = []
        for u, pipes in vert_by_utility.items():
            groups.extend(group_vertical_pipes(pipes, u, space_info))
            
        for u, pipes in horiz_by_utility.items():
            groups.extend(group_horizontal_pipes(pipes, u, space_info))
            
        distinct_groups = []
        for g in sorted(groups, key=lambda x: x["PipeCount"], reverse=True):
            is_subset = False
            for existing in distinct_groups:
                if existing["Utility"] == g["Utility"] and existing["GroupType"] == g["GroupType"]:
                    if set(g["EdgeIds"]).issubset(set(existing["EdgeIds"])):
                        is_subset = True
                        break
            if not is_subset:
                distinct_groups.append(g)

        if distinct_groups:
            equipment_results.append({
                "EquipmentName": "전체(util_only)",
                "EquipmentId": "N/A",
                "Process": "N/A",
                "Maker": "N/A",
                "Type": "N/A",
                "Groups": distinct_groups
            })
            
    return equipment_results

def main():
    """ 
    진입점 함수 
    - 실행 환경 인자(Argv) 검증
    - 와일드카드를 통한 파일 스캔 목록화 (Glob)
    - 결과 데이터 취합 및 CSV File Write 루틴
    """
    global TOL_Z_ELEVATION, TOL_ANGLE_DEG, MAX_SPACING, MAX_LONGITUDINAL_GAP, TOL_ALIGNMENT, MIN_GROUP_SIZE
    parser = argparse.ArgumentParser(description="Analyze group pipes from JSON files.")
    parser.add_argument("directory_path", help="Path to the directory containing JSON files.")
    parser.add_argument("--mode", choices=["equipment", "utility"], default="equipment", 
                        help="Analysis mode: 'equipment' (Equipment + Utility) or 'utility' (Utility Only)")
    
    # 알고리즘 상수 설정을 위한 추가 옵션
    parser.add_argument("--tol_z", type=float, default=TOL_Z_ELEVATION, 
                        help=f"TOL_Z_ELEVATION: 수평 배관 판단 시 Z축(높이) 허용 오차 (default: {TOL_Z_ELEVATION} mm)")
    parser.add_argument("--tol_angle", type=float, default=TOL_ANGLE_DEG, 
                        help=f"TOL_ANGLE_DEG: 평행 여부 판단 최대 허용 각도 (default: {TOL_ANGLE_DEG} Deg)")
    parser.add_argument("--max_spacing", type=float, default=MAX_SPACING, 
                        help=f"MAX_SPACING: 그룹 내 인접 배관 간 최대 허용 간격 (default: {MAX_SPACING} mm)")
    parser.add_argument("--max_gap", type=float, default=MAX_LONGITUDINAL_GAP, 
                        help=f"MAX_LONGITUDINAL_GAP: 길이 방향 겹침/이격 한계치 (default: {MAX_LONGITUDINAL_GAP} mm)")
    parser.add_argument("--tol_align", type=float, default=TOL_ALIGNMENT, 
                        help=f"TOL_ALIGNMENT: 정렬 축 오차 범위 (default: {TOL_ALIGNMENT} mm)")
    parser.add_argument("--min_group_count", type=int, default=MIN_GROUP_SIZE, 
                        help=f"MIN_GROUP_SIZE: 그룹 인정 최소 배관 개수 (default: {MIN_GROUP_SIZE})")

    args = parser.parse_args()
    
    # 전역변수 업데이트 (사용자 입력값 반영)
    TOL_Z_ELEVATION = args.tol_z
    TOL_ANGLE_DEG = args.tol_angle
    MAX_SPACING = args.max_spacing
    MAX_LONGITUDINAL_GAP = args.max_gap
    TOL_ALIGNMENT = args.tol_align
    MIN_GROUP_SIZE = args.min_group_count

    target_dir = args.directory_path
    mode = args.mode
    
    if not os.path.exists(target_dir):
        print(f"Error: Directory {target_dir} not found.")
        sys.exit(1)
        
    all_results = []
    
    # Recursively find json files
    search_pattern = os.path.join(target_dir, "**", "*.json")
    json_files = glob.glob(search_pattern, recursive=True)
    
    print(f"Found {len(json_files)} JSON files in {target_dir}")
    
    for file_path in json_files:
        try:
            print(f"Analyzing {file_path} in '{mode}' mode...")
            eq_results = analyze_json(file_path, mode=mode)
            file_name = os.path.basename(file_path)
            
            for eq_data in eq_results:
                eq_name = eq_data["EquipmentName"]
                for g in eq_data["Groups"]:
                    # 집계된 데이터를 딕셔너리 형태로 리스트에 추가 (사용자 요청 순서 반영)
                    all_results.append({
                        "파일명": file_name,
                        "장비명": eq_name,
                        "장비ID": eq_data["EquipmentId"],
                        "Process": eq_data["Process"],
                        "Maker": eq_data["Maker"],
                        "장비유형": eq_data["Type"],
                        "유틸리티": g["Utility"],
                        "Bay": g["Bay"],
                        "Level": g["Level"],
                        "입상/수평": g["GroupType"],
                        "배관수": g["PipeCount"],
                        "간격": g["Spacing"],
                        "높이": g["Elevation"],
                        "BOP": g["BOP"],
                        "LeftX": g["MinX"],
                        "LeftY": g["MinY"],
                        "LeftZ": g["MinZ"],
                        "RightX": g["MaxX"],
                        "RightY": g["MaxY"],
                        "RightZ": g["MaxZ"],
                        "bound너비": g["BoundWidthX"],
                        "bound깊이": g["BoundDepthY"],
                        "bound두께": g["BoundHeightZ"]
                    })
        except Exception as e:
            print(f"Error processing {file_path}: {e}")
            
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    dir_name = os.path.basename(os.path.normpath(target_dir))
    mode_suffix = "equipment+util" if mode == "equipment" else "util_only"
    output_csv = f"group_rule_{dir_name}_{timestamp}_{mode_suffix}.csv"
    
    if all_results:
        # 사용자 요청에 따른 CSV 컬럼 순서 정의
        fieldnames = ["파일명", "장비명", "장비ID", "Process", "Maker", "장비유형", "유틸리티", "Bay", "Level", "입상/수평", 
                "배관수", "간격", "높이", "BOP", "LeftX", "LeftY", "LeftZ", "RightX", "RightY", "RightZ", 
                "bound너비", "bound깊이", "bound두께"]
        try:
            # utf-8-sig ensures Excel opens it correctly with Korean characters
            with open(output_csv, mode='w', newline='', encoding='utf-8-sig') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(all_results)
            print(f"Results saved to {output_csv}")

            # --- Excel 파일로도 저장 (pandas 활용) ---
            try:
                import pandas as pd
                output_xlsx = output_csv.replace(".csv", ".xlsx")
                df = pd.DataFrame(all_results)
                # 컬럼 순서 보정 (CSV와 동일하게)
                df = df[fieldnames]
                df.to_excel(output_xlsx, index=False)
                print(f"Results also saved to {output_xlsx}")
            except ImportError:
                print("Note: 'pandas' and 'openpyxl' are required for Excel export. Only CSV was saved.")
            except Exception as e:
                print(f"Excel export failed: {e}")

        except Exception as e:
            print(f"Error saving to CSV: {e}")

        except Exception as e:
            print(f"Error saving to CSV: {e}")
    else:
        print("No pipe groups found.")

if __name__ == "__main__":
    main()
