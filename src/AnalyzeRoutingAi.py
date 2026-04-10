"""
AnalyzeRoutingAi.py
======================
3D 배관 설계 데이터(JSON)에서 개별 배관 경로를 추출(Phase 1)하고, 
추출된 경로들 사이의 물리적/기하학적 유사성을 분석하여 그룹 배관(Trunk/Bundle)을 탐지(Phase 2)하는 통합 솔루션입니다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[전체 처리 흐름 (Workflow)]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Phase 1: 개별 배관 경로 탐색 (Routing Analysis)
   - data-v10 디렉토리의 원본 JSON(Nodes, Edges, Equipment)을 로드합니다.
   - 각 장비(Equipment)의 POC를 시작점으로 설정합니다.
   - BFS(너비 우선 탐색) 알고리즘을 사용하여 종단점(Duct, Nozzle 등)까지의 모든 가능한 경로를 찾습니다.
   - 탐색된 경로 정보를 ./RoutingResults 폴더에 장비별 JSON 파일로 저장합니다.

2. Phase 2: 그룹 배관 패턴 분석 (Group Pattern Analysis)
   - Phase 1에서 생성된 모든 경로 데이터를 다시 로드합니다.
   - 동일 장비 + 동일 유틸리티(Gas/Liquid 타입)를 사용하는 배관들을 그룹 후보군으로 묶습니다.
   - 3단계 필터링 알고리즘을 적용하여 실제 그룹(번들)을 판별합니다.
     ① 패턴 필터: 경로의 꺾임 형태(path_arrow) 유사도 검사 (Levenshtein 알고리즘)
     ② 공간 필터: 시작점(POC) 간의 거리 및 수평 구간(H)의 고도(Z) 일치 여부 검사
     ③ 구간 탐지: 주경로(TRUNK)와 분산되는 구간(FAN-IN/OUT)의 바운딩 박스 산출
   - 최종 분석 결과를 ./GroupPipeResults 폴더에 JSON 및 CSV 형식으로 저장합니다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[핵심 알고리즘 (Core Algorithms)]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. 경로 탐색 (BFS): 
   - 부모 포인터(Parent Pointer) 방식을 사용하여 메모리 효율성을 극대화했습니다. 
   - 사이클 방지를 위해 '엣지 방문 집합'과 '부모 역추적'의 이중 검증을 수행합니다.

2. 패턴 유사도 (Levenshtein Distance):
   - 배관의 방향 코드(R:수직, H:수평, D:경사)를 문자열로 변환한 뒤, 두 경로 간의 편집 거리를 계산합니다.
   - 1.0(완전 일치) ~ 0.0(완전 불일치) 사이의 값으로 변환하여 유사도를 측정합니다.

3. 클러스터링 (Union-Find):
   - 유사도 임계값을 넘는 경로들을 효율적으로 하나의 그룹으로 병합하기 위해 사용합니다.
"""

import json
import math
import os
import glob
import argparse
import csv
import itertools
from collections import deque, defaultdict
from datetime import datetime

# ─────────────────────────────────────────────────────────────
# 0. 설정 상수 (Settings / Parameters)
# ─────────────────────────────────────────────────────────────

# --- Phase 1: Routing (경로 탐색 관련) ---
MAX_BRANCH_COUNT = 8              # 단일 노드에서 허용되는 최대 분기(Branch) 수. 초과 시 데이터 이상 경고.
DIRECTION_ANGLE_TOLERANCE = 5.0    # 방향 분류 임계값(도°). 수평(0) 또는 수직(90)에서 이 범위 내면 해당 방향으로 간주.

# --- Phase 2: Grouping (그룹 분석 관련) ---
PATTERN_SIMILARITY_MIN = 0.70      # 그룹으로 인정할 최소 패턴 유사도 (70% 이상)
START_POC_XY_MAX = 5000.0          # 그룹 후보 배관들의 시작점(POC) 간 평면(XY) 최대 허용 거리 (mm)
TOL_Z_ELEVATION = 100.0            # 수평 배관 고도 판별 시 허용 오차 (mm)
TOL_Z_LEVEL = 200.0                # 수평 구간(H)이 동일한 높이라고 판단할 최대 오차 범위 (mm)
MAX_SPACING = 300.0                # 배관 간 최대 허용 간격 (mm)
TOL_ANGLE_DEG = 3.0                # 평행 판단 최대 허용 각도 (Degree)
TRUNK_MAX_XY_SPREAD = 1500.0       # 주경로(TRUNK) 구간에서 배관들이 벌어질 수 있는 최대 XY 거리 (mm)
TRUNK_Z_BAND_FACTOR = 2.0          # TRUNK 구간 노드들을 수집할 때 Z 허용 오차의 배수
MIN_GROUP_SIZE = 2                 # 최소 몇 개 이상의 배관이 뭉쳐야 그룹으로 인정할지 결정

# --- 경로 기본값 (Default Paths) ---
DEFAULT_INPUT_DIR   = "./data-v10"
DEFAULT_ROUTING_DIR = "./RoutingResults"
DEFAULT_GROUP_DIR    = "./GroupPipeResults"

# 배관 요소 타입 약어 (출력 요약용)
TYPE_ABBREV = {
    "ELBOW": "EB", "TEE": "TE", "REDUCER": "RD", "UNION": "UN", "FLANGE": "FL",
    "ENDCAP": "EC", "CONNECTOR": "CN", "SOCKET": "SK", "BENDING": "BD", "CLAMP": "CL",
    "GLAND": "GL", "GASKET": "GK", "BELLOWS": "BL", "VALVE": "VL", "FILTER": "FI",
    "REGULATOR": "RG", "DAMPER": "DA", "DAMPER_DUCT": "DD", "POC": "PO", "TAKEOFF": "TK",
    "LATERAL PIPE": "LP", "LATERAL": "LA", "DUCT": "DT", "EQUIPMENT": "EQ", "SUB_EQUIPMENT": "SE",
    "BRANCH": "BR", "JUNCTION": "JN", "CROSS": "CR", "WYE": "WY", "ETC": "ET", "DIRECT_NODE": "DN"
}

# ─────────────────────────────────────────────────────────────
# 1. 공통 유틸리티 (Common Utilities)
# ─────────────────────────────────────────────────────────────

def _get_guid(obj):
    """
    JSON 객체에서 고유 식별자(GUID)를 추출합니다.
    다양한 데이터 형식(guid, Guid, id, Id)에 대응합니다.
    """
    if not obj: return None
    return obj.get("guid") or obj.get("Guid") or obj.get("id") or obj.get("Id")

def _get_position(node_data):
    """
    노드 데이터에서 (x, y, z) 좌표 튜플을 추출하여 반환합니다.
    """
    pos = node_data.get("position")
    if isinstance(pos, (list, tuple)) and len(pos) >= 3:
        return float(pos[0]), float(pos[1]), float(pos[2])
    x = node_data.get("x", node_data.get("X", 0.0))
    y = node_data.get("y", node_data.get("Y", 0.0))
    z = node_data.get("z", node_data.get("Z", 0.0))
    return float(x), float(y), float(z)

def _dist_xy(p1, p2):
    """두 3D 좌표 사이의 평면(XY) 유클리드 거리를 계산합니다."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def _xy_spread(positions):
    """좌표 목록 중에서 가장 멀리 떨어진 두 점 사이의 XY 거리를 반환합니다. (분산도 측정)"""
    if len(positions) < 2: return 0.0
    return max(_dist_xy(a, b) for a, b in itertools.combinations(positions, 2))

def _bbox(positions):
    """
    주어진 좌표 목록을 모두 포함하는 3D 바운딩 박스(BBox) 정보를 생성합니다.
    결과: x_min/max, y_min/max, z_min/max 및 분산도 포함.
    """
    if not positions: return None
    return {
        "x_min": round(min(p[0] for p in positions), 1),
        "x_max": round(max(p[0] for p in positions), 1),
        "y_min": round(min(p[1] for p in positions), 1),
        "y_max": round(max(p[1] for p in positions), 1),
        "z_min": round(min(p[2] for p in positions), 1),
        "z_max": round(max(p[2] for p in positions), 1),
        "width_x": round(max(p[0] for p in positions) - min(p[0] for p in positions), 1),
        "depth_y": round(max(p[1] for p in positions) - min(p[1] for p in positions), 1),
        "thick_z": round(max(p[2] for p in positions) - min(p[2] for p in positions), 1),
        "xy_spread": round(_xy_spread(positions), 1),
        "node_count": len(positions),
    }

def _compute_path_arrow(positions):
    """
    경로의 꺾임 패턴을 분석하여 R(Riser:수직), H(Header:수평), D(Diagonal:경사) 코드로 변환합니다.
    예: 'R-H-H-D-R' (수직 상승 후 수평 이동, 경사 하강 후 다시 수직 이동하는 형태)
    """
    if len(positions) < 2: return ""
    codes = []
    for i in range(len(positions) - 1):
        x1, y1, z1 = positions[i]
        x2, y2, z2 = positions[i+1]
        dx, dy, dz = x2-x1, y2-y1, z2-z1
        total = math.sqrt(dx*dx + dy*dy + dz*dz)
        if total < 1e-6: continue
        v_angle = math.degrees(math.asin(abs(dz) / total)) # 수평면 대비 수직 각도 계산
        if v_angle > (90.0 - DIRECTION_ANGLE_TOLERANCE): codes.append("R")
        elif v_angle < DIRECTION_ANGLE_TOLERANCE: codes.append("H")
        else: codes.append("D")
    return "-".join(codes)

def _build_path_summary(steps):
    """
    전체 배관 경로 단계를 타입 약어로 요약하여 문자열로 만듭니다.
    예: PO(시작점) -> EB(엘보) -> VL(밸브) -> DT(덕트)
    """
    abbrevs = []
    for s in steps:
        if s["kind"] != "NODE": continue
        node_type = (s["data"].get("type") or "").upper().strip()
        if not node_type: abbrevs.append("??")
        elif node_type in TYPE_ABBREV: abbrevs.append(TYPE_ABBREV[node_type])
        else: abbrevs.append(node_type[:2]) # 미등록 타입은 앞 2자만 사용
    return "->".join(abbrevs)

# ─────────────────────────────────────────────────────────────
# 2. Phase 1: RoutingGraph (경로 탐색 클래스)
# ─────────────────────────────────────────────────────────────

class RoutingGraph:
    """
    배관 네트워크 그래프를 로드하고 BFS 탐색을 통해 경로를 찾는 클래스입니다.
    """
    def __init__(self):
        self.node_by_guid = {}    # 노드 ID -> 노드 객체 매핑 테이블
        self.edge_by_guid = {}    # 엣지 ID -> 엣지 객체 매핑 테이블
        self.equipment_list = []  # 장비(Equipment) 목록
        self.BRANCH_NODE_TYPES = {"TEE", "BRANCH", "JUNCTION", "CROSS", "WYE"}

    def load_from_json(self, file_path):
        """설계 데이터 파일을 읽어 내부 인덱스를 구축합니다."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if isinstance(data, list): return False # 리스트 형식의 루트는 비정상 데이터로 간주
            nodes  = data.get("Nodes", data.get("nodes", []))
            edges  = data.get("Edges", data.get("edges", []))
            equips = data.get("Equipment", data.get("equipment", []))
            for n in nodes:
                g = _get_guid(n)
                if g: self.node_by_guid[g] = n
            for e in edges:
                g = _get_guid(e)
                if g: self.edge_by_guid[g] = e
            self.equipment_list = [equips] if isinstance(equips, dict) else equips
            return True
        except Exception: return False

    def get_neighbors(self, node_guid):
        """특정 노드에서 이동 가능한 인접한 (엣지, 다음 노드) 목록을 가져옵니다."""
        curr_node = self.node_by_guid.get(node_guid)
        if not curr_node: return []
        neighbors = []
        for guid in curr_node.get("connectionGuidList", []):
            edge = self.edge_by_guid.get(guid)
            if edge:
                if edge.get("type", "").upper() == "VIRTUAL": continue # 논리적 연결인 가상 엣지는 무시
                conn = edge.get("connectionGuidList", [])
                if len(conn) >= 2:
                    # 엣지가 참조하는 두 노드 중 현재 노드가 아닌 쪽이 다음 노드임
                    opp = conn[0] if conn[1] == node_guid else conn[1]
                    neighbors.append((edge, opp))
                continue
            # 엣지 객체 없이 직접 연결된 경우 (pseudo-edge 생성)
            if guid in self.node_by_guid:
                neighbors.append(({"type": "DIRECT_NODE", "id": "direct"}, guid))
        return neighbors

    def is_branch_node(self, node_guid):
        """해당 노드가 배관이 갈라지는 지점(분기점)인지 판별합니다."""
        node = self.node_by_guid.get(node_guid)
        if not node: return False
        ntype = node.get("type", "").upper()
        nname = (node.get("name") or node.get("id") or "").upper()
        if ntype in self.BRANCH_NODE_TYPES: return True
        if any(bt in nname for bt in self.BRANCH_NODE_TYPES): return True
        return len(self.get_neighbors(node_guid)) >= 3 # 유효 연결이 3개 이상이면 분기점으로 간주

    def find_routing_paths(self):
        """모든 장비의 POC를 찾고 탐색을 시작하여 최종 경로들을 반환합니다."""
        results = {}
        all_equipment_pocs = set() # 다른 장비의 시작점에 도달했는지 확인하기 위한 용도
        for eq in self.equipment_list:
            for p in eq.get("pocList", []):
                g = _get_guid(p)
                if g: all_equipment_pocs.add(g)

        for eq in self.equipment_list:
            eq_id = _get_guid(eq) or "UnknownID"
            eq_info = {
                "id": eq_id,
                "name": eq.get("name") or eq.get("Name") or "Unknown",
                "process": eq.get("process") or eq.get("Process") or "-",
                "maker": eq.get("maker") or eq.get("Maker") or "-"
            }
            poc_list = []
            for p in eq.get("pocList", []):
                g = _get_guid(p)
                if g and g in self.node_by_guid:
                    poc_list.append(self.node_by_guid[g])
            
            if not poc_list: continue
            eq_results = []
            for poc in poc_list:
                # 각 POC로부터 출발하여 도달 가능한 모든 배관 경로 탐색 (BFS)
                paths = self._trace_paths_from_poc(poc, _get_guid(poc), all_equipment_pocs)
                eq_results.append({"poc": poc, "paths": paths})
            if eq_results:
                results[eq_id] = {"info": eq_info, "results": eq_results}
        return results

    def _trace_paths_from_poc(self, start_node, start_guid, all_start_guids):
        """
        [핵심 알고리즘: BFS 배관 탐색]
        - node_records: 부모 포인터를 저장하여 경로를 역추적할 수 있게 합니다.
        - queue: 탐색할 노드들을 저장하며 사이클 방지를 위한 방문 정보(frozenset)를 함께 관리합니다.
        """
        # node_records 튜플: (현재노드GUID, 노드객체, 부모인덱스, 엣지정보, 분기정보)
        node_records = [(start_guid, start_node, -1, None, {"depth": 0, "segments": []})]
        queue = deque([(start_guid, 0, frozenset())])
        paths_found = []

        while queue:
            curr_guid, r_idx, v_edges = queue.popleft()
            curr_node = self.node_by_guid.get(curr_guid)
            if not curr_node: continue

            # 탐색 종단 조건 검사 (출발 POC를 제외한 지점)
            if curr_guid != start_guid:
                ntype, nname, nid = curr_node.get("type", "").upper(), (curr_node.get("name") or "").upper(), (curr_node.get("id") or "").upper()
                stop, label = False, ""
                
                # 1. 다른 기기의 접속점에 도달한 경우
                if curr_guid in all_start_guids: stop, label = True, "부대장비 PoC 도달"
                # 2. 덕트(Duct) 또는 테이크오프 노드에 도달한 경우
                elif ntype in ("DUCT", "TAKEOFF") or "DUCT" in nid or "DUCT" in nname: stop, label = True, "Duct PoC 도달"
                # 3. 우회 배관이나 말단 배관에 도달한 경우
                elif ntype in ("LATERAL", "LATERAL PIPE") or "LATERAL" in nid or "LATERAL" in nname: stop, label = True, "LateralPipe 도달"
                # 4. 장비의 노즐(Nozzle)에 도달한 경우
                elif "NOZZLE" in nname or "NOZZLE" in nid: stop, label = True, "Nozzle PoC 도달"
                # 5. 명시적으로 'end'라고 표시된 노드인 경우
                elif ntype == "POC" and (curr_node.get("name") == "end" or nid.startswith("END")): stop, label = True, "종단 PoC 도달"
                # 6. 장비 노드 자체에 도달한 경우
                elif ntype == "EQUIPMENT": stop, label = True, "부대장비 도달"
                
                if not stop:
                    neighbors = self.get_neighbors(curr_guid)
                    # 7. 더 이상 연결된 곳이 없는 막다른 골목인 경우
                    if not neighbors: stop, label = True, "배관 끝단(막힘)"
                else: neighbors = []

                if stop:
                    # 종단점 도달 시, node_records를 역추적하여 전체 경로(steps)를 생성하고 저장합니다.
                    _, _, _, _, b_info = node_records[r_idx]
                    path_steps = self._reconstruct_path(node_records, r_idx)
                    paths_found.append({
                        "end_node": curr_node, "label": label, "path": path_steps,
                        "branch_depth": b_info["depth"], "branch_segments": b_info["segments"]
                    })
                    continue # 해당 경로는 여기서 마무리
            else: neighbors = self.get_neighbors(curr_guid)

            # 분기점 처리 및 이웃 노드로 확장
            is_br = (curr_guid != start_guid) and self.is_branch_node(curr_guid)
            _, _, _, _, c_b_info = node_records[r_idx]

            for b_idx, (edge, next_g) in enumerate(neighbors):
                e_sig = _get_guid(edge) or f"direct_{curr_guid}_{next_g}"
                if e_sig in v_edges: continue # 이미 통과한 배관(엣지)은 다시 가지 않음 (사이클 방지 1)
                if self._is_in_path(node_records, r_idx, next_g): continue # 현재 경로상의 노드 재방문 방지 (사이클 방지 2)
                next_node = self.node_by_guid.get(next_g)
                if not next_node: continue

                # 분기 정보 갱신 (깊이 및 통과 지점 기록)
                new_b_info = c_b_info if not is_br else {
                    "depth": c_b_info["depth"] + 1,
                    "segments": c_b_info["segments"] + [{"branch_node_guid": curr_guid, "branch_index": b_idx, "branch_total": len(neighbors)}]
                }
                node_records.append((next_g, next_node, r_idx, edge, new_b_info))
                queue.append((next_g, len(node_records)-1, v_edges | {e_sig}))
        return paths_found

    def _is_in_path(self, node_records, curr_idx, target_g):
        """부모 역추적을 통해 특정 노드가 현재 탐색 경로상에 이미 존재하는지 확인합니다."""
        idx = curr_idx
        while idx >= 0:
            if node_records[idx][0] == target_g: return True
            idx = node_records[idx][2]
        return False

    def _reconstruct_path(self, node_records, end_idx):
        """부모 인덱스를 따라 역추적하여 시작점부터 종점까지의 NODE-EDGE 순서 목록을 복원합니다."""
        path = []
        idx = end_idx
        while idx >= 0:
            path.append({"kind": "NODE", "data": node_records[idx][1]})
            if node_records[idx][3]: path.append({"kind": "EDGE", "data": node_records[idx][3]})
            idx = node_records[idx][2]
        path.reverse() # 역추적이므로 다시 뒤집어야 정방향이 됨
        return path

# ─────────────────────────────────────────────────────────────
# 3. Phase 2: Grouping (유사도 측정 및 그룹화)
# ─────────────────────────────────────────────────────────────

def pattern_similarity(arrow1, arrow2):
    """
    [알고리즘: Levenshtein Distance]
    두 배관의 방향 패턴 문자열 간의 편집 거리를 사용하여 0~1 사이의 유사도를 반환합니다.
    """
    if not arrow1 and not arrow2: return 1.0
    if not arrow1 or not arrow2: return 0.0
    a, b = arrow1.split("-"), arrow2.split("-")
    m, n = len(a), len(b)
    dp = list(range(n + 1))
    for i in range(1, m + 1):
        prev = dp[:]
        dp[0] = i
        for j in range(1, n + 1):
            if a[i - 1] == b[j - 1]: dp[j] = prev[j - 1]
            else: dp[j] = 1 + min(prev[j], dp[j - 1], prev[j - 1])
    return 1.0 - dp[n] / max(m, n)

def _parse_vectors(v_str):
    """(x,y,z), (x,y,z) 형태의 요약 문자열을 수치 리스트로 파싱합니다."""
    if not v_str: return []
    import re
    # 괄호 안의 숫자들을 찾아 리스트화
    matches = re.findall(r"\(([^)]+)\)", v_str)
    res = []
    for m in matches:
        res.append([float(x.strip()) for x in m.split(",")])
    return res

def compute_composite_similarity(r1, r2):
    """
    [고도화된 복합 유사도 분석 알고리즘]
    1. Arrow Similarity (30%): 배관 방향 코드의 형태적 유사성 (Levenshtein)
    2. Vector Alignment (30%): 세그먼트별 이동 벡터의 방향 일치도 (Cosine Correlation)
    3. Range Similarity (20%): x, y, z축 전체 물리적 규모(기하학적 스케일)의 일치도
    4. Length Similarity (20%): 배관 전체 경로 길이의 일치도
    """
    # 1. 방향 유사도 (Arrow)
    s_arrow = pattern_similarity(r1["path_arrow"], r2["path_arrow"])

    # 2. 벡터 상관관계 유사도 (Vector Correlation) - 신규
    vec1 = _parse_vectors(r1["path_step_vector"])
    vec2 = _parse_vectors(r2["path_step_vector"])
    s_vector = 0.0
    min_len = min(len(vec1), len(vec2))
    if min_len > 0:
        dot_sums = 0.0
        aligned_count = 0
        for i in range(min_len):
            v1, v2 = vec1[i], vec2[i]
            mag1 = math.sqrt(sum(x*x for x in v1))
            mag2 = math.sqrt(sum(x*x for x in v2))
            if mag1 > 0 and mag2 > 0:
                dot = sum(v1[j]*v2[j] for j in range(3))
                # Cosine Similarity: 완벽하게 같은 방향이면 1.0
                dot_sums += (dot / (mag1 * mag2))
                aligned_count += 1
            elif mag1 == 0 and mag2 == 0:
                dot_sums += 1.0 # 둘 다 정지점인 경우
                aligned_count += 1
        s_vector = dot_sums / aligned_count if aligned_count > 0 else 0.0
    else: 
        s_vector = 1.0 if not vec1 and not vec2 else 0.0
    
    # 3. 물리적 규모 유사도 (Range)
    rng1, rng2 = r1["path_range"], r2["path_range"]
    diffs = []
    for k in ["x", "y", "z"]:
        v1, v2 = rng1[k], rng2[k]
        mv = max(v1, v2, 1.0)
        diffs.append(1.0 - (abs(v1 - v2) / mv))
    s_range = sum(diffs) / 3.0
    
    # 4. 전체 길이 유사도 (Length)
    l1 = sum(float(x) for x in r1["path_step_length"].split(", ")) if r1["path_step_length"] else 0.0
    l2 = sum(float(x) for x in r2["path_step_length"].split(", ")) if r2["path_step_length"] else 0.0
    if l1 == 0 and l2 == 0: s_length = 1.0
    elif l1 == 0 or l2 == 0: s_length = 0.0
    else: s_length = 1.0 - (abs(l1 - l2) / max(l1, l2, 1.0))
    
    # 가중 합산 (최종 정밀 유사도)
    return (s_arrow * 0.3) + (s_vector * 0.3) + (s_range * 0.2) + (s_length * 0.2)

class GroupAnalyzer:
    """
    추출된 배관 경로 목록을 복합 유사도와 공간 정보를 기반으로 클러스터링(그룹화) 합니다.
    """
    def __init__(self, records): self.records = records

    def find_groups(self):
        """전체 경로 중에서 복합 유사도가 높은 것들을 찾아 후보 리스트를 반환합니다."""
        # 1. 장비 정보 + 유틸리티별로 1차 버킷 생성
        buckets = defaultdict(list)
        for r in self.records:
            # 부가 정보도 버킷 키에 포함하여 데이터 일관성 유지
            key = (r["equipment_name"], r["equipment_id"], r["equipment_process"], r["equipment_maker"], r["utility"])
            buckets[key].append(r)
        
        candidates = []
        for (eq_name, eq_id, eq_proc, eq_maker, util), group_recs in buckets.items():
            if len(group_recs) < MIN_GROUP_SIZE: continue
            n = len(group_recs)
            sim = [[0.0]*n for _ in range(n)]
            parent = list(range(n)) # Union-Find용 배열
            
            def _find(x):
                while parent[x] != x: parent[x], x = parent[parent[x]], parent[x]
                return x
            def _union(x, y): parent[_find(x)] = _find(y)

            # 2. 모든 경로 쌍에 대해 복합 유사도 계산
            for i in range(n):
                sim[i][i] = 1.0
                for j in range(i+1, n):
                    s = compute_composite_similarity(group_recs[i], group_recs[j])
                    sim[i][j] = sim[j][i] = s
                    # 3. 복합 유사도가 임계값 이상이면 같은 클러스터로 Union
                    if s >= PATTERN_SIMILARITY_MIN: _union(i, j)

            # 4. 클러스터별로 경로 묶기
            clusters = defaultdict(list)
            for i, rec in enumerate(group_recs): clusters[_find(i)].append((i, rec))

            for cl in clusters.values():
                if len(cl) < MIN_GROUP_SIZE: continue
                indices, recs = [idx for idx, _ in cl], [r for _, r in cl]
                
                # 5. 공간 필터 적용: 시작 POC 간의 평면 거리 확인
                p_pos = [r["start_pos"] for r in recs]
                max_xy = max(_dist_xy(a, b) for a, b in itertools.combinations(p_pos, 2)) if len(p_pos) >= 2 else 0.0
                if max_xy > START_POC_XY_MAX: continue

                # 6. 공통 수평 레이어(Z-Level) 확인
                c_z = self._find_common_z_levels(recs)
                pairs = list(itertools.combinations(range(len(indices)), 2))
                avg_sim = sum(sim[indices[i]][indices[j]] for i, j in pairs)/len(pairs) if pairs else 1.0
                
                # 7. 평균 간격(Spacing) 계산 - 시작 POC 기준 보완
                spacing = 0.0
                if len(recs) > 1:
                    # POC들 간의 최소 거리를 기반으로 간격 추정
                    p_dists = [_dist_xy(recs[i]["start_pos"], recs[j]["start_pos"]) for i, j in itertools.combinations(range(len(recs)), 2)]
                    spacing = sum(p_dists) / len(p_dists) if p_dists else 0.0

                candidates.append({
                    "equipment_name": eq_name, "equipment_id": eq_id, 
                    "equipment_process": eq_proc, "equipment_maker": eq_maker,
                    "utility": util, "size": recs[0].get("size", "-"),
                    "path_count": len(recs), "paths": recs, "avg_similarity": round(avg_sim, 4),
                    "max_start_xy_dist": round(max_xy, 1), 
                    "spacing": round(spacing, 1),
                    "common_z_levels": c_z
                })
        return candidates

    def _find_common_z_levels(self, recs):
        """경로들의 수평(H) 구간들을 분석하여 동일한 높이 레벨을 공유하는지 확인합니다."""
        all_segs = []
        for ridx, r in enumerate(recs):
            for s in r["h_segments"]: all_segs.append((ridx, s["mean_z"], s["mid_xy"]))
        if not all_segs: return []
        all_segs.sort(key=lambda x: x[1])
        used, result = [False]*len(all_segs), []
        for i, (ridx_i, z_i, xy_i) in enumerate(all_segs):
            if used[i]: continue
            cl_p, cl_xy, cl_z = {ridx_i}, [xy_i], [z_i]
            for j in range(i+1, len(all_segs)):
                if used[j]: continue
                if abs(all_segs[j][1] - z_i) <= TOL_Z_LEVEL:
                    cl_p.add(all_segs[j][0]); cl_xy.append(all_segs[j][2]); cl_z.append(all_segs[j][1]); used[j] = True
            used[i] = True
            if len(cl_p) >= MIN_GROUP_SIZE:
                spread = _xy_spread([(x, y, 0) for x, y in cl_xy])
                result.append({"mean_z": round(sum(cl_z)/len(cl_z), 1), "path_count": len(cl_p), "xy_spread": round(spread, 1)})
        return sorted(result, key=lambda x: x["mean_z"])

def detect_zones(cand):
    """
    한 그룹의 배관들이 함께 가는 구간(TRUNK)과 갈라지는 구간(FAN-IN/OUT)을 탐지합니다.
    배관 트렁크의 높이(Z)를 기준으로 구간 노드들을 분류합니다.
    """
    common_z = cand["common_z_levels"]
    if not common_z: return {"trunk": None, "fan_in": None, "fan_out": None, "is_trunk_estimated": False}
    
    # 1. 분산도가 가장 낮은 공통 Z 레벨을 TRUNK 레벨로 선정
    t_cands = [z for z in common_z if z["xy_spread"] <= TRUNK_MAX_XY_SPREAD]
    is_est = False
    if t_cands: t_info = max(t_cands, key=lambda x: (x["path_count"], -x["xy_spread"]))
    else: t_info = min(common_z, key=lambda x: x["xy_spread"]); is_est = True
    
    tz, z_band = t_info["mean_z"], TOL_Z_LEVEL * TRUNK_Z_BAND_FACTOR
    f_in, trk, f_out = [], [], []
    for r in cand["paths"]:
        for pos in r["node_positions"]:
            z = pos[2]
            # 2. 높이 차이에 따른 노드 분류
            if abs(z - tz) <= z_band: trk.append(pos) # TRUNK 구간
            elif z < tz - z_band: f_in.append(pos)    # 장비에서 TRUNK 전까지 (FAN-IN)
            else: f_out.append(pos)                   # TRUNK 이후 종단까지 (FAN-OUT)
    
    t_box = _bbox(trk)
    if t_box: t_box.update({"mean_z": tz, "path_count": t_info["path_count"]})
    return {"trunk": t_box, "fan_in": _bbox(f_in), "fan_out": _bbox(f_out), "is_trunk_estimated": is_est}

# ─────────────────────────────────────────────────────────────
# 4. Phase 3: Orchestration (메인 제어 로직)
# ─────────────────────────────────────────────────────────────

def main():
    """
    프로그램의 진입점(Entry Point)입니다.
    Phase 1(탐색)과 Phase 2(분석)을 순차적으로 진두지휘합니다.
    """
    parser = argparse.ArgumentParser(description="배관 라우팅 탐색 및 그룹 분석 통합 툴")
    parser.add_argument("--input", default=DEFAULT_INPUT_DIR, help="원본 데이터 디렉토리 (data-v10)")
    parser.add_argument("--routing_out", default=DEFAULT_ROUTING_DIR, help="라우팅 결과 저장 경로")
    parser.add_argument("--group_out", default=DEFAULT_GROUP_DIR, help="그룹 분석 결과 저장 경로")
    args = parser.parse_args()

    os.makedirs(args.routing_out, exist_ok=True)
    os.makedirs(args.group_out, exist_ok=True)

    # --- Phase 1: 개별 경로 탐색 (Routing) ---
    json_files = glob.glob(os.path.join(args.input, "**", "*.json"), recursive=True)
    print(f"[Phase 1] {len(json_files)}개 원본 파일 처리 중...")
    for fpath in json_files:
        graph = RoutingGraph()
        if not graph.load_from_json(fpath): continue
        results = graph.find_routing_paths()
        fbase = os.path.basename(fpath).replace(".json", "")
        for eq_id, eq_data in results.items():
            info = eq_data["info"]
            poc_results = eq_data["results"]
            safe_name = "".join(c if c.isalnum() else "_" for c in info["name"])
            
            out_data = {
                "equipment_id": info["id"],
                "equipment_name": info["name"],
                "equipment_process": info["process"],
                "equipment_maker": info["maker"],
                "poc_paths": []
            }
            for item in poc_results:
                poc = item["poc"]
                poc_id = _get_guid(poc)
                poc_entry = {"start_poc_id": poc_id, "start_poc_info": poc, "paths": []}
                for p_data in item["paths"]:
                    steps = p_data["path"]
                    start_pos = _get_position(poc) # 출발점 POC 좌표
                    prev_node_pos = start_pos
                    
                    lengths, vectors = [], []
                    for step in steps:
                        if step["kind"] == "NODE":
                            curr_pos = _get_position(step["data"])
                            
                            # 1. 이전 노드와의 x,y,z 방향 변위 계산
                            v = {
                                "x": round(curr_pos[0] - prev_node_pos[0], 1),
                                "y": round(curr_pos[1] - prev_node_pos[1], 1),
                                "z": round(curr_pos[2] - prev_node_pos[2], 1)
                            }
                            step["path_step_vector"] = v
                            vectors.append(f"({v['x']},{v['y']},{v['z']})")
                            
                            # 2. 이전 노드와의 직선 거리(길이) 계산
                            dist = math.sqrt(sum((curr_pos[i] - prev_node_pos[i])**2 for i in range(3)))
                            l = round(dist, 1)
                            step["path_step_length"] = l
                            lengths.append(str(l))
                            
                            # 현재 노드를 다음 계산을 위한 이전 노드로 저장
                            prev_node_pos = curr_pos

                    node_pos = [_get_position(s["data"]) for s in steps if s["kind"] == "NODE"]
                    arrow = _compute_path_arrow(node_pos)
                    
                    # 3. 시작 PoC부터 종단 노드까지의 x, y, z축 전체 범위(절대 변위) 계산
                    last_pos = node_pos[-1] if node_pos else start_pos
                    p_range = {
                        "x": round(abs(last_pos[0] - start_pos[0]), 1),
                        "y": round(abs(last_pos[1] - start_pos[1]), 1),
                        "z": round(abs(last_pos[2] - start_pos[2]), 1)
                    }

                    poc_entry["paths"].append({
                        "terminal_label": p_data["label"], 
                        "path_summary": _build_path_summary(steps),
                        "path_arrow": arrow, 
                        "path_range": p_range,
                        "path_step_length": ", ".join(lengths),
                        "path_step_vector": ", ".join(vectors),
                        "steps": steps, 
                        "end_node_id": _get_guid(p_data["end_node"]),
                        "end_node_type": p_data["end_node"].get("type", "-")
                    })
                out_data["poc_paths"].append(poc_entry)
            
            out_path = os.path.join(args.routing_out, f"{fbase}_{safe_name}_Path.json")
            with open(out_path, 'w', encoding='utf-8') as f:
                json.dump(out_data, f, ensure_ascii=False, indent=2)

    # --- Phase 2: 그룹 패턴 분석 (Grouping) ---
    print(f"[Phase 2] {args.routing_out}에서 데이터 수집 중...")
    all_recs, routing_files = [], glob.glob(os.path.join(args.routing_out, "*.json"))
    
    # 레벨 정보 관리 (Global or File-based)
    space_info_registry = []
    for fpath in json_files:
        try:
            with open(fpath, 'r', encoding='utf-8') as f:
                data = json.load(f)
                s_info = data.get("FileInfo", {}).get("SpaceInfo", [])
                if s_info: space_info_registry.extend(s_info)
        except: pass

    for rf in routing_files:
        try:
            with open(rf, encoding="utf-8") as f: data = json.load(f)
            eq_name = data.get("equipment_name", "-")
            eq_id = data.get("equipment_id", "-")
            eq_proc = data.get("equipment_process", "-")
            eq_maker = data.get("equipment_maker", "-")
            for ent in data.get("poc_paths", []):
                p_inf = ent.get("start_poc_info", {})
                p_util = p_inf.get("utility") or p_inf.get("Utility") or "-"
                p_size = p_inf.get("size") or p_inf.get("Size") or "-"
                p_pos = _get_position(p_inf)
                for p in ent.get("paths", []):
                    steps = p.get("steps", [])
                    n_pos = [_get_position(s["data"]) for s in steps if s["kind"] == "NODE"]
                    h_segs = [] # 수평 구간 추출
                    arr = p.get("path_arrow", "")
                    if arr and len(n_pos) >= 2:
                        # 방향 코드열(arrow)에서 'H'인 구간만 추출하여 통계 산출
                        codes = arr.split("-")
                        i = 0
                        while i < len(codes):
                            if codes[i] == "H":
                                j = i
                                while j < len(codes) and codes[j] == "H": j += 1
                                seg_pos = n_pos[i:min(j, len(n_pos)-1)+1]
                                if seg_pos: h_segs.append({"mean_z": sum(pt[2] for pt in seg_pos)/len(seg_pos), "mid_xy": (sum(pt[0] for pt in seg_pos)/len(seg_pos), sum(pt[1] for pt in seg_pos)/len(seg_pos))})
                                i = j
                            else: i += 1
                    
                    # 분석용 레코드 생성
                    all_recs.append({
                        "equipment_name": eq_name, "equipment_id": eq_id,
                        "equipment_process": eq_proc, "equipment_maker": eq_maker,
                        "poc_id": ent.get("start_poc_id"), "utility": p_util, "size": p_size,
                        "start_pos": p_pos, "path_arrow": arr, "node_positions": n_pos, "h_segments": h_segs,
                        "path_summary": p.get("path_summary"), "terminal_label": p.get("terminal_label"),
                        "end_node_id": p.get("end_node_id"), "end_node_type": p.get("end_node_type"),
                        "path_range": p.get("path_range"), "path_step_length": p.get("path_step_length"),
                        "path_step_vector": p.get("path_step_vector")
                    })
        except Exception: continue
    
    print(f"[Phase 2] 총 {len(all_recs)}개 경로 패턴 기반 군집화 시작...")
    analyzer = GroupAnalyzer(all_recs)
    cands = analyzer.find_groups()
    print(f"[Phase 2] 최종 그룹 후보 {len(cands)}개 탐지됨")

    # 결과 저장 (JSON & CSV)
    ts = datetime.now().strftime("%Y%m%d%H%M%S")
    json_p, csv_p = os.path.join(args.group_out, f"group_pipe_results_{ts}.json"), os.path.join(args.group_out, f"group_pipe_results_{ts}.csv")
    
    final_out = []
    for gid, c in enumerate(cands, 1):
        z = detect_zones(c) # 각 후보 그룹에 대해 TRUNK/FAN-IN/OUT 구간 산출
        
        # 주경로 타입 판단 (입상/수평)
        g_type = "수평"
        if c["paths"]:
            arr = c["paths"][0]["path_arrow"]
            if arr.count("R") > arr.count("H"): g_type = "입상"

        # Elevation 및 BOP 계산용 레벨 정보 탐색 (임시 저장소 사용)
        elevation = z["trunk"]["mean_z"] if z["trunk"] else (c["paths"][0]["start_pos"][2] if c["paths"] else 0.0)
        
        level_name = "N/A"
        bop_value = 0.0
        for space in space_info_registry:
            boundary = space.get("boundary", {})
            lvl_min_z = boundary.get("min", {}).get("z")
            lvl_max_z = boundary.get("max", {}).get("z")
            if lvl_min_z is not None and lvl_max_z is not None:
                if lvl_min_z <= elevation <= lvl_max_z:
                    level_name = space.get("levelName") or "N/A"
                    bop_value = round(elevation - lvl_min_z, 1)
                    break

        final_out.append({
            "group_id": gid, 
            "equipment_process": c["equipment_process"],
            "equipment_maker": c["equipment_maker"],
            "equipment_name": c["equipment_name"],
            "equipment_id": c["equipment_id"],
            "utility": c["utility"], "size": c["size"],
            "group_type": g_type,
            "path_count": c["path_count"], 
            "avg_similarity": c["avg_similarity"], 
            "max_start_xy_dist": c["max_start_xy_dist"],
            "spacing": c["spacing"],
            "elevation": round(elevation, 1),
            "level": level_name,
            "bop": bop_value,
            "zones": z, 
            "paths": [{
                "poc_id": r["poc_id"], 
                "terminal_label": r["terminal_label"],
                "path_summary": r["path_summary"],
                "path_arrow": r["path_arrow"],
                "path_range": r["path_range"],
                "path_step_length": r["path_step_length"],
                "path_step_vector": r["path_step_vector"]
            } for r in c["paths"]]
        })
    with open(json_p, "w", encoding="utf-8") as f: json.dump(final_out, f, indent=2, ensure_ascii=False)
    
    # 요약 정보 CSV 작성
    header = [
        "group_id", "equipment_process", "equipment_maker", "equipment_name", "equipment_id",
        "utility", "size", "입상/수평", "path_count", "spacing", "elevation", "level", "bop", "avg_similarity", 
        "trunk_z", "trunk_width", "trunk_depth", "trunk_thick", "trunk_xy_spread",
        "fan_in_z_min", "fan_in_z_max", "fan_out_z_min", "fan_out_z_max", "poc_ids"
    ]
    with open(csv_p, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        for r in final_out:
            tz, fn, fo = r["zones"]["trunk"], r["zones"]["fan_in"], r["zones"]["fan_out"]
            writer.writerow({
                "group_id": r["group_id"], 
                "equipment_process": r["equipment_process"], "equipment_maker": r["equipment_maker"],
                "equipment_name": r["equipment_name"], "equipment_id": r["equipment_id"],
                "utility": r["utility"], "size": r["size"],
                "입상/수평": r["group_type"],
                "path_count": r["path_count"], 
                "spacing": r["spacing"],
                "elevation": r["elevation"],
                "level": r["level"],
                "bop": r["bop"],
                "avg_similarity": r["avg_similarity"], 
                "trunk_z": tz["mean_z"] if tz else "-", 
                "trunk_width": tz["width_x"] if tz else "-",
                "trunk_depth": tz["depth_y"] if tz else "-",
                "trunk_thick": tz["thick_z"] if tz else "-",
                "trunk_xy_spread": tz["xy_spread"] if tz else "-",
                "fan_in_z_min": fn["z_min"] if fn else "-", "fan_in_z_max": fn["z_max"] if fn else "-",
                "fan_out_z_min": fo["z_min"] if fo else "-", "fan_out_z_max": fo["z_max"] if fo else "-",
                "poc_ids": "|".join(p["poc_id"] for p in r["paths"])
            })
    print(f"완료!")
    print(f"  - 상세결과(JSON): {json_p}")
    print(f"  - 요약결과(CSV): {csv_p}")

if __name__ == "__main__": main()
