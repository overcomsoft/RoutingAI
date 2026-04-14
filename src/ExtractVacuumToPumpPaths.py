"""
ExtractVacuumToPumpPaths.py
===========================
Vaccum(진공) 배관경로 추출 스크립트

data/input/*.json 파일에서 utilityGroup="Vaccum"인 메인장비 PoC(Point of Connection)부터
Pump 장비 EQUIPMENT 노드까지의 배관경로를 BFS(너비우선탐색)로 추출한다.

출력 형식 (경로별):
  - 시작 PoC 좌표 (메인장비 Vaccum PoC)
  - 종단 Pump EQUIPMENT 좌표
  - 경로 리스트: [NODE, EDGE, NODE, EDGE, ...] (배관자재-배관 교대 순서)
  - 각 세그먼트별 변위 벡터 (dx, dy, dz) 및 거리(length)
  - 방향 패턴 문자열:
      R(Riser-수직), H(Header-수평), D(Diagonal-경사)
      예: R-D-R-D  (수직→경사→수직→경사)

================================================================================
[실행 명령어]
================================================================================

  # 기본 실행 (data/input → data/VacuumPumpPaths)
  python src/ExtractVacuumToPumpPaths.py

  # 입력/출력 디렉토리 지정
  python src/ExtractVacuumToPumpPaths.py -i ./data/input -o ./data/VacuumPumpPaths

  # 도움말
  python src/ExtractVacuumToPumpPaths.py --help

================================================================================
[전체 코드 흐름도]
================================================================================

  main()
    │
    ├─ 1. 명령줄 인자 파싱 (--input, --output)
    │
    ├─ 2. 입력 디렉토리에서 *.json 파일 목록 수집
    │
    └─ 3. 각 JSON 파일에 대해 process_json_file() 호출
         │
         ├─ 3-1. JSON 로드 → PipeGraph 그래프 구축
         │         ├─ nodes_by_guid : 노드 guid → 노드 dict 매핑
         │         ├─ edges_by_guid : 엣지 guid → 엣지 dict 매핑
         │         └─ node_to_edges : 노드 guid → [연결된 엣지 guid 목록] (역방향 인덱스)
         │
         ├─ 3-2. collect_pump_node_guids()
         │         └─ Nodes에서 type="EQUIPMENT" + 이름에 "pump" 포함인 노드 guid 수집
         │
         ├─ 3-3. collect_vaccum_start_pocs()
         │         └─ Equipment[].pocList에서 utilityGroup="Vaccum"인 PoC 수집
         │            (Pump 장비 자체의 PoC는 제외)
         │
         └─ 3-4. 각 Vaccum PoC에 대해 extract_paths_to_pump() BFS 탐색
                   │
                   ├─ BFS 큐: (현재 노드 guid, 지금까지의 경로)
                   ├─ visited_nodes: 전역 방문 집합 (중복 경로 방지)
                   ├─ reached_pumps: 이미 도달한 Pump EQUIPMENT guid 집합
                   │
                   ├─ 각 인접 노드에 대해:
                   │   ├─ Pump EQUIPMENT 도달 → 경로 기록 (대상별 최단 1개만)
                   │   ├─ 비-Pump EQUIPMENT 도달 → 종단 처리 (더 이상 진행 안함)
                   │   └─ 일반 노드 → 큐에 추가하여 탐색 계속
                   │
                   └─ 각 발견된 경로에 대해 enrich_path() 후처리
                       ├─ 노드 좌표로부터 변위 벡터(dx,dy,dz) 산출
                       ├─ 각 변위의 방향 분류 (R/H/D)
                       ├─ direction_raw: 모든 세그먼트 방향 (예: R-R-R-D-D-R)
                       └─ direction_pattern: 연속 동일 방향 압축 (예: R-D-R)

  출력:
    ├─ 파일별 JSON: {파일명}_vacuum_pump_paths.json
    └─ 전체 요약: summary.json
"""

import json
import glob
import os
import math
import argparse
from collections import defaultdict, deque


# ============================================================================
# 전역 설정 상수
# ============================================================================
DIRECTION_ANGLE_TOL = 5.0
"""방향 분류 허용 각도 (단위: 도)
- 수평면과의 각도가 85°~90° → R (Riser, 수직배관)
- 수평면과의 각도가 0°~5°  → H (Header, 수평배관)
- 그 외 5°~85°            → D (Diagonal, 경사배관)
"""

MAX_BFS_DEPTH = 300
"""BFS 최대 탐색 깊이.
경로의 스텝 수(NODE+EDGE)가 이 값×2를 초과하면 해당 분기를 중단한다.
배관 네트워크에서 300홉 이내에 Pump에 도달하지 못하면 연결이 없는 것으로 간주.
"""

MAX_PATHS_PER_POC = 100
"""하나의 시작 PoC에서 추출할 수 있는 최대 경로 수.
여러 Pump 장비가 존재할 경우 각 Pump별 최단 경로 1개씩 추출되므로,
이 값은 Pump 장비 수의 상한을 의미한다.
"""

PUMP_NAME_KEYWORDS = ["pump"]
"""Pump 장비를 식별하기 위한 키워드 목록.
Equipment.name 또는 Node.name을 소문자로 변환한 후 이 키워드 포함 여부를 확인.
예: "Comp_Pump_KAS_MU300P_9??" → "pump" 포함 → Pump 장비로 인식.
"""


# ============================================================================
# 방향 패턴 유틸리티 함수
# ============================================================================

def classify_direction(dx: float, dy: float, dz: float) -> str:
    """변위 벡터(dx, dy, dz)를 방향 코드로 분류한다.

    인자:
        dx: X축 변위 (mm)
        dy: Y축 변위 (mm)
        dz: Z축 변위 (mm, 수직 방향)

    반환:
        "R" - Riser (수직배관):   수평면 기준 각도 ≥ 85°
        "H" - Header (수평배관):  수평면 기준 각도 ≤ 5°
        "D" - Diagonal (경사배관): 5° < 각도 < 85°

    산출 방법:
        1. 수평 거리 = sqrt(dx² + dy²)
        2. 수직 거리 = |dz|
        3. 수평면 기준 앙각 = atan2(수직거리, 수평거리)
        4. 앙각에 따라 R/H/D 분류
    """
    # 벡터 길이가 거의 0이면 수평으로 간주
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-6:
        return "H"

    horiz = math.sqrt(dx * dx + dy * dy)  # XY 평면 수평 거리
    vert = abs(dz)                         # Z축 수직 거리

    # 수평면 기준 앙각 (0°=완전수평, 90°=완전수직)
    angle_from_horiz = math.degrees(math.atan2(vert, horiz)) if horiz > 1e-6 else 90.0

    if angle_from_horiz >= (90.0 - DIRECTION_ANGLE_TOL):   # ≥ 85° → 수직
        return "R"
    elif angle_from_horiz <= DIRECTION_ANGLE_TOL:           # ≤ 5°  → 수평
        return "H"
    else:                                                    # 5°~85° → 경사
        return "D"


def compute_displacement(pos_from: list, pos_to: list) -> dict:
    """두 3D 좌표 사이의 변위 벡터를 계산한다.

    인자:
        pos_from: 시작 좌표 [x, y, z] (mm)
        pos_to:   도착 좌표 [x, y, z] (mm)

    반환:
        {"dx": float, "dy": float, "dz": float}  (소수점 2자리 반올림)
    """
    return {
        "dx": round(pos_to[0] - pos_from[0], 2),
        "dy": round(pos_to[1] - pos_from[1], 2),
        "dz": round(pos_to[2] - pos_from[2], 2),
    }


# ============================================================================
# PipeGraph: 배관 네트워크 그래프 클래스
# ============================================================================

class PipeGraph:
    """Node-Edge 양방향 인덱스 그래프.

    JSON 데이터에서 Node.connectionGuidList → Edge guid 관계가
    단방향인 경우가 존재한다.
    (즉, Edge가 Node를 참조하지만 해당 Node의 connectionGuidList에는
     그 Edge guid가 누락된 경우가 있음)

    이를 해결하기 위해 Edge.connectionGuidList로부터 역방향 인덱스
    (node_to_edges)를 구축하여 양방향 탐색이 가능하게 한다.

    속성:
        nodes_by_guid (dict): 노드 guid → 노드 딕셔너리 매핑
            키: 노드의 고유 식별자 (UUID 문자열)
            값: {"guid", "id", "type", "name", "position", "connectionGuidList", ...}

        edges_by_guid (dict): 엣지 guid → 엣지 딕셔너리 매핑
            키: 엣지의 고유 식별자 (UUID 문자열)
            값: {"guid", "id", "type", "utility", "size", "material", "connectionGuidList", ...}

        node_to_edges (dict[str, list[str]]): 노드 guid → 연결된 엣지 guid 목록
            Edge.connectionGuidList를 순회하여 구축한 역방향 인덱스.
            한 노드가 여러 엣지에 연결될 수 있으므로 list로 저장.
    """

    def __init__(self, json_data: dict):
        """JSON 데이터로부터 그래프를 구축한다.

        인자:
            json_data: 전체 JSON 딕셔너리 (Nodes, Edges 키 포함)

        구축 과정:
            1단계: Nodes 배열 순회 → nodes_by_guid 인덱스 생성
            2단계: Edges 배열 순회 → edges_by_guid 인덱스 생성
                   + 각 Edge의 connectionGuidList에 포함된 노드 guid에 대해
                     node_to_edges 역방향 인덱스 등록
        """
        self.nodes_by_guid: dict = {}           # {노드guid: 노드dict}
        self.edges_by_guid: dict = {}           # {엣지guid: 엣지dict}
        self.node_to_edges: dict[str, list[str]] = defaultdict(list)
        # {노드guid: [해당 노드를 참조하는 엣지guid 목록]}

        # 1단계: 노드 인덱스 구축
        for n in json_data.get("Nodes", []):
            self.nodes_by_guid[n["guid"]] = n

        # 2단계: 엣지 인덱스 + 역방향(Edge→Node) 인덱스 구축
        for e in json_data.get("Edges", []):
            self.edges_by_guid[e["guid"]] = e
            # Edge.connectionGuidList의 각 노드 guid에 이 엣지를 등록
            for ng in e.get("connectionGuidList", []):
                self.node_to_edges[ng].append(e["guid"])

    def get_neighbors(self, node_guid: str) -> list[tuple[dict, dict]]:
        """주어진 노드에 연결된 (엣지, 반대편 노드) 쌍의 목록을 반환한다.

        인자:
            node_guid: 현재 노드의 guid

        반환:
            [(edge_dict, opposite_node_dict), ...] 형태의 리스트

        탐색 방법 (양방향 통합):
            1. node_to_edges 역방향 인덱스에서 이 노드를 참조하는 엣지 수집
            2. 노드 자체의 connectionGuidList에서 엣지 guid 추가 수집
            3. 수집된 모든 엣지에 대해 반대편 노드를 찾아 (엣지, 노드) 쌍 반환
            4. (엣지guid, 노드guid) 중복 방지를 위해 seen 집합 사용
        """
        results = []
        seen = set()  # (edge_guid, node_guid) 중복 방지용 집합

        # 1. 역방향 인덱스: 이 노드를 참조하는 엣지들
        edge_guids = set(self.node_to_edges.get(node_guid, []))

        # 2. 정방향: 노드 자체의 connectionGuidList에 있는 엣지 추가
        node = self.nodes_by_guid.get(node_guid)
        if node:
            for cg in node.get("connectionGuidList", []):
                if cg in self.edges_by_guid:
                    edge_guids.add(cg)

        # 3. 각 엣지에서 반대편 노드 탐색
        for eg in edge_guids:
            edge = self.edges_by_guid.get(eg)
            if not edge:
                continue
            for ng in edge.get("connectionGuidList", []):
                # 자기 자신이 아닌 유효한 노드만 선택
                if ng != node_guid and ng in self.nodes_by_guid:
                    key = (eg, ng)
                    if key not in seen:
                        seen.add(key)
                        results.append((edge, self.nodes_by_guid[ng]))
        return results


# ============================================================================
# Pump 장비 식별 함수
# ============================================================================

def is_pump_equipment(name: str) -> bool:
    """장비 이름에 Pump 키워드가 포함되는지 확인한다.

    인자:
        name: 장비 이름 문자열 (예: "Comp_Pump_KAS_MU300P_9??")

    반환:
        True  - PUMP_NAME_KEYWORDS 중 하나라도 포함되면 Pump 장비
        False - 키워드 미포함 (일반 장비)
    """
    lower = name.lower()
    return any(kw in lower for kw in PUMP_NAME_KEYWORDS)


def collect_pump_node_guids(json_data: dict) -> set[str]:
    """Pump EQUIPMENT 타입 노드의 guid 집합을 수집한다.

    JSON의 Nodes 배열에서 아래 2가지 조건을 모두 만족하는 노드를 찾는다:
      - type == "EQUIPMENT"
      - name에 "pump" 키워드 포함 (is_pump_equipment 판별)

    인자:
        json_data: 전체 JSON 딕셔너리

    반환:
        set[str] - Pump EQUIPMENT 노드의 guid 집합
                   예: {"d5e37df3-...", "a81d5b97-..."}
    """
    guids = set()
    for n in json_data.get("Nodes", []):
        if n["type"] == "EQUIPMENT" and is_pump_equipment(n.get("name", "")):
            guids.add(n["guid"])
    return guids


# ============================================================================
# 메인장비 Vaccum PoC 수집
# ============================================================================

def collect_vaccum_start_pocs(json_data: dict) -> list[dict]:
    """메인장비(Pump가 아닌 장비)의 utilityGroup='Vaccum' PoC 목록을 수집한다.

    Equipment 배열을 순회하면서:
      1. Pump 장비는 건너뜀 (is_pump_equipment으로 제외)
      2. 각 장비의 pocList에서 utilityGroup=="Vaccum"인 PoC를 필터링
      3. PoC의 name(예: "POC35440")을 Nodes에서 검색하여 노드 guid 매핑

    인자:
        json_data: 전체 JSON 딕셔너리

    반환:
        리스트. 각 원소는 다음 키를 포함하는 딕셔너리:
        {
            "equipment_name": str  - 소속 장비명 (예: "DANHJ14")
            "equipment_id":   str  - 소속 장비 ID (예: "EQ_00003_DANHJ14")
            "poc_name":       str  - PoC 이름=노드ID (예: "POC35440")
            "poc_guid":       str  - 대응하는 Node의 guid (BFS 시작점)
            "utility":        str  - 유틸리티 종류 (예: "Foreline", "PV")
            "poc_position":   list - PoC 3D 좌표 [x, y, z]
            "end_pocs":       list - 설계상 종단 PoC 정보 (참조용)
        }
    """
    # Nodes 배열에서 id → 노드 매핑 (PoC name과 Node id가 동일)
    nodes_by_id = {n["id"]: n for n in json_data.get("Nodes", [])}

    results = []
    for eq in json_data.get("Equipment", []):
        # Pump 장비의 PoC는 시작점이 아니므로 건너뜀
        if is_pump_equipment(eq.get("name", "")):
            continue

        for poc in eq.get("pocList", []):
            # Vaccum 유틸리티그룹만 필터링
            if poc.get("utilityGroup") != "Vaccum":
                continue

            # PoC name으로 Nodes에서 대응하는 노드를 찾아 guid를 얻음
            node = nodes_by_id.get(poc["name"])
            if node:
                results.append({
                    "equipment_name": eq["name"],
                    "equipment_id":   eq["id"],
                    "poc_name":       poc["name"],
                    "poc_guid":       node["guid"],
                    "utility":        poc.get("utility", ""),
                    "poc_position":   poc["pocPosition"],
                    "end_pocs":       poc.get("endPocs", []),
                })
    return results


# ============================================================================
# BFS 경로 추출 (핵심 알고리즘)
# ============================================================================

def extract_paths_to_pump(
    graph: PipeGraph,
    start_node_guid: str,
    pump_eq_guids: set[str],
) -> list[list[dict]]:
    """시작 노드에서 Pump EQUIPMENT 노드까지 BFS 최단경로를 추출한다.

    알고리즘:
        - 전역 visited_nodes 집합을 사용하는 표준 BFS (최단경로 보장)
        - 각 Pump EQUIPMENT별 최초 도달 경로만 기록 (reached_pumps로 중복 방지)
        - Pump의 PoC 노드는 중간 경유점으로 통과하여 EQUIPMENT까지 도달
        - 비-Pump EQUIPMENT 노드 도달 시 해당 분기 종단 처리 (다른 장비로 진입 방지)

    인자:
        graph:           PipeGraph 그래프 인스턴스
        start_node_guid: BFS 시작 노드의 guid (메인장비 Vaccum PoC)
        pump_eq_guids:   Pump EQUIPMENT 노드 guid 집합 (BFS 도달 목표)

    반환:
        list[list[dict]] - 발견된 경로들의 리스트
        각 경로는 [NODE_dict, EDGE_dict, NODE_dict, ...] 형태로
        시작 PoC에서 Pump EQUIPMENT까지의 교대 순서(배관자재-배관)를 나타냄.

    BFS 흐름:
        1. 시작 노드를 큐에 넣고 visited에 등록
        2. 큐에서 노드를 꺼내 인접 노드 탐색
           - Pump EQUIPMENT 도달 → 경로 기록, 해당 Pump는 reached_pumps에 등록
           - 이미 방문한 노드 → 건너뜀
           - 비-Pump EQUIPMENT → 종단 처리 (큐에 넣지 않음)
           - 일반 노드 → visited에 등록하고 큐에 추가
        3. 큐가 빌 때까지 반복
    """
    start_node = graph.nodes_by_guid[start_node_guid]
    initial_path = [_node_record(start_node)]  # 경로의 첫 원소: 시작 노드

    # BFS 큐: (현재 노드 guid, 시작부터 현재까지의 경로 리스트)
    queue = deque()
    queue.append((start_node_guid, initial_path))

    visited_nodes = {start_node_guid}  # 전역 방문 노드 집합 (중복 방문 방지)
    found_paths = []                   # 발견된 경로 결과 리스트
    reached_pumps = set()              # 이미 도달 완료한 Pump EQUIPMENT guid 집합

    while queue:
        cur_guid, path = queue.popleft()

        # 경로 길이 제한 (NODE+EDGE 교대이므로 MAX_BFS_DEPTH×2)
        if len(path) > MAX_BFS_DEPTH * 2:
            continue

        # 현재 노드의 모든 인접 (엣지, 반대편 노드) 쌍을 탐색
        for edge, next_node in graph.get_neighbors(cur_guid):
            ng = next_node["guid"]  # 다음 노드의 guid

            # ── Pump EQUIPMENT 도달 판정 ──
            # 해당 Pump에 최초 도달한 경우에만 경로를 기록한다.
            # (같은 Pump에 2번째 도달은 더 긴 경로이므로 무시)
            if ng in pump_eq_guids and ng not in reached_pumps:
                reached_pumps.add(ng)
                new_path = path + [_edge_record(edge), _node_record(next_node)]
                found_paths.append(new_path)
                # 최대 경로 수 도달 시 조기 종료
                if len(found_paths) >= MAX_PATHS_PER_POC:
                    return found_paths
                continue

            # ── 이미 방문한 노드 건너뜀 ──
            if ng in visited_nodes:
                continue

            visited_nodes.add(ng)
            new_path = path + [_edge_record(edge), _node_record(next_node)]

            # ── 비-Pump EQUIPMENT 종단 처리 ──
            # 다른 장비(Scrubber, VMB 등)의 EQUIPMENT 노드에 도달하면
            # 해당 분기는 더 이상 진행하지 않는다.
            if next_node["type"] == "EQUIPMENT":
                continue

            # ── 일반 노드 → 큐에 추가하여 탐색 계속 ──
            queue.append((ng, new_path))

    return found_paths


def _node_record(node: dict) -> dict:
    """그래프 노드를 경로 저장용 딕셔너리로 변환한다.

    인자:
        node: JSON Nodes 배열의 원소 딕셔너리

    반환:
        {
            "kind":     "NODE"          - 경로 원소 종류 (NODE 고정)
            "id":       "POC35440"      - 노드 ID
            "type":     "POC"           - 노드 타입 (POC, ELBOW, TEE, VALVE, BENDING,
                                          CLAMP, REDUCER, FLANGE, BELLOWS, EQUIPMENT 등)
            "name":     "start"         - 노드 이름 (배관자재명)
            "position": [x, y, z]       - 3D 좌표 (mm)
            "size":     "100A"          - 사이즈 (노드에 있는 경우)
            "material": "SUS316 EP"     - 재질 (노드에 있는 경우)
        }
    """
    return {
        "kind":     "NODE",
        "id":       node["id"],
        "type":     node["type"],
        "name":     node.get("name", ""),
        "position": node["position"],
        "size":     node.get("size", ""),
        "material": node.get("material", ""),
    }


def _edge_record(edge: dict) -> dict:
    """그래프 엣지를 경로 저장용 딕셔너리로 변환한다.

    인자:
        edge: JSON Edges 배열의 원소 딕셔너리

    반환:
        {
            "kind":     "EDGE"          - 경로 원소 종류 (EDGE 고정)
            "id":       "PIPE35487"     - 엣지 ID
            "type":     "PIPE"          - 엣지 타입 (PIPE, VIRTUAL, FLEXIBLE)
            "utility":  "Foreline"      - 유틸리티 (배관 용도)
            "size":     "100A"          - 배관 사이즈
            "material": "SUS316 EP"     - 배관 재질
        }
    """
    return {
        "kind":     "EDGE",
        "id":       edge["id"],
        "type":     edge["type"],
        "utility":  edge.get("utility", ""),
        "size":     edge.get("size", ""),
        "material": edge.get("material", ""),
    }


# ============================================================================
# 경로 후처리: 변위 벡터 및 방향 패턴 산출
# ============================================================================

def enrich_path(path: list[dict]) -> dict:
    """추출된 경로 리스트로부터 변위 벡터 시퀀스와 방향 패턴을 산출한다.

    처리 과정:
        1. 경로에서 NODE 원소만 추출하여 좌표 시퀀스 구성
        2. 인접한 노드 좌표 쌍에서 변위 벡터(dx, dy, dz) 계산
        3. 변위 길이가 0에 가까운 것은 제외 (동일 위치 노드)
        4. 각 변위의 방향을 R/H/D로 분류
        5. direction_raw: 모든 세그먼트의 방향을 나열 (예: R-R-R-D-D-R)
        6. direction_pattern: 연속 동일 방향을 압축 (예: R-D-R)

    인자:
        path: [NODE_dict, EDGE_dict, NODE_dict, ...] 형태의 경로

    반환:
        {
            "start_position":    [x, y, z]  - 경로 시작 좌표
            "end_position":      [x, y, z]  - 경로 끝 좌표
            "displacements":     [           - 변위 벡터 리스트
                {
                    "dx": float,             - X축 변위 (mm)
                    "dy": float,             - Y축 변위 (mm)
                    "dz": float,             - Z축 변위 (mm)
                    "length": float,         - 변위 거리 (mm)
                    "direction": "R"|"H"|"D" - 방향 분류
                }, ...
            ]
            "direction_pattern": "R-D-R"     - 압축 방향 패턴
            "direction_raw":     "R-R-D-R-R" - 전체 방향 시퀀스
        }
    """
    # 1. 경로에서 NODE의 position만 순서대로 추출
    node_positions = []
    for step in path:
        if step["kind"] == "NODE" and step.get("position"):
            pos = step["position"]
            if isinstance(pos, list) and len(pos) >= 3:
                node_positions.append(pos)

    # 2~4. 인접 노드 간 변위 벡터 계산 및 방향 분류
    displacements = []  # 변위 벡터 리스트
    directions = []     # 방향 코드 리스트 ("R", "H", "D")

    for i in range(1, len(node_positions)):
        disp = compute_displacement(node_positions[i - 1], node_positions[i])

        # 변위 길이가 거의 0인 경우 건너뜀 (동일 위치의 연속 노드)
        length = math.sqrt(disp["dx"]**2 + disp["dy"]**2 + disp["dz"]**2)
        if length < 1e-3:
            continue

        disp["length"] = round(length, 2)
        disp["direction"] = classify_direction(disp["dx"], disp["dy"], disp["dz"])
        displacements.append(disp)
        directions.append(disp["direction"])

    # 5~6. 방향 패턴 생성
    #   direction_raw:     모든 세그먼트 방향 나열 (예: R-R-R-D-D-R)
    #   direction_pattern: 연속 동일 방향 압축   (예: R-D-R)
    #     → R-R-R은 "쭉 수직으로 내려감"이므로 하나의 R로 압축
    compressed = []
    for d in directions:
        if not compressed or compressed[-1] != d:
            compressed.append(d)

    return {
        "start_position":    node_positions[0] if node_positions else [],
        "end_position":      node_positions[-1] if node_positions else [],
        "displacements":     displacements,
        "direction_pattern": "-".join(compressed),
        "direction_raw":     "-".join(directions),
    }


# ============================================================================
# 단일 JSON 파일 처리
# ============================================================================

def process_json_file(json_path: str) -> list[dict]:
    """하나의 JSON 파일에서 모든 Vaccum PoC → Pump EQUIPMENT 경로를 추출한다.

    처리 흐름:
        1. JSON 파일 로드
        2. PipeGraph 그래프 구축 (양방향 인덱스)
        3. Pump EQUIPMENT 노드 guid 수집
        4. 메인장비 Vaccum PoC 수집
        5. 각 PoC에서 BFS 실행 → Pump까지의 경로 추출
        6. 각 경로에 대해 변위 벡터 및 방향 패턴 산출
        7. 결과 딕셔너리 리스트 반환

    인자:
        json_path: 입력 JSON 파일의 절대 경로

    반환:
        list[dict] - 추출된 경로 정보 딕셔너리 리스트
        각 딕셔너리에 포함되는 키:
          - file:              입력 파일명
          - equipment_name:    시작 장비명
          - equipment_id:      시작 장비 ID
          - start_poc_name:    시작 PoC 이름 (예: "POC35440")
          - start_poc_utility: 유틸리티 종류 (예: "Foreline")
          - start_position:    시작 PoC 3D 좌표 [x, y, z]
          - end_position:      Pump EQUIPMENT 3D 좌표 [x, y, z]
          - end_node_id:       종단 노드 ID (예: "EQUIPMENT34994")
          - end_node_type:     종단 노드 타입 ("EQUIPMENT")
          - end_node_name:     종단 노드 이름 (예: "Comp_Pump_KAS_MU300P_9??")
          - direction_pattern: 압축 방향 패턴 (예: "R-D-R")
          - direction_raw:     전체 방향 시퀀스 (예: "R-R-R-D-R-R")
          - displacements:     변위 벡터 리스트
          - step_count:        경로 원소 수 (NODE + EDGE 총 개수)
          - path:              경로 원소 리스트 [NODE, EDGE, NODE, ...]
    """
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # 1. 그래프 구축
    graph = PipeGraph(data)

    # 2. Pump EQUIPMENT 노드 guid 수집
    pump_eq_guids = collect_pump_node_guids(data)
    if not pump_eq_guids:
        return []  # Pump 장비가 없는 파일은 처리 대상 아님

    # 3. 메인장비 Vaccum PoC 수집
    vaccum_pocs = collect_vaccum_start_pocs(data)
    results = []

    # 4. 각 Vaccum PoC에서 BFS 탐색
    for vp in vaccum_pocs:
        paths = extract_paths_to_pump(
            graph, vp["poc_guid"], pump_eq_guids
        )
        # 5. 각 경로에 대해 후처리 및 결과 조립
        for idx, path in enumerate(paths):
            enriched = enrich_path(path)
            end_step = path[-1] if path else {}

            results.append({
                "file":              os.path.basename(json_path),
                "equipment_name":    vp["equipment_name"],
                "equipment_id":      vp["equipment_id"],
                "start_poc_name":    vp["poc_name"],
                "start_poc_utility": vp["utility"],
                "start_position":    enriched["start_position"],
                "end_position":      enriched["end_position"],
                "end_node_id":       end_step.get("id", ""),
                "end_node_type":     end_step.get("type", ""),
                "end_node_name":     end_step.get("name", ""),
                "direction_pattern": enriched["direction_pattern"],
                "direction_raw":     enriched["direction_raw"],
                "displacements":     enriched["displacements"],
                "step_count":        len(path),
                "path":              path,
            })

    return results


# ============================================================================
# 메인 실행부
# ============================================================================

def main():
    """프로그램 진입점.

    실행 흐름:
        1. 명령줄 인자 파싱 (--input, --output)
        2. 입력 디렉토리에서 *.json 파일 목록 수집
        3. 각 JSON 파일에 대해 process_json_file() 호출
        4. 파일별 결과 JSON 저장: {파일명}_vacuum_pump_paths.json
        5. 전체 요약 JSON 저장: summary.json
        6. 콘솔에 처리 결과 요약 출력
    """
    # ── 1. 명령줄 인자 파싱 ──
    parser = argparse.ArgumentParser(
        description="Vaccum PoC → Pump PoC 배관 경로 추출"
    )
    parser.add_argument(
        "--input", "-i",
        default=os.path.join(os.path.dirname(__file__), "..", "data", "input"),
        help="입력 JSON 디렉토리 경로 (기본: data/input)",
    )
    parser.add_argument(
        "--output", "-o",
        default=os.path.join(os.path.dirname(__file__), "..", "data", "VacuumPumpPaths"),
        help="출력 디렉토리 경로 (기본: data/VacuumPumpPaths)",
    )
    args = parser.parse_args()

    input_dir  = os.path.abspath(args.input)   # 입력 디렉토리 절대경로
    output_dir = os.path.abspath(args.output)   # 출력 디렉토리 절대경로
    os.makedirs(output_dir, exist_ok=True)      # 출력 디렉토리 생성 (없으면)

    # ── 2. 입력 JSON 파일 수집 ──
    json_files = sorted(glob.glob(os.path.join(input_dir, "*.json")))
    if not json_files:
        print(f"[ERROR] JSON 파일을 찾을 수 없습니다: {input_dir}")
        return

    all_results = []    # 전체 파일의 결과를 모으는 리스트
    summary_rows = []   # 요약 정보 리스트 (summary.json용)

    # ── 3. 각 JSON 파일 처리 ──
    for json_path in json_files:
        fname = os.path.basename(json_path)
        print(f"\n{'='*60}")
        print(f"처리 중: {fname}")
        print(f"{'='*60}")

        results = process_json_file(json_path)

        if not results:
            print(f"  → Vaccum→Pump 경로 없음 (Pump 장비 미존재 또는 경로 미연결)")
            continue

        # ── 콘솔 출력: 각 경로 요약 ──
        print(f"  → 추출된 경로 수: {len(results)}")
        for r in results:
            print(f"    [{r['start_poc_name']}] {r['equipment_name']} "
                  f"→ {r['end_node_name']} | "
                  f"패턴: {r['direction_pattern']} | "
                  f"스텝: {r['step_count']}")

        # ── 4. 파일별 JSON 결과 저장 ──
        out_name = fname.replace("_total.json", "_vacuum_pump_paths.json")
        out_path = os.path.join(output_dir, out_name)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(results, f, ensure_ascii=False, indent=2)
        print(f"  → 저장: {out_path}")

        # ── 전체 결과 및 요약 누적 ──
        all_results.extend(results)
        for r in results:
            summary_rows.append({
                "file":              r["file"],
                "equipment":         r["equipment_name"],
                "start_poc":         r["start_poc_name"],
                "utility":           r["start_poc_utility"],
                "end_node":          r["end_node_name"],
                "direction_pattern": r["direction_pattern"],
                "step_count":        r["step_count"],
                "displacement_count": len(r["displacements"]),
            })

    # ── 5. 전체 요약 JSON 저장 ──
    summary_path = os.path.join(output_dir, "summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary_rows, f, ensure_ascii=False, indent=2)

    # ── 6. 최종 처리 결과 출력 ──
    print(f"\n{'='*60}")
    print(f"전체 완료: {len(all_results)}개 경로 추출 (파일 {len(json_files)}개)")
    print(f"요약 저장: {summary_path}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
