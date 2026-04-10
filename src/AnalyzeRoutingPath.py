"""
AnalyzeRoutingPath.py
=====================
3D 배관 설계 데이터(JSON)에서 Equipment의 POC(Point of Connection)를 시작점으로
배관 네트워크를 BFS(너비 우선 탐색)로 순회하여 각 종단점(Duct, Nozzle, 부대장비 등)까지의
라우팅 경로를 추출하고 JSON 파일로 저장한다.

[전체 처리 흐름]
  1. data-v10/ 내 JSON 파일 로드
     └─ Nodes, Edges, Equipment 를 GUID 기반 딕셔너리로 인덱싱
  2. 각 Equipment의 pocList를 순회
     └─ POC 노드를 시작점으로 _trace_paths_from_poc() 호출
  3. BFS 탐색
     ├─ get_neighbors()  : 인접 노드 조회 (VIRTUAL 엣지 제외)
     ├─ is_branch_node() : 분기점 감지 → branch_info 갱신
     ├─ 종단 조건 충족 시 경로 저장
     └─ Cycle 방지 : Edge GUID 방문 집합 + 부모 포인터 역추적
  4. save_json_output()으로 결과 JSON 저장
     └─ RoutingResults/{파일명}_{장비명}_Path.json

[출력 JSON 구조]
  {
    "equipment_name": "...",
    "equipment_summary": {
      "total_poc_count": N,
      "poc_details": [
        {
          "poc_id": "...",
          "max_branch_depth": N,        <- 해당 POC에서 발생한 최대 분기 깊이
          "found_end_nodes": [
            { "id", "type", "label", "branch_depth" }
          ]
        }
      ]
    },
    "poc_paths": [
      {
        "start_poc_id": "...",
        "paths": [
          {
            "terminal_label": "...",    <- 종단 이유
            "branch_depth": N,          <- 이 경로가 통과한 분기점 수
            "branch_segments": [        <- 각 분기점의 메타데이터
              { "branch_node_guid", "branch_index", "branch_total" }
            ],
            "steps": [                  <- NODE/EDGE 교차 목록
              { "kind": "NODE"|"EDGE", "data": {...} }
            ]
          }
        ]
      }
    ]
  }
"""

import json
import math
import os
import glob
from collections import deque

# ─────────────────────────────────────────────────────────────
# 전역 상수
# ─────────────────────────────────────────────────────────────

# 단일 분기점(TEE/JUNCTION 등)에서 허용하는 최대 유효 연결 수.
# 이 값을 초과하면 데이터 이상 가능성이 있으므로 콘솔 경고를 출력한다.
MAX_BRANCH_COUNT = 8

# path_arrow 방향 분류 각도 오차 허용 범위 (단위: 도°).
# 수직(90°) 또는 수평(0°)으로부터 이 각도 이내이면 순수 수직/수평으로 판별한다.
#
#   R (Riser)    : 수직각도 θ > (90° - DIRECTION_ANGLE_TOLERANCE)
#   H (Header)   : 수직각도 θ < DIRECTION_ANGLE_TOLERANCE
#   D (Diagonal) : 그 외 (DIRECTION_ANGLE_TOLERANCE ≤ θ ≤ 90° - DIRECTION_ANGLE_TOLERANCE)
#
# 예) 기본값 5° → R: θ > 85°,  H: θ < 5°,  D: 5° ≤ θ ≤ 85°
DIRECTION_ANGLE_TOLERANCE = 5.0

# 노드 타입 → 2자 약어 매핑 테이블.
# found_end_nodes[].path_summary 필드에서 경로를 축약 표기할 때 사용한다.
# data-v10 실제 타입 기준으로 작성하였으며, 미등록 타입은 _build_path_summary()에서
# 타입명 앞 2자를 대문자로 자동 변환한다.
TYPE_ABBREV = {
    # ── 배관 피팅 ──────────────────────────────────────────
    "ELBOW"        : "EB",  # 엘보 (방향 전환 이음쇠)
    "TEE"          : "TE",  # T형 이음쇠 (3방향 분기)
    "REDUCER"      : "RD",  # 레듀서 (구경 변환)
    "UNION"        : "UN",  # 유니온 (분리 가능한 이음쇠)
    "FLANGE"       : "FL",  # 플랜지
    "ENDCAP"       : "EC",  # 엔드캡 (배관 말단 마감)
    "CONNECTOR"    : "CN",  # 커넥터
    "SOCKET"       : "SK",  # 소켓
    "BENDING"      : "BD",  # 벤딩 (현장 굽힘 배관)
    "CLAMP"        : "CL",  # 클램프
    "GLAND"        : "GL",  # 글랜드 (실링 부품)
    "GASKET"       : "GK",  # 가스켓
    "BELLOWS"      : "BL",  # 벨로즈 (신축 이음)
    # ── 기기 / 장비 ────────────────────────────────────────
    "VALVE"        : "VL",  # 밸브
    "FILTER"       : "FI",  # 필터
    "REGULATOR"    : "RG",  # 레귤레이터 (압력 조절기)
    "DAMPER"       : "DA",  # 댐퍼
    "DAMPER_DUCT"  : "DD",  # 덕트용 댐퍼
    # ── 연결점 / 종단 ───────────────────────────────────────
    "POC"          : "PO",  # Point of Connection (배관 접속점)
    "TAKEOFF"      : "TK",  # Takeoff (덕트/주배관 분기 접속)
    "LATERAL PIPE" : "LP",  # Lateral 배관 (측면 분기)
    "LATERAL"      : "LA",  # Lateral (일반 측면 분기)
    "DUCT"         : "DT",  # 덕트
    "EQUIPMENT"    : "EQ",  # 장비 노드
    "SUB_EQUIPMENT": "SE",  # 부대 장비
    # ── 분기 구조 (BRANCH_NODE_TYPES 대응) ──────────────────
    "BRANCH"       : "BR",  # 분기점
    "JUNCTION"     : "JN",  # 접합부
    "CROSS"        : "CR",  # 십자형 분기
    "WYE"          : "WY",  # Y형 분기
    # ── 기타 ────────────────────────────────────────────────
    "ETC"          : "ET",  # 기타 피팅
    "DIRECT_NODE"  : "DN",  # Direct 연결 (엣지 없는 노드 직결)
}

# path_summary 약어 범례 (JSON 출력 파일 최상단 "_legend" 키로 삽입).
# 약어(2자) → {"type": 원본 타입명, "desc": 한글 설명} 형태.
# TYPE_ABBREV의 역방향 매핑이며, 출력 파일을 열었을 때 즉시 약어를 확인할 수 있도록 한다.
PATH_SUMMARY_LEGEND = {
    abbrev: {"type": type_name, "desc": desc}
    for type_name, abbrev, desc in [
        # ── 배관 피팅 ───────────────────────────────────────────
        ("ELBOW"        , "EB", "엘보 - 방향 전환 이음쇠"),
        ("TEE"          , "TE", "T형 이음쇠 - 3방향 분기"),
        ("REDUCER"      , "RD", "레듀서 - 구경 변환"),
        ("UNION"        , "UN", "유니온 - 분리 가능한 이음쇠"),
        ("FLANGE"       , "FL", "플랜지"),
        ("ENDCAP"       , "EC", "엔드캡 - 배관 말단 마감"),
        ("CONNECTOR"    , "CN", "커넥터"),
        ("SOCKET"       , "SK", "소켓"),
        ("BENDING"      , "BD", "벤딩 - 현장 굽힘 배관"),
        ("CLAMP"        , "CL", "클램프"),
        ("GLAND"        , "GL", "글랜드 - 실링 부품"),
        ("GASKET"       , "GK", "가스켓"),
        ("BELLOWS"      , "BL", "벨로즈 - 신축 이음"),
        # ── 기기 / 장비 ─────────────────────────────────────────
        ("VALVE"        , "VL", "밸브"),
        ("FILTER"       , "FI", "필터"),
        ("REGULATOR"    , "RG", "레귤레이터 - 압력 조절기"),
        ("DAMPER"       , "DA", "댐퍼"),
        ("DAMPER_DUCT"  , "DD", "덕트용 댐퍼"),
        # ── 연결점 / 종단 ────────────────────────────────────────
        ("POC"          , "PO", "Point of Connection - 배관 접속점"),
        ("TAKEOFF"      , "TK", "Takeoff - 덕트/주배관 분기 접속"),
        ("LATERAL PIPE" , "LP", "Lateral 배관 - 측면 분기"),
        ("LATERAL"      , "LA", "Lateral - 일반 측면 분기"),
        ("DUCT"         , "DT", "덕트"),
        ("EQUIPMENT"    , "EQ", "장비 노드"),
        ("SUB_EQUIPMENT", "SE", "부대 장비"),
        # ── 분기 구조 ────────────────────────────────────────────
        ("BRANCH"       , "BR", "분기점"),
        ("JUNCTION"     , "JN", "접합부"),
        ("CROSS"        , "CR", "십자형 분기"),
        ("WYE"          , "WY", "Y형 분기"),
        # ── 기타 ─────────────────────────────────────────────────
        ("ETC"          , "ET", "기타 피팅"),
        ("DIRECT_NODE"  , "DN", "Direct 연결 - 엣지 없는 노드 직결"),
        ("??"           , "??", "타입 정보 없음"),
    ]
}


# ─────────────────────────────────────────────────────────────
# 모듈 수준 유틸리티
# ─────────────────────────────────────────────────────────────

def _get_position(node_data):
    """
    노드 데이터에서 (x, y, z) 좌표 튜플을 추출한다.

    v10 데이터는 'position' 키에 [x, y, z] 배열로 좌표를 저장한다.
    position 필드가 없을 경우 개별 x/y/z 키를 fallback으로 사용한다.

    Parameters
    ----------
    node_data : dict
        노드 객체

    Returns
    -------
    tuple[float, float, float]
        (x, y, z) 좌표. 좌표를 찾을 수 없으면 (0.0, 0.0, 0.0).
    """
    pos = node_data.get("position")
    if isinstance(pos, (list, tuple)) and len(pos) >= 3:
        return float(pos[0]), float(pos[1]), float(pos[2])
    # fallback: 개별 필드 (대소문자 모두 시도)
    x = node_data.get("x", node_data.get("X", 0.0))
    y = node_data.get("y", node_data.get("Y", 0.0))
    z = node_data.get("z", node_data.get("Z", 0.0))
    return float(x), float(y), float(z)


def _build_path_arrow(steps):
    """
    BFS 경로(steps)에서 연속된 NODE 쌍의 좌표를 분석하여
    각 구간의 방향을 R / H / D 로 분류하고 '-'로 연결한 문자열을 반환한다.

    [분류 기준]
    수직각도 θ = arcsin(|dz| / 전체이동거리) 로 계산한다.
    Z축이 높이(수직) 방향이며, X·Y축이 수평 평면을 구성한다.
    각도 경계는 DIRECTION_ANGLE_TOLERANCE(기본 5°) 를 기준으로 결정된다.

      R (Riser)    : θ > (90° - DIRECTION_ANGLE_TOLERANCE) — 수직 배관 (상승 또는 하강)
      H (Header)   : θ < DIRECTION_ANGLE_TOLERANCE         — 수평 배관
      D (Diagonal) : 그 외                                  — 경사 배관

    [예시]
      PO->BD->VL->TE->TK  경로라면  →  "R-H-H-R-R"

    Parameters
    ----------
    steps : list[dict]
        _trace_paths_from_poc()가 반환하는 경로 단계 목록.
        {"kind": "NODE"|"EDGE", "data": {...}} 형태.

    Returns
    -------
    str
        구간별 방향 코드를 '-'로 연결한 문자열.
        노드가 2개 미만이거나 모든 구간 거리가 0이면 빈 문자열.
    """
    # steps에서 NODE 항목의 좌표만 순서대로 추출
    positions = []
    for step in steps:
        if step["kind"] == "NODE":
            positions.append(_get_position(step["data"]))

    if len(positions) < 2:
        return ""

    directions = []
    for i in range(len(positions) - 1):
        x1, y1, z1 = positions[i]
        x2, y2, z2 = positions[i + 1]

        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1

        total = math.sqrt(dx * dx + dy * dy + dz * dz)
        if total < 1e-6:
            # 두 노드의 좌표가 동일한 경우(거리 0) → 방향 판별 불가, 건너뜀
            continue

        # 수평면으로부터의 수직각도 (0°=완전수평, 90°=완전수직)
        vertical_angle_deg = math.degrees(math.asin(abs(dz) / total))

        if vertical_angle_deg > (90.0 - DIRECTION_ANGLE_TOLERANCE):
            directions.append("R")   # Riser   : 수직 배관 (θ > 90°-허용오차)
        elif vertical_angle_deg < DIRECTION_ANGLE_TOLERANCE:
            directions.append("H")   # Header  : 수평 배관 (θ < 허용오차)
        else:
            directions.append("D")   # Diagonal: 경사 배관

    return "-".join(directions)


def _build_path_summary(steps):
    """
    BFS 경로(steps)에서 NODE 항목만 추출하여 타입 약어를 '->'로 연결한 요약 문자열을 반환한다.

    예시:
      PO->BD->TE->VL->BD->DT

    [약어 결정 규칙]
      1. TYPE_ABBREV 테이블에 등록된 타입 → 해당 2자 약어 사용
      2. 미등록 타입 → 타입명 앞 2자를 대문자로 변환 (예: "SPECIAL" → "SP")
      3. 타입이 빈 문자열이거나 없는 경우 → "??" 표시

    Parameters
    ----------
    steps : list[dict]
        _trace_paths_from_poc()가 반환하는 경로 단계 목록.
        {"kind": "NODE"|"EDGE", "data": {...}} 형태.

    Returns
    -------
    str
        노드 타입 약어를 '->'로 연결한 문자열. 노드가 없으면 빈 문자열.
    """
    abbrevs = []
    for step in steps:
        if step["kind"] != "NODE":
            continue  # EDGE 항목은 건너뜀

        node_type = (step["data"].get("type") or "").upper().strip()

        if not node_type:
            abbrevs.append("??")
        elif node_type in TYPE_ABBREV:
            abbrevs.append(TYPE_ABBREV[node_type])
        else:
            # 미등록 타입: 앞 2자 대문자 사용
            abbrevs.append(node_type[:2] if len(node_type) >= 2 else node_type.ljust(2, "?"))

    return "->".join(abbrevs)


def _get_guid(obj):
    """
    JSON 객체에서 식별자(GUID)를 일관된 우선순위로 추출한다.

    v10 데이터는 키 이름이 "guid" / "Guid" / "id" / "Id" 등으로 혼용되므로
    이 함수를 통해 단일 진입점으로 통일한다.

    우선순위: guid → Guid → id → Id

    Parameters
    ----------
    obj : dict
        GUID를 포함할 수 있는 JSON 객체

    Returns
    -------
    str | None
        추출된 식별자 문자열. 해당 키가 없으면 None.
    """
    return obj.get("guid") or obj.get("Guid") or obj.get("id") or obj.get("Id")


# ─────────────────────────────────────────────────────────────
# 핵심 클래스
# ─────────────────────────────────────────────────────────────

class RoutingGraph:
    """
    배관 네트워크 그래프를 표현하고 라우팅 경로를 탐색하는 클래스.

    내부적으로 노드와 엣지를 GUID 기반 딕셔너리로 관리하여
    O(1) 조회가 가능하도록 한다.

    Attributes
    ----------
    node_by_guid : dict[str, dict]
        노드 GUID → 노드 객체 매핑.
        노드는 배관 상의 연결점(조인트, PoC, Tee 등)을 나타낸다.
    edge_by_guid : dict[str, dict]
        엣지 GUID → 엣지 객체 매핑.
        엣지는 두 노드를 잇는 배관 세그먼트를 나타낸다.
    equipment_list : list[dict]
        JSON 내 Equipment 목록. 각 장비는 pocList를 통해 배관망과 연결된다.
    terminal_types : set[str]
        종단점으로 인식할 노드 타입 집합 (참고용, 실제 판별은 _trace_paths_from_poc 내부).
    BRANCH_NODE_TYPES : set[str]
        분기점으로 판별할 노드 타입 키워드 집합.
        데이터에 타입이 명시된 경우 연결 수 조회 없이 즉시 분기점으로 판단한다.
    """

    # 분기점 타입 키워드 집합 (클래스 수준 상수 — 인스턴스마다 중복 생성하지 않음)
    # TEE     : T형 배관 이음쇠 (3방향 분기)
    # BRANCH  : 일반 분기점
    # JUNCTION: 접합부
    # CROSS   : 십자형 이음쇠 (4방향 분기)
    # WYE     : Y형 이음쇠 (3방향 분기, 각도 존재)
    BRANCH_NODE_TYPES = {"TEE", "BRANCH", "JUNCTION", "CROSS", "WYE"}

    def __init__(self):
        self.node_by_guid    = {}   # GUID → 노드 객체
        self.edge_by_guid    = {}   # GUID → 엣지 객체
        self.equipment_list  = []   # Equipment 목록
        self.terminal_types  = {
            "DUCT", "LATERAL", "LATERAL PIPE",
            "EQUIPMENT", "SUB_EQUIPMENT",
            "ENDCAP", "CAP", "NOZZLE",
        }

    # ──────────────────────────────────────────────────────────
    # 데이터 로드
    # ──────────────────────────────────────────────────────────

    def load_from_json(self, file_path):
        """
        JSON 파일을 읽어 노드·엣지·장비 데이터를 GUID 기반 딕셔너리로 인덱싱한다.

        [처리 순서]
          1. 파일을 UTF-8로 열어 JSON 파싱
          2. JSON 루트가 리스트인 경우 처리 불가 → False 반환
          3. Nodes  → node_by_guid 딕셔너리 구축
          4. Edges  → edge_by_guid 딕셔너리 구축
          5. Equipment → equipment_list 설정 (단일 dict인 경우 리스트로 감쌈)

        Parameters
        ----------
        file_path : str
            읽어들일 JSON 파일의 경로

        Returns
        -------
        bool
            로드 성공 시 True, 처리 불가(루트가 list) 시 False
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # v10 데이터 중 일부는 JSON 루트가 배열 형태 → 처리 불가
        if isinstance(data, list):
            print(f"[Skip] {os.path.basename(file_path)} - JSON root is a list.")
            return False

        # 키 대소문자 혼용 대응 (Nodes/nodes, Edges/edges, Equipment/equipment)
        nodes  = data.get("Nodes",     data.get("nodes",     []))
        edges  = data.get("Edges",     data.get("edges",     []))
        equips = data.get("Equipment", data.get("equipment", []))

        # 노드 GUID 인덱싱 — 이후 O(1)로 노드 객체 조회 가능
        for n in nodes:
            g = _get_guid(n)
            if g:
                self.node_by_guid[g] = n

        # 엣지 GUID 인덱싱 — get_neighbors()에서 엣지 타입/연결 정보 조회에 사용
        for e in edges:
            g = _get_guid(e)
            if g:
                self.edge_by_guid[g] = e

        # Equipment 정규화: 단일 dict인 경우 리스트로 통일
        if isinstance(equips, dict):
            self.equipment_list = [equips]
        else:
            self.equipment_list = equips

        return True

    # ──────────────────────────────────────────────────────────
    # 그래프 탐색 유틸리티
    # ──────────────────────────────────────────────────────────

    def get_neighbors(self, node_guid):
        """
        주어진 노드 GUID에서 이동 가능한 (엣지, 상대방 노드 GUID) 쌍의 목록을 반환한다.

        v10 데이터의 connectionGuidList에는 두 가지 종류의 GUID가 섞여 있다:
          A. 엣지 GUID  → edge_by_guid 에서 조회 성공
          B. 노드 GUID  → node_by_guid 에서 조회 성공 (Direct 연결)

        [VIRTUAL 엣지 제외]
        VIRTUAL 타입 엣지는 실제 물리 배관이 아닌 논리적 연결을 나타내므로
        라우팅 경로 탐색에서 명시적으로 제외한다.

        [반환 형식]
          [(edge_obj, opposite_node_guid), ...]
          - edge_obj          : 실제 엣지 dict 또는 DIRECT_NODE 가상 엣지 dict
          - opposite_node_guid: 현재 노드에서 해당 엣지를 통해 도달하는 반대편 노드 GUID

        Parameters
        ----------
        node_guid : str
            기준 노드의 GUID

        Returns
        -------
        list[tuple[dict, str]]
            (엣지 객체, 상대방 노드 GUID) 쌍의 목록. 연결이 없으면 빈 리스트.
        """
        curr_node = self.node_by_guid.get(node_guid)
        if not curr_node:
            return []

        neighbors = []
        for guid in curr_node.get("connectionGuidList", []):

            # ── 케이스 A: connectionGuidList 항목이 엣지 GUID인 경우 ──
            edge = self.edge_by_guid.get(guid)
            if edge:
                # VIRTUAL 엣지는 물리 연결이 아니므로 경로 탐색에서 제외
                if edge.get("type", "").upper() == "VIRTUAL":
                    continue

                # 엣지의 connectionGuidList에서 반대편 노드 GUID 추출.
                # [node_a_guid, node_b_guid] 형태에서 현재 노드가 아닌 쪽을 선택한다.
                conn_list = edge.get("connectionGuidList", [])
                if len(conn_list) >= 2:
                    opp_guid = conn_list[0] if conn_list[1] == node_guid else conn_list[1]
                    neighbors.append((edge, opp_guid))
                continue

            # ── 케이스 B: connectionGuidList 항목이 노드 GUID인 경우 (Direct 연결) ──
            # 별도의 엣지 객체 없이 두 노드가 직접 연결된 경우.
            # 실제 배관 엣지가 없으므로 가상(pseudo) 엣지 dict를 생성하여 반환 형식을 통일한다.
            if guid in self.node_by_guid:
                pseudo_edge = {
                    "type": "DIRECT_NODE",
                    "id":   "direct",
                    "name": "direct_connection",
                }
                neighbors.append((pseudo_edge, guid))

        return neighbors

    def is_branch_node(self, node_guid):
        """
        주어진 노드가 분기점인지 판별한다.

        [판별 우선순위]
          1. 노드 타입(type)이 BRANCH_NODE_TYPES 집합에 포함
             (TEE / BRANCH / JUNCTION / CROSS / WYE)
             → get_neighbors() 호출 없이 즉시 True 반환 (빠른 경로)
          2. 노드 이름(name) 또는 ID에 BRANCH_NODE_TYPES 키워드 포함
             → 타입 필드가 없어도 이름 기반으로 보완 판별
          3. 유효 연결 수(VIRTUAL 제외) >= 3
             → 타입/이름에 명시되지 않았어도 구조적으로 분기인 노드를 감지

        Parameters
        ----------
        node_guid : str
            판별할 노드의 GUID

        Returns
        -------
        bool
            분기점이면 True, 아니면 False
        """
        node = self.node_by_guid.get(node_guid)
        if not node:
            return False

        node_type = node.get("type", "").upper()
        # 이름/ID 둘 다 확인 (데이터마다 어느 쪽에 정보가 있는지 다름)
        node_name = (node.get("name") or node.get("id") or "").upper()

        # 우선순위 1: 타입 기반 빠른 판별
        if node_type in self.BRANCH_NODE_TYPES:
            return True

        # 우선순위 2: 이름/ID 기반 판별
        if any(bt in node_name for bt in self.BRANCH_NODE_TYPES):
            return True

        # 우선순위 3: 구조 기반 판별 (타입·이름 정보 없는 경우의 안전망)
        return len(self.get_neighbors(node_guid)) >= 3

    # ──────────────────────────────────────────────────────────
    # 라우팅 경로 탐색 (공개 API)
    # ──────────────────────────────────────────────────────────

    def find_routing_paths(self):
        """
        모든 Equipment의 POC를 시작점으로 라우팅 경로를 탐색하여 반환한다.

        [처리 흐름]
          1. 전체 Equipment의 pocList에서 POC GUID 집합(all_equipment_pocs) 구축
             → BFS 도중 "타 Equipment의 POC에 도달"했는지 O(1)로 판별하기 위함
          2. 각 Equipment 순회
             a. pocList에서 POC 노드 객체 수집
             b. 각 POC에 대해 _trace_paths_from_poc() 호출
             c. 결과를 Equipment 이름을 키로 수집

        Returns
        -------
        dict[str, list[dict]]
            {
              "장비이름": [
                {
                  "poc"  : <POC 노드 객체>,
                  "paths": <_trace_paths_from_poc() 반환값>
                },
                ...
              ]
            }
        """
        results = {}

        # ── 1단계: 전체 Equipment POC GUID 집합 구축 ──
        # BFS 종단 조건 중 "타 Equipment의 POC 도달" 판별에 사용.
        # set()으로 구성하여 탐색 중 O(1) 조회 가능.
        all_equipment_pocs = set()
        for eq in self.equipment_list:
            for p in eq.get("pocList", []):
                p_guid = _get_guid(p)
                if p_guid:
                    all_equipment_pocs.add(p_guid)

        # ── 2단계: Equipment별 POC 탐색 ──
        for eq in self.equipment_list:
            if not isinstance(eq, dict):
                continue

            # Equipment 이름 추출 (키 대소문자 혼용 대응)
            eq_name = eq.get("name") or eq.get("Name") or "Unknown_Equipment"

            # pocList에서 실제 노드 객체 수집.
            # node_by_guid에 없는 GUID는 데이터 불일치이므로 제외한다.
            poc_list = []
            for p in eq.get("pocList", []):
                p_guid = _get_guid(p)
                if p_guid and p_guid in self.node_by_guid:
                    poc_list.append(self.node_by_guid[p_guid])

            if not poc_list:
                continue  # POC가 없는 Equipment는 건너뜀

            eq_poc_results = []

            for poc in poc_list:
                poc_guid = _get_guid(poc)
                # 단일 POC에서 출발하는 모든 분기 경로 탐색
                paths_found = self._trace_paths_from_poc(poc, poc_guid, all_equipment_pocs)

                eq_poc_results.append({
                    "poc"  : poc,
                    "paths": paths_found,
                })

            if eq_poc_results:
                results[eq_name] = eq_poc_results

        return results

    # ──────────────────────────────────────────────────────────
    # BFS 핵심 탐색 (내부 메서드)
    # ──────────────────────────────────────────────────────────

    def _trace_paths_from_poc(self, start_node, start_guid, all_start_guids):
        """
        단일 POC 노드에서 출발하여 도달 가능한 모든 종단점까지의 경로를 BFS로 탐색한다.

        ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        [메모리 효율 설계 — 부모 포인터(Parent Pointer) 방식]

        기존 방식 : 큐 항목마다 전체 경로 리스트를 복사 → O(경로길이) 메모리/항목
        개선 방식 : node_records 테이블에 부모 인덱스를 저장하고
                    종단 도달 시 역추적으로 경로를 재구성 → O(1) 메모리/항목

        node_records 구조 (append-only 리스트):
          index │ (curr_guid, node_data, parent_idx, edge_from_parent, branch_info)
          ──────┼──────────────────────────────────────────────────────────────────
            0   │ (start_guid, start_node, -1, None, {"depth":0, "segments":[]})
            1   │ (next_guid,  next_node,   0, edge, branch_info)
            2   │ (next_guid2, next_node2,  1, edge, branch_info)
            ...

          - curr_guid        : 현재 노드의 GUID
          - node_data        : 현재 노드의 전체 dict 객체
          - parent_idx       : node_records 내 부모 항목의 인덱스 (-1이면 시작 노드)
          - edge_from_parent : 부모 노드 → 현재 노드를 잇는 엣지 dict (시작 노드는 None)
          - branch_info      : 이 노드에 도달하기까지 통과한 분기 정보 (아래 참조)

        ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        [분기 정보 구조 — branch_info dict]

          {
            "depth"   : int,   # 시작 POC → 현재 노드까지 통과한 분기점 수
            "segments": [      # 각 분기점에서의 메타데이터 목록
              {
                "branch_node_guid": str,  # 분기점 노드의 GUID
                "branch_index"    : int,  # 이 경로가 선택한 분기 인덱스 (0-based)
                "branch_total"    : int,  # 해당 분기점의 전체 연결 수
              }
            ]
          }

          - 직선 구간 통과 시 : 동일 branch_info 객체를 그대로 전달 (불필요한 복사 없음)
          - 분기점 통과 시    : 새 dict를 생성하여 depth+1, segments에 항목 추가

        ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        [큐(queue) 항목 구조]

          (curr_guid, record_idx, visited_edges)

          - curr_guid     : 현재 처리 중인 노드의 GUID
          - record_idx    : node_records 내 현재 항목의 인덱스
          - visited_edges : 현재 경로에서 이미 방문한 엣지 GUID 집합 (frozenset)
                            frozenset을 사용하여 각 분기 경로가 독립적인 방문 집합을 유지

        ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        [종단 조건 — 6가지]

          1. "부대장비 PoC 도달"  : curr_guid가 all_start_guids에 포함
          2. "Duct PoC 도달"      : 타입이 DUCT/TAKEOFF 또는 이름/ID에 "DUCT" 포함
          3. "LateralPipe 도달"   : 타입이 LATERAL/LATERAL PIPE 또는 이름/ID에 "LATERAL" 포함
          4. "Nozzle PoC 도달"    : 이름 또는 ID에 "NOZZLE" 포함
          5. "종단 PoC 도달"      : 타입이 POC이고 이름이 "end" 또는 ID가 "END"로 시작
          6. "배관 끝단(막힘)"    : get_neighbors()가 빈 리스트 반환 (연결 없는 말단)

        ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        [사이클 방지 이중 체계]

          A. visited_edges (frozenset) : 같은 엣지를 동일 경로에서 두 번 통과하는 것을 방지
          B. _is_in_path()             : 부모 포인터 역추적으로 경로상 노드 중복 확인

        Parameters
        ----------
        start_node      : dict
            탐색 시작점인 POC 노드 객체
        start_guid      : str
            시작 POC 노드의 GUID
        all_start_guids : set[str]
            모든 Equipment POC GUID 집합 (종단 조건 1 판별용)

        Returns
        -------
        list[dict]
            발견된 경로 목록. 각 항목:
            {
              "end_node"        : dict,   # 종단 노드 객체
              "label"           : str,    # 종단 이유 레이블
              "reason"          : str,    # label과 동일 (하위 호환)
              "path"            : list,   # NODE/EDGE 교차 목록
              "branch_depth"    : int,    # 이 경로에서 통과한 분기점 수
              "branch_segments" : list,   # 각 분기점 메타데이터 목록
            }
        """
        # ── 초기화 ──
        # 시작 노드의 branch_info: 아직 분기점을 통과하지 않은 상태
        start_branch_info = {"depth": 0, "segments": []}

        # node_records: 부모 포인터 테이블 (append-only)
        # 튜플 구조: (curr_guid, node_data, parent_idx, edge_from_parent, branch_info)
        node_records = [(start_guid, start_node, -1, None, start_branch_info)]

        # BFS 큐: (curr_guid, record_idx, visited_edges)
        # frozenset을 사용하여 각 분기 경로가 독립적인 방문 집합을 가짐
        queue = deque([(start_guid, 0, frozenset())])

        # 최종 반환할 경로 목록
        paths_found = []

        # ── BFS 메인 루프 ──
        while queue:
            curr_guid, record_idx, visited_edges = queue.popleft()

            curr_node = self.node_by_guid.get(curr_guid)
            if not curr_node:
                continue  # 데이터 불일치: 노드 객체 없음 → 건너뜀

            # 현재 노드의 타입·이름·ID를 대문자로 정규화 (종단 조건 비교용)
            node_type = curr_node.get("type", "").upper()
            node_name = (curr_node.get("name") or curr_node.get("id") or "").upper()
            node_id   = curr_node.get("id", "").upper()

            # ── 종단 조건 판별 (시작 노드는 제외) ──
            if curr_guid != start_guid:
                stop  = False   # 종단 조건 충족 여부
                label = ""      # 종단 이유 레이블

                # 종단 조건 1: 타 Equipment의 POC에 도달
                if curr_guid in all_start_guids:
                    stop, label = True, "부대장비 PoC 도달"

                # 종단 조건 2-a: Duct 관련 노드 (덕트 연결점)
                elif node_type in ("DUCT", "TAKEOFF") or "DUCT" in node_id or "DUCT" in node_name:
                    stop, label = True, "Duct PoC 도달"

                # 종단 조건 2-b: Lateral 배관 관련 노드 (측면 분기 배관)
                elif node_type in ("LATERAL", "LATERAL PIPE") or "LATERAL" in node_id or "LATERAL" in node_name:
                    stop, label = True, "LateralPipe 도달"

                # 종단 조건 3: Nozzle (장비 접속 노즐)
                elif "NOZZLE" in node_name or "NOZZLE" in node_id:
                    stop, label = True, "Nozzle PoC 도달"

                # 종단 조건 4: 일반 종단 PoC (name="end" 또는 ID가 "END"로 시작)
                elif node_type == "POC" and (
                    curr_node.get("name") == "end"
                    or curr_node.get("id", "").startswith("END")
                ):
                    stop, label = True, "종단 PoC 도달"

                # 종단 조건 5: 타 장비(Equipment) 노드 자체에 도달
                elif node_type == "EQUIPMENT":
                    stop, label = True, "부대장비 도달"

                # 종단 조건 6: 배관 끝단 (연결 없는 말단)
                # stop이 이미 True이면 get_neighbors() 중복 호출을 피한다
                if not stop:
                    neighbors = self.get_neighbors(curr_guid)
                    if not neighbors:
                        stop, label = True, "배관 끝단(막힘)"
                else:
                    neighbors = []  # 종단 확정 → 이웃 탐색 불필요

                # ── 종단 확정 시: 경로 재구성 후 저장 ──
                if stop:
                    # node_records에서 현재 항목의 branch_info 추출
                    _, _, _, _, branch_info = node_records[record_idx]
                    paths_found.append({
                        "end_node"        : curr_node,
                        "label"           : label,
                        "reason"          : label,  # 하위 호환 필드
                        "path"            : self._reconstruct_path(node_records, record_idx),
                        "branch_depth"    : branch_info["depth"],
                        "branch_segments" : branch_info["segments"],
                    })
                    continue  # 이 경로 탐색 종료, 다음 큐 항목으로

            else:
                # 시작 노드: 종단 조건 없이 바로 이웃 조회
                neighbors = self.get_neighbors(curr_guid)

            # ── 분기점 감지 ──
            # 시작 노드는 분기점 판별에서 제외 (시작점은 항상 직선 출발로 취급)
            is_branch    = (curr_guid != start_guid) and self.is_branch_node(curr_guid)
            branch_count = len(neighbors)

            # 과도한 분기 경고: MAX_BRANCH_COUNT 초과 시 데이터 이상 가능성 알림
            if is_branch and branch_count > MAX_BRANCH_COUNT:
                print(
                    f"[Warning] 과도한 분기 감지: node={curr_guid}, "
                    f"분기수={branch_count} (MAX={MAX_BRANCH_COUNT})"
                )

            # 현재 노드의 branch_info 가져오기 (이웃 확장 시 전달할 기반 정보)
            _, _, _, _, curr_branch_info = node_records[record_idx]

            # ── 이웃 노드로 확장 ──
            # enumerate를 사용하여 분기 인덱스(branch_idx)를 함께 추적.
            # 분기점에서는 각 이웃이 독립적인 경로의 시작이 된다.
            for branch_idx, (edge, next_guid) in enumerate(neighbors):

                # 엣지 고유 식별자 계산.
                # 실제 엣지 GUID가 없는 Direct 연결의 경우 두 노드 GUID로 합성한다.
                edge_sig = _get_guid(edge) or f"direct_{curr_guid}_{next_guid}"

                # 사이클 방지 A: 같은 엣지를 현재 경로에서 이미 방문했으면 건너뜀
                if edge_sig in visited_edges:
                    continue

                # 사이클 방지 B: 목적지 노드가 현재 경로상에 이미 존재하면 건너뜀
                if self._is_in_path(node_records, record_idx, next_guid):
                    continue

                next_node = self.node_by_guid.get(next_guid)
                if not next_node:
                    continue  # 데이터 불일치: 목적지 노드 없음

                # ── branch_info 갱신 ──
                if is_branch:
                    # 분기점 통과: 새 branch_info 생성 (깊이 +1, 분기 메타데이터 추가)
                    new_branch_info = {
                        "depth"   : curr_branch_info["depth"] + 1,
                        "segments": curr_branch_info["segments"] + [{
                            "branch_node_guid": curr_guid,      # 분기점 노드 GUID
                            "branch_index"    : branch_idx,     # 선택된 분기 인덱스 (0-based)
                            "branch_total"    : branch_count,   # 이 분기점의 전체 연결 수
                        }],
                    }
                else:
                    # 직선 구간: 기존 branch_info를 그대로 전달 (객체 복사 없음)
                    new_branch_info = curr_branch_info

                # node_records에 새 항목 추가 (인덱스 = 현재 리스트 길이)
                new_record_idx = len(node_records)
                node_records.append((next_guid, next_node, record_idx, edge, new_branch_info))

                # 큐에 추가: visited_edges에 현재 엣지를 포함한 새 frozenset 전달
                queue.append((next_guid, new_record_idx, visited_edges | {edge_sig}))

        return paths_found

    # ──────────────────────────────────────────────────────────
    # 내부 헬퍼 메서드
    # ──────────────────────────────────────────────────────────

    def _is_in_path(self, node_records, current_idx, target_guid):
        """
        부모 포인터를 역추적하여 현재 경로에 target_guid 노드가 이미 포함되어 있는지 확인한다.

        [사이클 방지 역할]
        BFS에서 동일 노드를 두 번 방문하면 무한 루프(사이클)가 발생할 수 있다.
        visited_edges만으로는 다른 엣지를 통해 같은 노드에 재진입하는 경우를 막지 못하므로
        이 메서드로 경로상 노드 중복을 추가로 검사한다.

        [동작 원리]
        node_records[current_idx]부터 parent_idx를 따라 루트(parent_idx == -1)까지
        역추적하며 각 노드의 GUID를 target_guid와 비교한다.
        최악의 경우 O(경로 길이)이지만, 배관 경로는 일반적으로 수십~수백 노드 수준이다.

        Parameters
        ----------
        node_records  : list
            부모 포인터 테이블 (5-tuple 목록)
        current_idx   : int
            역추적을 시작할 현재 항목의 인덱스
        target_guid   : str
            경로 내 존재 여부를 확인할 노드 GUID

        Returns
        -------
        bool
            target_guid가 현재 경로에 포함되어 있으면 True
        """
        idx = current_idx
        while idx >= 0:
            # 5-tuple 언패킹: (guid, node_data, parent_idx, edge_data, branch_info)
            guid, _, parent_idx, _, _ = node_records[idx]
            if guid == target_guid:
                return True
            idx = parent_idx
        return False

    def _reconstruct_path(self, node_records, end_idx):
        """
        부모 포인터를 역추적하여 시작 POC → 종단 노드까지의 경로를 재구성한다.

        [동작 원리]
        node_records[end_idx]부터 parent_idx를 따라 루트까지 역추적하며
        NODE와 EDGE를 교차로 수집한 뒤, 역순으로 뒤집어 정방향 경로를 반환한다.

        [반환 형식]
          [
            {"kind": "NODE", "data": <시작 노드>},
            {"kind": "EDGE", "data": <엣지 1>},
            {"kind": "NODE", "data": <중간 노드>},
            {"kind": "EDGE", "data": <엣지 2>},
            ...
            {"kind": "NODE", "data": <종단 노드>},
          ]

        Parameters
        ----------
        node_records : list
            부모 포인터 테이블 (5-tuple 목록)
        end_idx      : int
            종단 노드의 node_records 인덱스

        Returns
        -------
        list[dict]
            {"kind": "NODE"|"EDGE", "data": {...}} 형태의 경로 단계 목록.
            시작 노드부터 종단 노드까지 정방향으로 정렬됨.
        """
        path = []
        idx  = end_idx

        # 역추적: 종단 → 시작 순으로 NODE/EDGE를 수집
        while idx >= 0:
            # 5-tuple 언패킹: (guid, node_data, parent_idx, edge_data, branch_info)
            _, node_data, parent_idx, edge_data, _ = node_records[idx]

            # 현재 노드 추가
            path.append({"kind": "NODE", "data": node_data})

            # 부모 → 현재를 연결한 엣지 추가 (시작 노드는 edge_data가 None)
            if edge_data is not None:
                path.append({"kind": "EDGE", "data": edge_data})

            idx = parent_idx

        # 역추적으로 수집했으므로 뒤집어 정방향으로 변환
        path.reverse()
        return path


# ─────────────────────────────────────────────────────────────
# 결과 저장
# ─────────────────────────────────────────────────────────────

def save_json_output(eq_name, poc_results, out_file):
    """
    단일 Equipment의 라우팅 탐색 결과를 JSON 파일로 저장한다.

    [출력 구조]
      equipment_name        : 장비 이름
      equipment_summary
        total_poc_count     : 장비의 총 POC 수
        poc_details[]       : POC별 요약
          poc_id            : POC 식별자
          poc_name          : POC 이름
          poc_type          : POC 타입
          poc_utility       : 유틸리티 코드
          poc_size          : 배관 구경
          total_paths_found : 이 POC에서 발견된 경로 수
          max_branch_depth  : 발견된 경로 중 최대 분기 깊이
          found_end_nodes[] : 종단 노드 요약 (id, type, label, branch_depth)
      poc_paths[]           : POC별 상세 경로
        start_poc_id        : 시작 POC 식별자
        start_poc_info      : 시작 POC 전체 객체
        paths[]             : 개별 경로
          terminal_label    : 종단 이유 레이블
          end_node_id       : 종단 노드 식별자
          end_node_type     : 종단 노드 타입
          end_node_size     : 종단 노드 배관 구경
          end_node_info     : 종단 노드 전체 객체
          branch_depth      : 이 경로에서 통과한 분기점 수
          branch_segments   : 각 분기점 메타데이터 목록
          steps             : NODE/EDGE 교차 경로 단계 목록

    Parameters
    ----------
    eq_name     : str
        Equipment 이름
    poc_results : list[dict]
        find_routing_paths() 반환값에서 해당 Equipment의 POC 결과 목록
    out_file    : str
        저장할 JSON 파일의 전체 경로
    """
    total_poc_count = len(poc_results)
    poc_summary    = []  # equipment_summary.poc_details 에 들어갈 요약 목록
    poc_paths_data = []  # poc_paths 에 들어갈 상세 경로 목록

    for item in poc_results:
        poc = item["poc"]

        # POC 기본 정보 추출 (키 대소문자 혼용 대응)
        poc_id      = _get_guid(poc) or "unknown"
        poc_name    = poc.get("name")    or "unknown"
        poc_utility = poc.get("utility") or poc.get("Utility") or "-"
        poc_size    = poc.get("size")    or poc.get("Size")    or "-"
        poc_type    = poc.get("type")    or poc.get("Type")    or "-"
        path_count  = len(item["paths"])

        # 이 POC에서 도달한 종단 노드들의 요약 정보 수집
        end_nodes_summary = []
        for p_data in item["paths"]:
            en = p_data["end_node"]
            path_steps = p_data.get("path", [])
            # 경로 steps에서 노드 타입 약어 요약 문자열 생성
            # 예: PO->BD->TE->VL->BD->DT
            path_summary = _build_path_summary(path_steps)
            # 경로 steps에서 구간별 방향 코드 문자열 생성
            # 예: R-H-H-D-R  (R=Riser 수직, H=Header 수평, D=Diagonal 경사)
            path_arrow = _build_path_arrow(path_steps)
            end_nodes_summary.append({
                "id"          : _get_guid(en) or "unknown",
                "type"        : en.get("type") or "unknown",
                "label"       : p_data["label"],
                "branch_depth": p_data.get("branch_depth", 0),
                "path_summary": path_summary,
                "path_arrow"  : path_arrow,
            })

        # 이 POC에서 발견된 경로들의 최대 분기 깊이 (경로가 없으면 0)
        max_branch_depth = max(
            (p.get("branch_depth", 0) for p in item["paths"]),
            default=0,
        )

        poc_summary.append({
            "poc_id"           : poc_id,
            "poc_name"         : poc_name,
            "poc_type"         : poc_type,
            "poc_utility"      : poc_utility,
            "poc_size"         : poc_size,
            "total_paths_found": path_count,
            "max_branch_depth" : max_branch_depth,
            "found_end_nodes"  : end_nodes_summary,
        })

        # 이 POC의 상세 경로 데이터 구성
        poc_entry = {
            "start_poc_id"  : poc_id,
            "start_poc_info": poc,
            "paths"         : [],
        }

        for p_data in item["paths"]:
            end_n = p_data["end_node"]
            path_entry = {
                "terminal_label" : p_data["label"],
                "end_node_id"    : _get_guid(end_n),
                "end_node_type"  : end_n.get("type") or "-",
                "end_node_size"  : end_n.get("size") or end_n.get("Size") or "-",
                "end_node_info"  : end_n,
                "branch_depth"   : p_data.get("branch_depth", 0),
                "branch_segments": p_data.get("branch_segments", []),
                "steps"          : p_data["path"],
            }
            poc_entry["paths"].append(path_entry)

        poc_paths_data.append(poc_entry)

    # 최종 출력 객체 조립
    # "_legend"를 첫 번째 키로 두어 JSON 파일 상단에서 바로 약어 의미를 확인할 수 있게 한다.
    output_data = {
        "_legend": {
            "_description"      : "path_summary 필드에 사용된 노드 타입 약어 설명 (약어: {type, desc})",
            "_format"           : "path_summary 예시: PO->BD->VL->TE->TK  (시작POC -> ... -> 종단노드)",
            "_path_arrow_desc"      : "path_arrow: 연속 NODE 쌍의 좌표 벡터로 구간 방향을 분류한 코드 (Z축=수직)",
            "_path_arrow_format"    : "path_arrow 예시: R-H-H-D-R  (구간 수 = 노드 수 - 1)",
            "_path_arrow_tolerance" : f"각도 허용 오차: {DIRECTION_ANGLE_TOLERANCE}°  (DIRECTION_ANGLE_TOLERANCE 변수로 조정 가능)",
            "_path_arrow_codes"     : {
                "R": f"Riser    — 수직 배관 (수직각도 > {90.0 - DIRECTION_ANGLE_TOLERANCE}°, 상승 또는 하강)",
                "H": f"Header   — 수평 배관 (수직각도 < {DIRECTION_ANGLE_TOLERANCE}°, X·Y 방향 이동)",
                "D": f"Diagonal — 경사 배관 ({DIRECTION_ANGLE_TOLERANCE}° ≤ 수직각도 ≤ {90.0 - DIRECTION_ANGLE_TOLERANCE}°)",
            },
            **PATH_SUMMARY_LEGEND,
        },
        "equipment_name"   : eq_name,
        "equipment_summary": {
            "total_poc_count": total_poc_count,
            "poc_details"    : poc_summary,
        },
        "poc_paths": poc_paths_data,
    }

    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)


# ─────────────────────────────────────────────────────────────
# 진입점
# ─────────────────────────────────────────────────────────────

def main():
    """
    data-v10/ 디렉터리 내 모든 JSON 파일을 순회하여 라우팅 경로를 분석하고
    결과를 RoutingResults/ 디렉터리에 저장한다.

    [처리 흐름]
      1. data-v10/**/*.json 파일 목록 수집
      2. 각 JSON 파일에 대해 RoutingGraph 인스턴스 생성 및 로드
      3. find_routing_paths()로 경로 탐색
      4. Equipment별로 save_json_output() 호출
         → RoutingResults/{파일명}_{장비명}_Path.json 저장
    """
    target_dir = "./data-v10"
    if not os.path.isdir(target_dir):
        print(f"Directory not found: {os.path.abspath(target_dir)}")
        return

    # 재귀적으로 모든 하위 디렉터리의 JSON 파일 수집
    json_files = glob.glob(os.path.join(target_dir, "**", "*.json"), recursive=True)
    if not json_files:
        print(f"No JSON files found in {target_dir}")
        return

    # 출력 디렉터리 생성 (이미 존재해도 오류 없음)
    out_dir = "./RoutingResults"
    os.makedirs(out_dir, exist_ok=True)

    for fpath in json_files:
        print(f"Processing: {fpath}")

        # JSON 파일마다 독립적인 그래프 인스턴스 생성 (파일 간 상태 격리)
        graph = RoutingGraph()
        if not graph.load_from_json(fpath):
            continue  # 로드 실패(루트가 list 등) → 건너뜀

        results = graph.find_routing_paths()

        # 파일 기본명 추출 (확장자 제거)
        fname_base = os.path.basename(fpath).replace(".json", "")

        for eq_name, poc_results in results.items():
            # 장비 이름의 특수문자를 '_'로 치환하여 파일명 안전하게 생성
            safe_name = "".join(c if c.isalnum() else "_" for c in eq_name)
            out_file  = os.path.join(out_dir, f"{fname_base}_{safe_name}_Path.json")

            save_json_output(eq_name, poc_results, out_file)
            print(f"  -> Saved routing paths to {out_file}")


if __name__ == "__main__":
    main()
