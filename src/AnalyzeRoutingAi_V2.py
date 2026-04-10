from __future__ import annotations

"""
AnalyzeRoutingAi_V2.py
==============================
3D 배관 설계 데이터(JSON)에서 개별 배관 경로를 추출하고,
추출된 경로들의 형상/방향/길이/공간 정보를 바탕으로 그룹 배관(Trunk/Bundle)을 분석하는 개선 버전입니다.

개선 포인트
-----------
1. 예외를 묵살하지 않고 파일명과 원인을 로깅합니다.
2. 경로 특징량을 문자열이 아닌 구조화된 숫자 리스트로도 함께 저장합니다.
3. 시작-끝 절대 변위가 아니라 전체 노드의 BBox Range를 사용합니다.
4. 같은 장비 내부 POC와 다른 장비 POC를 구분하여 종단 판정을 개선합니다.
5. 복합 유사도 계산 시 코사인 유사도를 0~1 범위로 정규화합니다.
6. Grouping 시 size까지 버킷 키에 포함하여 오검출을 줄입니다.
7. Phase 1 / Phase 2를 분리 실행할 수 있습니다.
8. 경로 수, 깊이, 큐 길이에 대한 안전 장치를 추가했습니다.

실행 예
-------
python AnalyzeRoutingAi_V2.py --phase all
python AnalyzeRoutingAi_V2.py --phase routing --input ./data-v10
python AnalyzeRoutingAi_V2.py --phase grouping --routing_out ./RoutingResults
"""

import argparse
import csv
import glob
import itertools
import json
import logging
import math
import os
import re
from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

# ─────────────────────────────────────────────────────────────
# 0. 설정 값
# ─────────────────────────────────────────────────────────────

TYPE_ABBREV = {
    "ELBOW": "EB", "TEE": "TE", "REDUCER": "RD", "UNION": "UN", "FLANGE": "FL",
    "ENDCAP": "EC", "CONNECTOR": "CN", "SOCKET": "SK", "BENDING": "BD", "CLAMP": "CL",
    "GLAND": "GL", "GASKET": "GK", "BELLOWS": "BL", "VALVE": "VL", "FILTER": "FI",
    "REGULATOR": "RG", "DAMPER": "DA", "DAMPER_DUCT": "DD", "POC": "PO", "TAKEOFF": "TK",
    "LATERAL PIPE": "LP", "LATERAL": "LA", "DUCT": "DT", "EQUIPMENT": "EQ", "SUB_EQUIPMENT": "SE",
    "BRANCH": "BR", "JUNCTION": "JN", "CROSS": "CR", "WYE": "WY", "ETC": "ET", "DIRECT_NODE": "DN"
}

BRANCH_NODE_TYPES = {"TEE", "BRANCH", "JUNCTION", "CROSS", "WYE"}


@dataclass
class AnalysisConfig:
    # 입력 / 출력
    input_dir: str = "./data-v10"
    routing_out: str = "./RoutingResults"
    group_out: str = "./GroupPipeResults"

    # BFS / 경로 탐색 보호 장치
    max_branch_count: int = 8
    direction_angle_tolerance: float = 5.0
    max_paths_per_poc: int = 5000
    max_queue_size: int = 100000
    max_depth: int = 512

    # Grouping 임계값
    pattern_similarity_min: float = 0.70
    start_poc_xy_max: float = 5000.0
    tol_z_level: float = 200.0
    trunk_max_xy_spread: float = 1500.0
    trunk_z_band_factor: float = 2.0
    min_group_size: int = 2
    require_common_z_levels: bool = False


LOGGER = logging.getLogger("AnalyzeRoutingAi")


# ─────────────────────────────────────────────────────────────
# 1. 공통 유틸리티
# ─────────────────────────────────────────────────────────────


def _normalize_text(value: Any) -> str:
    """문자열 비교 전 공백 제거와 대문자 정규화를 수행합니다."""
    if value is None:
        return ""
    return str(value).strip().upper()



def _get_guid(obj: Optional[Dict[str, Any]]) -> Optional[str]:
    """여러 형태의 키(guid/Guid/id/Id) 중 사용 가능한 식별자를 반환합니다."""
    if not obj:
        return None
    return obj.get("guid") or obj.get("Guid") or obj.get("id") or obj.get("Id")



def _safe_filename(value: str) -> str:
    """파일명으로 안전한 문자열을 생성합니다."""
    value = value or "Unknown"
    safe = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in value)
    return safe or "Unknown"



def _get_position(node_data: Dict[str, Any]) -> Tuple[float, float, float]:
    """노드 데이터에서 (x, y, z) 좌표를 반환합니다."""
    pos = node_data.get("position")
    if isinstance(pos, (list, tuple)) and len(pos) >= 3:
        return float(pos[0]), float(pos[1]), float(pos[2])

    x = node_data.get("x", node_data.get("X", 0.0))
    y = node_data.get("y", node_data.get("Y", 0.0))
    z = node_data.get("z", node_data.get("Z", 0.0))
    return float(x), float(y), float(z)



def _dist_xy(p1: Sequence[float], p2: Sequence[float]) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)



def _dist_xyz(p1: Sequence[float], p2: Sequence[float]) -> float:
    return math.sqrt(sum((p1[i] - p2[i]) ** 2 for i in range(3)))



def _xy_spread(positions: Sequence[Sequence[float]]) -> float:
    if len(positions) < 2:
        return 0.0
    return max(_dist_xy(a, b) for a, b in itertools.combinations(positions, 2))



def _bbox(positions: Sequence[Sequence[float]]) -> Optional[Dict[str, Any]]:
    if not positions:
        return None

    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    return {
        "x_min": round(min(xs), 1),
        "x_max": round(max(xs), 1),
        "y_min": round(min(ys), 1),
        "y_max": round(max(ys), 1),
        "z_min": round(min(zs), 1),
        "z_max": round(max(zs), 1),
        "x_range": round(max(xs) - min(xs), 1),
        "y_range": round(max(ys) - min(ys), 1),
        "z_range": round(max(zs) - min(zs), 1),
        "xy_spread": round(_xy_spread(positions), 1),
        "node_count": len(positions),
    }



def _build_path_summary(steps: Sequence[Dict[str, Any]]) -> str:
    """NODE step만 대상으로 타입 약어를 연결한 요약 문자열을 생성합니다."""
    abbrevs: List[str] = []
    for step in steps:
        if step.get("kind") != "NODE":
            continue
        node = step.get("data", {})
        node_type = _normalize_text(node.get("type"))
        if not node_type:
            abbrevs.append("??")
        elif node_type in TYPE_ABBREV:
            abbrevs.append(TYPE_ABBREV[node_type])
        else:
            abbrevs.append(node_type[:2])
    return "->".join(abbrevs)



def _compute_segment_code(p1: Sequence[float], p2: Sequence[float], tolerance_deg: float) -> Optional[str]:
    """두 점 사이의 이동을 R/H/D 코드로 변환합니다."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    total = math.sqrt(dx * dx + dy * dy + dz * dz)
    if total < 1e-9:
        return None

    v_angle = math.degrees(math.asin(abs(dz) / total))
    if v_angle > (90.0 - tolerance_deg):
        return "R"
    if v_angle < tolerance_deg:
        return "H"
    return "D"



def _compute_path_arrow(positions: Sequence[Sequence[float]], tolerance_deg: float) -> str:
    """경로 전체의 segment 코드열을 생성합니다."""
    if len(positions) < 2:
        return ""

    codes: List[str] = []
    for i in range(len(positions) - 1):
        code = _compute_segment_code(positions[i], positions[i + 1], tolerance_deg)
        if code:
            codes.append(code)
    return "-".join(codes)



def _extract_h_segments(
    positions: Sequence[Sequence[float]],
    arrow: str,
) -> List[Dict[str, Any]]:
    """path_arrow에서 연속된 H 구간을 찾아 mean_z와 mid_xy를 계산합니다."""
    if not arrow or len(positions) < 2:
        return []

    codes = arrow.split("-")
    segments: List[Dict[str, Any]] = []
    i = 0
    while i < len(codes):
        if codes[i] != "H":
            i += 1
            continue

        j = i
        while j < len(codes) and codes[j] == "H":
            j += 1

        # H 세그먼트는 positions[i] ~ positions[j] 구간에 대응합니다.
        seg_positions = positions[i:j + 1]
        if seg_positions:
            mean_z = sum(p[2] for p in seg_positions) / len(seg_positions)
            mid_x = sum(p[0] for p in seg_positions) / len(seg_positions)
            mid_y = sum(p[1] for p in seg_positions) / len(seg_positions)
            segments.append({
                "mean_z": round(mean_z, 3),
                "mid_xy": (round(mid_x, 3), round(mid_y, 3)),
                "node_count": len(seg_positions),
            })
        i = j
    return segments



def _serialize_vectors(vectors: Sequence[Dict[str, float]]) -> str:
    return ", ".join(f"({v['x']},{v['y']},{v['z']})" for v in vectors)



def _serialize_lengths(lengths: Sequence[float]) -> str:
    return ", ".join(str(round(v, 1)) for v in lengths)



def _parse_legacy_vectors(value: Any) -> List[Dict[str, float]]:
    """기존 문자열 포맷과 신규 리스트 포맷을 모두 수용합니다."""
    if value is None:
        return []

    if isinstance(value, list):
        parsed: List[Dict[str, float]] = []
        for item in value:
            if isinstance(item, dict):
                parsed.append({
                    "x": round(float(item.get("x", 0.0)), 1),
                    "y": round(float(item.get("y", 0.0)), 1),
                    "z": round(float(item.get("z", 0.0)), 1),
                })
            elif isinstance(item, (list, tuple)) and len(item) >= 3:
                parsed.append({
                    "x": round(float(item[0]), 1),
                    "y": round(float(item[1]), 1),
                    "z": round(float(item[2]), 1),
                })
        return parsed

    if isinstance(value, str):
        matches = re.findall(r"\(([^)]+)\)", value)
        parsed = []
        for match in matches:
            parts = [p.strip() for p in match.split(",")]
            if len(parts) >= 3:
                parsed.append({
                    "x": round(float(parts[0]), 1),
                    "y": round(float(parts[1]), 1),
                    "z": round(float(parts[2]), 1),
                })
        return parsed

    return []



def _parse_legacy_lengths(value: Any) -> List[float]:
    """기존 문자열 포맷과 신규 리스트 포맷을 모두 수용합니다."""
    if value is None:
        return []

    if isinstance(value, list):
        return [round(float(v), 1) for v in value]

    if isinstance(value, str):
        tokens = [t.strip() for t in value.split(",") if t.strip()]
        return [round(float(t), 1) for t in tokens]

    return []



def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))



def _ensure_list_of_positions(steps: Sequence[Dict[str, Any]]) -> List[Tuple[float, float, float]]:
    return [_get_position(step["data"]) for step in steps if step.get("kind") == "NODE"]



def _compute_path_features(
    steps: Sequence[Dict[str, Any]],
    config: AnalysisConfig,
) -> Dict[str, Any]:
    """
    경로에서 구조화된 분석 특징량을 계산합니다.

    개선 사항
    ---------
    1. 시작 NODE의 0벡터 / 0길이를 제외합니다.
    2. 시작-끝 절대 변위가 아니라 전체 경로 BBox Range를 사용합니다.
    3. 문자열 포맷과 구조화 포맷을 함께 저장합니다.
    """
    node_positions = _ensure_list_of_positions(steps)
    arrow = _compute_path_arrow(node_positions, config.direction_angle_tolerance)
    bbox = _bbox(node_positions)

    step_vectors: List[Dict[str, float]] = []
    step_lengths: List[float] = []
    for i in range(1, len(node_positions)):
        prev_pos = node_positions[i - 1]
        curr_pos = node_positions[i]
        vector = {
            "x": round(curr_pos[0] - prev_pos[0], 1),
            "y": round(curr_pos[1] - prev_pos[1], 1),
            "z": round(curr_pos[2] - prev_pos[2], 1),
        }
        step_vectors.append(vector)
        step_lengths.append(round(_dist_xyz(prev_pos, curr_pos), 1))

    if bbox:
        path_range = {
            "x": bbox["x_range"],
            "y": bbox["y_range"],
            "z": bbox["z_range"],
        }
    else:
        path_range = {"x": 0.0, "y": 0.0, "z": 0.0}

    return {
        "node_positions": node_positions,
        "path_arrow": arrow,
        "path_bbox": bbox,
        "path_range": path_range,
        "path_step_vectors": step_vectors,
        "path_step_lengths": step_lengths,
        "path_step_vector": _serialize_vectors(step_vectors),   # 하위 호환용
        "path_step_length": _serialize_lengths(step_lengths),   # 하위 호환용
        "path_total_length": round(sum(step_lengths), 1),
        "h_segments": _extract_h_segments(node_positions, arrow),
    }


# ─────────────────────────────────────────────────────────────
# 2. BFS 기반 경로 탐색기
# ─────────────────────────────────────────────────────────────


class RoutingGraph:
    """JSON 설계 데이터를 그래프로 로드하고 BFS로 모든 경로를 탐색합니다."""

    def __init__(self, config: AnalysisConfig):
        self.config = config
        self.node_by_guid: Dict[str, Dict[str, Any]] = {}
        self.edge_by_guid: Dict[str, Dict[str, Any]] = {}
        self.equipment_list: List[Dict[str, Any]] = []
        self.poc_owner_map: Dict[str, List[str]] = defaultdict(list)

    def load_from_json(self, file_path: str) -> bool:
        """설계 JSON을 읽어 노드/엣지/장비 인덱스를 생성합니다."""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            LOGGER.exception("JSON 로딩 실패: %s", file_path)
            LOGGER.debug("상세 예외: %s", exc)
            return False

        if isinstance(data, list):
            LOGGER.warning("루트가 list 형식인 비정상 파일이라 건너뜁니다: %s", file_path)
            return False

        self.node_by_guid.clear()
        self.edge_by_guid.clear()
        self.equipment_list.clear()
        self.poc_owner_map.clear()

        nodes = data.get("Nodes", data.get("nodes", []))
        edges = data.get("Edges", data.get("edges", []))
        equips = data.get("Equipment", data.get("equipment", []))
        self.equipment_list = [equips] if isinstance(equips, dict) else list(equips or [])

        for node in nodes:
            guid = _get_guid(node)
            if not guid:
                LOGGER.warning("GUID 없는 노드를 건너뜁니다.")
                continue
            self.node_by_guid[guid] = node

        for edge in edges:
            guid = _get_guid(edge)
            if not guid:
                LOGGER.warning("GUID 없는 엣지를 건너뜁니다.")
                continue
            self.edge_by_guid[guid] = edge

        for eq in self.equipment_list:
            eq_id = _get_guid(eq) or "UnknownEquipment"
            for poc in eq.get("pocList", []):
                poc_guid = _get_guid(poc)
                if poc_guid:
                    self.poc_owner_map[poc_guid].append(eq_id)

        return True

    def get_neighbors(self, node_guid: str) -> List[Tuple[Dict[str, Any], str]]:
        """
        현재 노드에서 이동 가능한 (edge, next_node_guid) 목록을 반환합니다.

        개선 사항
        ---------
        1. edge.connectionGuidList 안에 현재 node_guid가 실제로 포함되는지 검증합니다.
        2. 중복 이웃을 제거합니다.
        3. VIRTUAL edge를 명시적으로 제외합니다.
        """
        curr_node = self.node_by_guid.get(node_guid)
        if not curr_node:
            return []

        neighbors: List[Tuple[Dict[str, Any], str]] = []
        seen_pairs = set()

        for ref_guid in curr_node.get("connectionGuidList", []):
            edge = self.edge_by_guid.get(ref_guid)
            if edge:
                edge_type = _normalize_text(edge.get("type"))
                if edge_type == "VIRTUAL":
                    continue

                conn = edge.get("connectionGuidList", []) or []
                if node_guid not in conn:
                    LOGGER.debug(
                        "엣지 연결 목록에 현재 노드가 없습니다. edge=%s node=%s",
                        _get_guid(edge), node_guid,
                    )
                    continue

                for next_guid in conn:
                    if next_guid == node_guid:
                        continue
                    if next_guid not in self.node_by_guid:
                        continue
                    pair_key = (_get_guid(edge), next_guid)
                    if pair_key not in seen_pairs:
                        neighbors.append((edge, next_guid))
                        seen_pairs.add(pair_key)
                continue

            # edge GUID가 아니라 직접 node GUID가 connectionGuidList에 들어있는 경우
            if ref_guid in self.node_by_guid:
                pseudo_edge = {"type": "DIRECT_NODE", "id": f"direct_{node_guid}_{ref_guid}"}
                pair_key = (_get_guid(pseudo_edge) or pseudo_edge["id"], ref_guid)
                if pair_key not in seen_pairs:
                    neighbors.append((pseudo_edge, ref_guid))
                    seen_pairs.add(pair_key)

        return neighbors

    def is_branch_node(self, node_guid: str) -> bool:
        """분기 노드 여부를 판정합니다."""
        node = self.node_by_guid.get(node_guid)
        if not node:
            return False

        ntype = _normalize_text(node.get("type"))
        nname = _normalize_text(node.get("name") or node.get("id"))
        neighbors = self.get_neighbors(node_guid)

        if len(neighbors) > self.config.max_branch_count:
            LOGGER.warning(
                "과도한 분기 수 감지: node=%s neighbor_count=%d",
                node_guid,
                len(neighbors),
            )

        if ntype in BRANCH_NODE_TYPES:
            return True
        if any(branch_type in nname for branch_type in BRANCH_NODE_TYPES):
            return True
        return len(neighbors) >= 3

    def find_routing_paths(self) -> Dict[str, Dict[str, Any]]:
        """모든 Equipment의 POC를 시작점으로 하여 경로를 추출합니다."""
        results: Dict[str, Dict[str, Any]] = {}

        for eq in self.equipment_list:
            eq_id = _get_guid(eq) or "UnknownID"
            eq_info = {
                "id": eq_id,
                "name": eq.get("name") or eq.get("Name") or "Unknown",
                "process": eq.get("process") or eq.get("Process") or "-",
                "maker": eq.get("maker") or eq.get("Maker") or "-",
            }

            poc_nodes: List[Dict[str, Any]] = []
            for poc in eq.get("pocList", []):
                poc_guid = _get_guid(poc)
                if poc_guid and poc_guid in self.node_by_guid:
                    poc_nodes.append(self.node_by_guid[poc_guid])

            if not poc_nodes:
                continue

            eq_results = []
            for poc_node in poc_nodes:
                poc_guid = _get_guid(poc_node)
                if not poc_guid:
                    continue
                paths = self._trace_paths_from_poc(
                    start_node=poc_node,
                    start_guid=poc_guid,
                    current_equipment_id=eq_id,
                )
                eq_results.append({"poc": poc_node, "paths": paths})

            if eq_results:
                results[eq_id] = {"info": eq_info, "results": eq_results}

        return results

    def _classify_terminal(
        self,
        curr_guid: str,
        curr_node: Dict[str, Any],
        start_guid: str,
        current_equipment_id: str,
    ) -> Tuple[bool, str]:
        """
        현재 노드가 종단인지 판정합니다.

        개선 사항
        ---------
        1. 같은 장비 내부의 다른 POC와 다른 장비 POC를 구분합니다.
        2. name/id/type 비교에 strip + upper 정규화를 적용합니다.
        """
        if curr_guid == start_guid:
            return False, ""

        node_type = _normalize_text(curr_node.get("type"))
        node_name = _normalize_text(curr_node.get("name"))
        node_id = _normalize_text(curr_node.get("id"))
        owners = self.poc_owner_map.get(curr_guid, [])

        if owners and current_equipment_id not in owners:
            return True, "다른 장비 PoC 도달"

        if node_type in {"DUCT", "TAKEOFF"} or "DUCT" in node_name or "DUCT" in node_id:
            return True, "Duct / TakeOff 도달"

        if node_type in {"LATERAL", "LATERAL PIPE"} or "LATERAL" in node_name or "LATERAL" in node_id:
            return True, "Lateral Pipe 도달"

        if "NOZZLE" in node_name or "NOZZLE" in node_id:
            return True, "Nozzle PoC 도달"

        if node_type == "POC" and (node_name == "END" or node_id.startswith("END")):
            return True, "종단 PoC 도달"

        if node_type == "EQUIPMENT":
            return True, "장비 노드 도달"

        if not self.get_neighbors(curr_guid):
            return True, "배관 끝단(막힘)"

        return False, ""

    def _trace_paths_from_poc(
        self,
        start_node: Dict[str, Any],
        start_guid: str,
        current_equipment_id: str,
    ) -> List[Dict[str, Any]]:
        """한 개의 시작 POC에서 BFS로 모든 종단 경로를 추출합니다."""
        node_records: List[Dict[str, Any]] = [{
            "guid": start_guid,
            "node": start_node,
            "parent_idx": -1,
            "edge": None,
            "branch_info": {"depth": 0, "segments": []},
            "depth": 0,
        }]
        queue: deque = deque([(start_guid, 0, frozenset(), 0)])
        paths_found: List[Dict[str, Any]] = []

        while queue:
            if len(queue) > self.config.max_queue_size:
                LOGGER.warning(
                    "큐 크기 한도 초과로 탐색을 중단합니다. start=%s queue_size=%d",
                    start_guid,
                    len(queue),
                )
                break

            curr_guid, record_idx, visited_edges, depth = queue.popleft()
            curr_node = self.node_by_guid.get(curr_guid)
            if not curr_node:
                continue

            if depth > self.config.max_depth:
                LOGGER.warning("최대 깊이 초과: start=%s current=%s depth=%d", start_guid, curr_guid, depth)
                continue

            stop, label = self._classify_terminal(
                curr_guid=curr_guid,
                curr_node=curr_node,
                start_guid=start_guid,
                current_equipment_id=current_equipment_id,
            )
            if stop:
                branch_info = node_records[record_idx]["branch_info"]
                path_steps = self._reconstruct_path(node_records, record_idx)
                paths_found.append({
                    "end_node": curr_node,
                    "label": label,
                    "path": path_steps,
                    "branch_depth": branch_info["depth"],
                    "branch_segments": branch_info["segments"],
                })
                if len(paths_found) >= self.config.max_paths_per_poc:
                    LOGGER.warning(
                        "시작 POC별 최대 경로 수에 도달하여 탐색을 중단합니다. start=%s limit=%d",
                        start_guid,
                        self.config.max_paths_per_poc,
                    )
                    break
                continue

            neighbors = self.get_neighbors(curr_guid)
            is_branch = (curr_guid != start_guid) and self.is_branch_node(curr_guid)
            current_branch_info = node_records[record_idx]["branch_info"]

            for branch_index, (edge, next_guid) in enumerate(neighbors):
                edge_sig = _get_guid(edge) or edge.get("id") or f"direct_{curr_guid}_{next_guid}"

                # 사이클 방지 1: 이미 사용한 edge 재사용 금지
                if edge_sig in visited_edges:
                    continue

                # 사이클 방지 2: 현재 부모 체인에 동일 노드가 있으면 금지
                if self._is_in_path(node_records, record_idx, next_guid):
                    continue

                next_node = self.node_by_guid.get(next_guid)
                if not next_node:
                    continue

                if is_branch:
                    new_branch_info = {
                        "depth": current_branch_info["depth"] + 1,
                        "segments": current_branch_info["segments"] + [{
                            "branch_node_guid": curr_guid,
                            "branch_index": branch_index,
                            "branch_total": len(neighbors),
                        }],
                    }
                else:
                    new_branch_info = current_branch_info

                node_records.append({
                    "guid": next_guid,
                    "node": next_node,
                    "parent_idx": record_idx,
                    "edge": edge,
                    "branch_info": new_branch_info,
                    "depth": depth + 1,
                })
                queue.append((next_guid, len(node_records) - 1, visited_edges | {edge_sig}, depth + 1))

        return paths_found

    def _is_in_path(self, node_records: List[Dict[str, Any]], curr_idx: int, target_guid: str) -> bool:
        """부모 포인터를 거슬러 올라가며 현재 경로상 중복 노드 여부를 확인합니다."""
        idx = curr_idx
        while idx >= 0:
            if node_records[idx]["guid"] == target_guid:
                return True
            idx = node_records[idx]["parent_idx"]
        return False

    def _reconstruct_path(self, node_records: List[Dict[str, Any]], end_idx: int) -> List[Dict[str, Any]]:
        """부모 포인터를 통해 start -> end 방향의 NODE/EDGE 목록을 복원합니다."""
        path: List[Dict[str, Any]] = []
        idx = end_idx
        while idx >= 0:
            record = node_records[idx]
            path.append({"kind": "NODE", "data": record["node"]})
            if record["edge"] is not None:
                path.append({"kind": "EDGE", "data": record["edge"]})
            idx = record["parent_idx"]
        path.reverse()
        return path


# ─────────────────────────────────────────────────────────────
# 3. Grouping 유사도 / 클러스터링
# ─────────────────────────────────────────────────────────────


def pattern_similarity(arrow1: str, arrow2: str) -> float:
    """Levenshtein Distance 기반의 방향 패턴 유사도입니다."""
    if not arrow1 and not arrow2:
        return 1.0
    if not arrow1 or not arrow2:
        return 0.0

    a = arrow1.split("-")
    b = arrow2.split("-")
    m, n = len(a), len(b)
    dp = list(range(n + 1))

    for i in range(1, m + 1):
        prev = dp[:]
        dp[0] = i
        for j in range(1, n + 1):
            if a[i - 1] == b[j - 1]:
                dp[j] = prev[j - 1]
            else:
                dp[j] = 1 + min(prev[j], dp[j - 1], prev[j - 1])

    return _clamp01(1.0 - dp[n] / max(m, n))



def _cosine_similarity_0_1(v1: Dict[str, float], v2: Dict[str, float]) -> float:
    """
    코사인 유사도를 0~1 범위로 정규화합니다.

    기존 코드에서는 음수가 나올 수 있었으나,
    개선 버전에서는 반대 방향을 0점으로 처리합니다.
    """
    a = (float(v1["x"]), float(v1["y"]), float(v1["z"]))
    b = (float(v2["x"]), float(v2["y"]), float(v2["z"]))
    mag_a = math.sqrt(sum(x * x for x in a))
    mag_b = math.sqrt(sum(x * x for x in b))

    if mag_a == 0.0 and mag_b == 0.0:
        return 1.0
    if mag_a == 0.0 or mag_b == 0.0:
        return 0.0

    dot = sum(a[i] * b[i] for i in range(3))
    cosine = dot / (mag_a * mag_b)
    return _clamp01(max(0.0, cosine))



def _vector_sequence_similarity(vectors1: List[Dict[str, float]], vectors2: List[Dict[str, float]]) -> float:
    if not vectors1 and not vectors2:
        return 1.0
    if not vectors1 or not vectors2:
        return 0.0

    min_len = min(len(vectors1), len(vectors2))
    max_len = max(len(vectors1), len(vectors2))
    if min_len == 0:
        return 0.0

    score_sum = 0.0
    for i in range(min_len):
        score_sum += _cosine_similarity_0_1(vectors1[i], vectors2[i])

    # 길이 차이 패널티를 곱해줍니다. 앞부분만 같고 뒤가 긴 경로를 과대평가하지 않기 위함입니다.
    prefix_score = score_sum / min_len
    coverage = min_len / max_len
    return _clamp01(prefix_score * coverage)



def _range_similarity(r1: Dict[str, float], r2: Dict[str, float]) -> float:
    scores = []
    for axis in ("x", "y", "z"):
        v1 = float(r1.get(axis, 0.0))
        v2 = float(r2.get(axis, 0.0))
        base = max(v1, v2, 1.0)
        scores.append(_clamp01(1.0 - (abs(v1 - v2) / base)))
    return sum(scores) / len(scores)



def _length_similarity(lengths1: List[float], lengths2: List[float]) -> float:
    total1 = sum(lengths1)
    total2 = sum(lengths2)
    if total1 == 0.0 and total2 == 0.0:
        return 1.0
    if total1 == 0.0 or total2 == 0.0:
        return 0.0
    return _clamp01(1.0 - abs(total1 - total2) / max(total1, total2, 1.0))



def _obstacle_relation_similarity(r1: Dict[str, Any], r2: Dict[str, Any]) -> float:
    """장애물관계 특징량의 유사도를 계산합니다."""
    or1 = r1.get("obstacle_relations")
    or2 = r2.get("obstacle_relations")
    if or1 is None or or2 is None:
        return 1.0  # 장애물 데이터 없으면 중립

    def _count_sim(v1: int, v2: int) -> float:
        return _clamp01(1.0 - abs(v1 - v2) / max(v1, v2, 1))

    def _val_sim(v1: float, v2: float) -> float:
        return _clamp01(1.0 - abs(v1 - v2) / max(abs(v1), abs(v2), 1.0))

    def _inf_safe_sim(v1: float, v2: float) -> float:
        INF = float("inf")
        if v1 >= INF and v2 >= INF:
            return 1.0
        if v1 >= INF or v2 >= INF:
            return 0.3
        return _val_sim(v1, v2)

    # 기둥 (0.35)
    s_col = (
        _count_sim(or1.get("col_count_nearby", 0), or2.get("col_count_nearby", 0)) * 0.2
        + _inf_safe_sim(or1.get("col_min_distance", float("inf")), or2.get("col_min_distance", float("inf"))) * 0.2
        + _count_sim(or1.get("col_crossings", 0), or2.get("col_crossings", 0)) * 0.3
        + pattern_similarity(or1.get("col_relative_pattern", ""), or2.get("col_relative_pattern", "")) * 0.3
    )

    # 포스트 (0.15)
    s_post = (
        _val_sim(or1.get("post_density", 0), or2.get("post_density", 0)) * 0.3
        + _val_sim(or1.get("post_grid_alignment", 0), or2.get("post_grid_alignment", 0)) * 0.4
        + _count_sim(or1.get("post_count_nearby", 0), or2.get("post_count_nearby", 0)) * 0.3
    )

    # 빔 (0.30)
    s_beam = (
        _count_sim(or1.get("beam_count_crossing", 0), or2.get("beam_count_crossing", 0)) * 0.4
        + _inf_safe_sim(or1.get("beam_min_clearance", float("inf")), or2.get("beam_min_clearance", float("inf"))) * 0.3
        + _val_sim(or1.get("beam_parallel_ratio", 0), or2.get("beam_parallel_ratio", 0)) * 0.3
    )

    # 그레이팅 (0.20)
    s_grating = (
        _val_sim(or1.get("grating_coverage", 0), or2.get("grating_coverage", 0)) * 0.4
        + _count_sim(or1.get("grating_count_below", 0), or2.get("grating_count_below", 0)) * 0.3
        + _count_sim(or1.get("grating_gap_count", 0), or2.get("grating_gap_count", 0)) * 0.3
    )

    return s_col * 0.35 + s_post * 0.15 + s_beam * 0.30 + s_grating * 0.20


def compute_composite_similarity(r1: Dict[str, Any], r2: Dict[str, Any]) -> float:
    """
    복합 유사도 계산.

    가중치 (장애물관계 포함)
    ------
    1. Arrow Similarity      : 0.25
    2. Vector Similarity     : 0.25
    3. Range Similarity      : 0.15
    4. Length Similarity      : 0.15
    5. Obstacle Relation Sim : 0.20
    """
    s_arrow = pattern_similarity(r1.get("path_arrow", ""), r2.get("path_arrow", ""))
    s_vector = _vector_sequence_similarity(r1.get("path_step_vectors", []), r2.get("path_step_vectors", []))
    s_range = _range_similarity(r1.get("path_range", {}), r2.get("path_range", {}))
    s_length = _length_similarity(r1.get("path_step_lengths", []), r2.get("path_step_lengths", []))
    s_obstacle = _obstacle_relation_similarity(r1, r2)
    score = (s_arrow * 0.25) + (s_vector * 0.25) + (s_range * 0.15) + (s_length * 0.15) + (s_obstacle * 0.20)
    return round(_clamp01(score), 6)


class GroupAnalyzer:
    """추출된 경로 레코드를 그룹 배관 후보로 묶습니다."""

    def __init__(self, records: List[Dict[str, Any]], config: AnalysisConfig):
        self.records = records
        self.config = config

    def find_groups(self) -> List[Dict[str, Any]]:
        """동일 장비 + 유틸리티 + size 버킷 내에서 유사한 경로를 클러스터링합니다."""
        buckets: Dict[Tuple[str, str, str, str, str, str], List[Dict[str, Any]]] = defaultdict(list)
        for record in self.records:
            key = (
                record.get("equipment_name", "-"),
                record.get("equipment_id", "-"),
                record.get("equipment_process", "-"),
                record.get("equipment_maker", "-"),
                record.get("utility", "-"),
                str(record.get("size", "-")),
            )
            buckets[key].append(record)

        candidates: List[Dict[str, Any]] = []
        for key, bucket_records in buckets.items():
            if len(bucket_records) < self.config.min_group_size:
                continue

            (
                eq_name,
                eq_id,
                eq_process,
                eq_maker,
                utility,
                size,
            ) = key

            n = len(bucket_records)
            sim_matrix = [[0.0] * n for _ in range(n)]
            parent = list(range(n))

            def _find(x: int) -> int:
                while parent[x] != x:
                    parent[x] = parent[parent[x]]
                    x = parent[x]
                return x

            def _union(x: int, y: int) -> None:
                parent[_find(x)] = _find(y)

            for i in range(n):
                sim_matrix[i][i] = 1.0
                for j in range(i + 1, n):
                    similarity = compute_composite_similarity(bucket_records[i], bucket_records[j])
                    sim_matrix[i][j] = similarity
                    sim_matrix[j][i] = similarity
                    if similarity >= self.config.pattern_similarity_min:
                        _union(i, j)

            clusters: Dict[int, List[Tuple[int, Dict[str, Any]]]] = defaultdict(list)
            for idx, rec in enumerate(bucket_records):
                clusters[_find(idx)].append((idx, rec))

            for cluster_items in clusters.values():
                if len(cluster_items) < self.config.min_group_size:
                    continue

                indices = [idx for idx, _ in cluster_items]
                recs = [rec for _, rec in cluster_items]
                start_positions = [rec["start_pos"] for rec in recs]
                max_xy = max(_dist_xy(a, b) for a, b in itertools.combinations(start_positions, 2)) if len(start_positions) >= 2 else 0.0
                if max_xy > self.config.start_poc_xy_max:
                    continue

                common_z_levels = self._find_common_z_levels(recs)
                if self.config.require_common_z_levels and not common_z_levels:
                    continue

                pairs = list(itertools.combinations(range(len(indices)), 2))
                avg_similarity = (
                    sum(sim_matrix[indices[i]][indices[j]] for i, j in pairs) / len(pairs)
                    if pairs else 1.0
                )

                unique_poc_count = len({rec.get("poc_id") for rec in recs})
                candidates.append({
                    "equipment_name": eq_name,
                    "equipment_id": eq_id,
                    "equipment_process": eq_process,
                    "equipment_maker": eq_maker,
                    "utility": utility,
                    "size": size,
                    "path_count": len(recs),
                    "unique_poc_count": unique_poc_count,
                    "paths": recs,
                    "avg_similarity": round(avg_similarity, 4),
                    "max_start_xy_dist": round(max_xy, 1),
                    "common_z_levels": common_z_levels,
                })

        candidates.sort(key=lambda x: (-x["path_count"], -x["avg_similarity"], x["equipment_name"]))
        return candidates

    def _find_common_z_levels(self, recs: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        수평(H) 구간의 mean_z를 이용해 공통 레벨을 찾습니다.

        개선 사항
        ---------
        1. 단순 시드 기준이 아니라 현재 클러스터 평균 Z 기준으로 확장합니다.
        2. path_count와 xy_spread를 함께 저장합니다.
        """
        all_segments: List[Tuple[int, float, Tuple[float, float]]] = []
        for rec_idx, rec in enumerate(recs):
            for seg in rec.get("h_segments", []):
                all_segments.append((rec_idx, float(seg["mean_z"]), tuple(seg["mid_xy"])))

        if not all_segments:
            return []

        all_segments.sort(key=lambda item: item[1])
        results: List[Dict[str, Any]] = []
        current_cluster: List[Tuple[int, float, Tuple[float, float]]] = [all_segments[0]]

        def _flush_cluster(cluster: List[Tuple[int, float, Tuple[float, float]]]) -> None:
            if not cluster:
                return
            rec_ids = {item[0] for item in cluster}
            if len(rec_ids) < self.config.min_group_size:
                return
            zs = [item[1] for item in cluster]
            xys = [item[2] for item in cluster]
            spread = _xy_spread([(xy[0], xy[1], 0.0) for xy in xys])
            results.append({
                "mean_z": round(sum(zs) / len(zs), 1),
                "path_count": len(rec_ids),
                "xy_spread": round(spread, 1),
            })

        for seg in all_segments[1:]:
            cluster_mean = sum(item[1] for item in current_cluster) / len(current_cluster)
            if abs(seg[1] - cluster_mean) <= self.config.tol_z_level:
                current_cluster.append(seg)
            else:
                _flush_cluster(current_cluster)
                current_cluster = [seg]
        _flush_cluster(current_cluster)

        results.sort(key=lambda item: item["mean_z"])
        return results



def detect_zones(candidate: Dict[str, Any], config: AnalysisConfig) -> Dict[str, Any]:
    """
    그룹 배관 후보의 trunk / fan-in / fan-out 영역을 추정합니다.

    개선 사항
    ---------
    순수 Z값 비교만으로 전체 노드를 분류하지 않고,
    각 경로에서 trunk 대역에 해당하는 연속 index 범위를 기준으로 앞/중간/뒤를 나눕니다.
    """
    common_z_levels = candidate.get("common_z_levels", [])
    if not common_z_levels:
        return {
            "trunk": None,
            "fan_in": None,
            "fan_out": None,
            "is_trunk_estimated": False,
        }

    trunk_candidates = [z for z in common_z_levels if z["xy_spread"] <= config.trunk_max_xy_spread]
    is_estimated = False
    if trunk_candidates:
        trunk_info = max(trunk_candidates, key=lambda item: (item["path_count"], -item["xy_spread"]))
    else:
        trunk_info = min(common_z_levels, key=lambda item: item["xy_spread"])
        is_estimated = True

    trunk_z = trunk_info["mean_z"]
    z_band = config.tol_z_level * config.trunk_z_band_factor

    fan_in_positions: List[Tuple[float, float, float]] = []
    trunk_positions: List[Tuple[float, float, float]] = []
    fan_out_positions: List[Tuple[float, float, float]] = []

    for path in candidate.get("paths", []):
        positions = path.get("node_positions", [])
        if not positions:
            continue

        trunk_indices = [idx for idx, pos in enumerate(positions) if abs(pos[2] - trunk_z) <= z_band]
        if trunk_indices:
            first_idx = trunk_indices[0]
            last_idx = trunk_indices[-1]
            fan_in_positions.extend(positions[:first_idx])
            trunk_positions.extend(positions[first_idx:last_idx + 1])
            fan_out_positions.extend(positions[last_idx + 1:])
        else:
            # trunk 대역과 겹치지 않는 경로는 보조적으로 Z 차이에 따라 배분합니다.
            for pos in positions:
                if pos[2] < trunk_z - z_band:
                    fan_in_positions.append(pos)
                elif pos[2] > trunk_z + z_band:
                    fan_out_positions.append(pos)

    trunk_box = _bbox(trunk_positions)
    if trunk_box:
        trunk_box.update({
            "mean_z": trunk_z,
            "path_count": trunk_info["path_count"],
        })

    return {
        "trunk": trunk_box,
        "fan_in": _bbox(fan_in_positions),
        "fan_out": _bbox(fan_out_positions),
        "is_trunk_estimated": is_estimated,
    }


# ─────────────────────────────────────────────────────────────
# 4. Phase 1 결과 입출력
# ─────────────────────────────────────────────────────────────


def _compute_obstacle_relations_for_path(
    node_positions: List[Tuple[float, float, float]],
    spatial_ctx,
) -> Optional[Dict[str, Any]]:
    """SpatialContext가 있으면 경로에 대한 장애물관계를 딕셔너리로 반환합니다."""
    if spatial_ctx is None or len(node_positions) < 2:
        return None
    try:
        from AutoRoutingDesigner_V2 import ObstacleRelationExtractor
        extractor = ObstacleRelationExtractor(spatial_ctx)
        start_pos = node_positions[0]
        end_pos = node_positions[-1]
        radius = max(_dist_xyz(start_pos, end_pos) * 0.8, 2000.0)
        feat = extractor.extract(node_positions, radius)
        from dataclasses import asdict
        return asdict(feat)
    except Exception:
        LOGGER.debug("장애물관계 추출 실패", exc_info=True)
        return None


def run_phase_routing(config: AnalysisConfig) -> List[str]:
    """원본 JSON을 읽어 경로 추출 결과(JSON)를 생성합니다."""
    os.makedirs(config.routing_out, exist_ok=True)
    json_files = glob.glob(os.path.join(config.input_dir, "**", "*.json"), recursive=True)
    LOGGER.info("[Phase 1] 원본 JSON %d개 처리 시작", len(json_files))

    output_files: List[str] = []
    for file_path in json_files:
        graph = RoutingGraph(config)
        if not graph.load_from_json(file_path):
            continue

        # SpatialContext 로드 (장애물관계 분석용)
        spatial_ctx = None
        try:
            from AutoRoutingDesigner_V2 import SpatialContext
            spatial_ctx = SpatialContext()
            if not spatial_ctx.load_from_json(file_path):
                spatial_ctx = None
        except Exception:
            LOGGER.debug("SpatialContext 로드 실패 (장애물관계 미포함)", exc_info=True)
            spatial_ctx = None

        results = graph.find_routing_paths()
        file_base = os.path.splitext(os.path.basename(file_path))[0]
        if not results:
            LOGGER.info("경로 결과 없음: %s", file_path)
            continue

        for eq_id, eq_data in results.items():
            info = eq_data["info"]
            output_data = {
                "source_file": file_path,
                "equipment_id": info["id"],
                "equipment_name": info["name"],
                "equipment_process": info["process"],
                "equipment_maker": info["maker"],
                "created_at": datetime.now().isoformat(),
                "poc_paths": [],
            }

            for poc_result in eq_data["results"]:
                poc_node = poc_result["poc"]
                poc_entry = {
                    "start_poc_id": _get_guid(poc_node),
                    "start_poc_info": poc_node,
                    "paths": [],
                }

                for path_data in poc_result["paths"]:
                    steps = path_data["path"]
                    features = _compute_path_features(steps, config)
                    # 장애물관계 특징량 추가
                    obs_relations = _compute_obstacle_relations_for_path(
                        features.get("node_positions", []), spatial_ctx
                    )
                    path_entry = {
                        "terminal_label": path_data["label"],
                        "branch_depth": path_data.get("branch_depth", 0),
                        "branch_segments": path_data.get("branch_segments", []),
                        "path_summary": _build_path_summary(steps),
                        "end_node_id": _get_guid(path_data["end_node"]),
                        "end_node_type": path_data["end_node"].get("type", "-"),
                        "steps": steps,
                        **features,
                    }
                    if obs_relations is not None:
                        path_entry["obstacle_relations"] = obs_relations
                    poc_entry["paths"].append(path_entry)

                output_data["poc_paths"].append(poc_entry)

            out_path = os.path.join(
                config.routing_out,
                f"{_safe_filename(file_base)}_{_safe_filename(info['name'])}_Path.json",
            )
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(output_data, f, indent=2, ensure_ascii=False)
            output_files.append(out_path)
            LOGGER.info("라우팅 결과 저장: %s", out_path)

    return output_files



def _load_routing_records(config: AnalysisConfig) -> List[Dict[str, Any]]:
    """Phase 1 결과 파일들을 읽어 Grouping용 레코드 목록으로 변환합니다."""
    routing_files = glob.glob(os.path.join(config.routing_out, "*.json"))
    LOGGER.info("[Phase 2] 라우팅 결과 %d개 로딩", len(routing_files))
    records: List[Dict[str, Any]] = []

    for routing_file in routing_files:
        try:
            with open(routing_file, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            LOGGER.exception("라우팅 결과 로딩 실패: %s", routing_file)
            continue

        eq_name = data.get("equipment_name", "-")
        eq_id = data.get("equipment_id", "-")
        eq_process = data.get("equipment_process", "-")
        eq_maker = data.get("equipment_maker", "-")

        for poc_entry in data.get("poc_paths", []):
            start_poc_info = poc_entry.get("start_poc_info", {})
            utility = start_poc_info.get("utility") or start_poc_info.get("Utility") or "-"
            size = start_poc_info.get("size") or start_poc_info.get("Size") or "-"
            start_pos = _get_position(start_poc_info)

            for path in poc_entry.get("paths", []):
                steps = path.get("steps", [])
                node_positions = path.get("node_positions") or _ensure_list_of_positions(steps)
                path_arrow = path.get("path_arrow") or _compute_path_arrow(node_positions, config.direction_angle_tolerance)
                path_step_vectors = path.get("path_step_vectors")
                if path_step_vectors is None:
                    path_step_vectors = _parse_legacy_vectors(path.get("path_step_vector"))
                else:
                    path_step_vectors = _parse_legacy_vectors(path_step_vectors)

                path_step_lengths = path.get("path_step_lengths")
                if path_step_lengths is None:
                    path_step_lengths = _parse_legacy_lengths(path.get("path_step_length"))
                else:
                    path_step_lengths = _parse_legacy_lengths(path_step_lengths)

                path_range = path.get("path_range")
                if not isinstance(path_range, dict):
                    bbox = _bbox(node_positions)
                    path_range = {
                        "x": bbox["x_range"] if bbox else 0.0,
                        "y": bbox["y_range"] if bbox else 0.0,
                        "z": bbox["z_range"] if bbox else 0.0,
                    }

                h_segments = path.get("h_segments")
                if h_segments is None:
                    h_segments = _extract_h_segments(node_positions, path_arrow)

                record = {
                    "equipment_name": eq_name,
                    "equipment_id": eq_id,
                    "equipment_process": eq_process,
                    "equipment_maker": eq_maker,
                    "poc_id": poc_entry.get("start_poc_id"),
                    "utility": utility,
                    "size": size,
                    "start_pos": start_pos,
                    "path_summary": path.get("path_summary"),
                    "terminal_label": path.get("terminal_label"),
                    "end_node_id": path.get("end_node_id"),
                    "end_node_type": path.get("end_node_type"),
                    "node_positions": node_positions,
                    "h_segments": h_segments,
                    "path_arrow": path_arrow,
                    "path_range": path_range,
                    "path_step_lengths": path_step_lengths,
                    "path_step_vectors": path_step_vectors,
                    "path_total_length": round(sum(path_step_lengths), 1),
                    # 하위 호환용 원본 필드도 유지
                    "path_step_length": path.get("path_step_length") or _serialize_lengths(path_step_lengths),
                    "path_step_vector": path.get("path_step_vector") or _serialize_vectors(path_step_vectors),
                }
                # 장애물관계 특징량 (있는 경우)
                obs_rel = path.get("obstacle_relations")
                if obs_rel is not None:
                    record["obstacle_relations"] = obs_rel
                records.append(record)

    return records


# ─────────────────────────────────────────────────────────────
# 5. Phase 2 결과 저장
# ─────────────────────────────────────────────────────────────


def run_phase_grouping(config: AnalysisConfig) -> Tuple[str, str, List[Dict[str, Any]]]:
    """Phase 1 결과를 읽어 그룹 배관 후보를 산출하고 JSON/CSV로 저장합니다."""
    os.makedirs(config.group_out, exist_ok=True)
    records = _load_routing_records(config)
    LOGGER.info("[Phase 2] 분석 대상 경로 수: %d", len(records))

    analyzer = GroupAnalyzer(records, config)
    candidates = analyzer.find_groups()
    LOGGER.info("[Phase 2] 그룹 후보 %d개 탐지", len(candidates))

    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    json_path = os.path.join(config.group_out, f"group_pipe_results_{timestamp}.json")
    csv_path = os.path.join(config.group_out, f"group_pipe_results_{timestamp}.csv")

    final_output: List[Dict[str, Any]] = []
    for group_id, candidate in enumerate(candidates, start=1):
        zones = detect_zones(candidate, config)
        final_output.append({
            "group_id": group_id,
            "equipment_process": candidate["equipment_process"],
            "equipment_maker": candidate["equipment_maker"],
            "equipment_name": candidate["equipment_name"],
            "equipment_id": candidate["equipment_id"],
            "utility": candidate["utility"],
            "size": candidate["size"],
            "path_count": candidate["path_count"],
            "unique_poc_count": candidate["unique_poc_count"],
            "avg_similarity": candidate["avg_similarity"],
            "max_start_xy_dist": candidate["max_start_xy_dist"],
            "common_z_levels": candidate["common_z_levels"],
            "zones": zones,
            "paths": [{
                "poc_id": rec["poc_id"],
                "terminal_label": rec["terminal_label"],
                "start_pos": rec["start_pos"],
                "end_pos": rec["node_positions"][-1] if rec["node_positions"] else rec["start_pos"],
                "displacement_vector": {
                    "x": round(rec["node_positions"][-1][0] - rec["start_pos"][0], 1) if rec["node_positions"] else 0.0,
                    "y": round(rec["node_positions"][-1][1] - rec["start_pos"][1], 1) if rec["node_positions"] else 0.0,
                    "z": round(rec["node_positions"][-1][2] - rec["start_pos"][2], 1) if rec["node_positions"] else 0.0,
                },
                "path_summary": rec["path_summary"],
                "path_arrow": rec["path_arrow"],
                "path_range": rec["path_range"],
                "path_total_length": rec["path_total_length"],
                "path_step_lengths": rec["path_step_lengths"],
                "path_step_vectors": rec["path_step_vectors"],
            } for rec in candidate["paths"]],
        })

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(final_output, f, indent=2, ensure_ascii=False)

    header = [
        "group_id",
        "equipment_process",
        "equipment_maker",
        "equipment_name",
        "equipment_id",
        "utility",
        "size",
        "path_count",
        "unique_poc_count",
        "avg_similarity",
        "max_start_xy_dist",
        "trunk_z",
        "trunk_xy_spread",
        "trunk_path_count",
        "is_estimated",
        "fan_in_z_min",
        "fan_in_z_max",
        "fan_out_z_min",
        "fan_out_z_max",
        "poc_ids",
    ]

    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        for row in final_output:
            trunk = row["zones"]["trunk"]
            fan_in = row["zones"]["fan_in"]
            fan_out = row["zones"]["fan_out"]
            writer.writerow({
                "group_id": row["group_id"],
                "equipment_process": row["equipment_process"],
                "equipment_maker": row["equipment_maker"],
                "equipment_name": row["equipment_name"],
                "equipment_id": row["equipment_id"],
                "utility": row["utility"],
                "size": row["size"],
                "path_count": row["path_count"],
                "unique_poc_count": row["unique_poc_count"],
                "avg_similarity": row["avg_similarity"],
                "max_start_xy_dist": row["max_start_xy_dist"],
                "trunk_z": trunk["mean_z"] if trunk else "-",
                "trunk_xy_spread": trunk["xy_spread"] if trunk else "-",
                "trunk_path_count": trunk["path_count"] if trunk else "-",
                "is_estimated": row["zones"]["is_trunk_estimated"],
                "fan_in_z_min": fan_in["z_min"] if fan_in else "-",
                "fan_in_z_max": fan_in["z_max"] if fan_in else "-",
                "fan_out_z_min": fan_out["z_min"] if fan_out else "-",
                "fan_out_z_max": fan_out["z_max"] if fan_out else "-",
                "poc_ids": "|".join(str(path["poc_id"]) for path in row["paths"]),
            })

    LOGGER.info("그룹 분석 JSON 저장: %s", json_path)
    LOGGER.info("그룹 분석 CSV 저장: %s", csv_path)
    return json_path, csv_path, final_output


# ─────────────────────────────────────────────────────────────
# 6. CLI
# ─────────────────────────────────────────────────────────────


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="배관 경로 탐색 및 그룹 배관 분석 개선 버전")
    parser.add_argument("--phase", choices=["all", "routing", "grouping"], default="all", help="실행할 단계")
    parser.add_argument("--input", default="./data-v10", help="원본 설계 JSON 디렉토리")
    parser.add_argument("--routing_out", default="./RoutingResults", help="라우팅 결과 저장 디렉토리")
    parser.add_argument("--group_out", default="./GroupPipeResults", help="그룹 분석 결과 저장 디렉토리")
    parser.add_argument("--log_level", default="INFO", help="로그 레벨 (DEBUG/INFO/WARNING/ERROR)")

    # 탐색 보호 장치
    parser.add_argument("--max_paths_per_poc", type=int, default=5000, help="POC별 최대 경로 수")
    parser.add_argument("--max_queue_size", type=int, default=100000, help="BFS 최대 큐 크기")
    parser.add_argument("--max_depth", type=int, default=512, help="BFS 최대 깊이")

    # 분석 임계값
    parser.add_argument("--pattern_similarity_min", type=float, default=0.70, help="그룹 인정 최소 유사도")
    parser.add_argument("--start_poc_xy_max", type=float, default=5000.0, help="시작 POC 간 최대 XY 거리")
    parser.add_argument("--tol_z_level", type=float, default=200.0, help="공통 수평 레벨 Z 허용 오차")
    parser.add_argument("--trunk_max_xy_spread", type=float, default=1500.0, help="TRUNK 최대 XY spread")
    parser.add_argument("--trunk_z_band_factor", type=float, default=2.0, help="TRUNK Z 밴드 배수")
    parser.add_argument("--min_group_size", type=int, default=2, help="최소 그룹 경로 수")
    parser.add_argument("--require_common_z_levels", action="store_true", help="공통 Z 레벨이 없는 후보는 제외")
    return parser



def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(message)s",
    )

    config = AnalysisConfig(
        input_dir=args.input,
        routing_out=args.routing_out,
        group_out=args.group_out,
        max_paths_per_poc=args.max_paths_per_poc,
        max_queue_size=args.max_queue_size,
        max_depth=args.max_depth,
        pattern_similarity_min=args.pattern_similarity_min,
        start_poc_xy_max=args.start_poc_xy_max,
        tol_z_level=args.tol_z_level,
        trunk_max_xy_spread=args.trunk_max_xy_spread,
        trunk_z_band_factor=args.trunk_z_band_factor,
        min_group_size=args.min_group_size,
        require_common_z_levels=args.require_common_z_levels,
    )

    if args.phase in {"all", "routing"}:
        run_phase_routing(config)

    if args.phase in {"all", "grouping"}:
        run_phase_grouping(config)

    LOGGER.info("완료")


if __name__ == "__main__":
    main()
