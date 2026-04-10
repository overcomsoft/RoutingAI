from __future__ import annotations

"""
AutoRoutingDesigner.py
==============================
GroupPipeResults의 그룹 배관 데이터를 템플릿으로 활용하여,
새로운 Start PoC → 목적지가 주어졌을 때 자동으로 배관 경로를 생성합니다.

알고리즘: Hybrid (Zone-Based Construction + Template Morphing)
- fan_in(하강) → trunk(수평 주행) → fan_out(상승) 3단계 구성
- 각 구간에서 템플릿 경로의 세그먼트 패턴을 참조하여 방향/길이 결정
- 피팅 세그먼트(짧은 R)는 원본 유지, 주행 세그먼트만 스케일링/회전

실행 예
-------
python AutoRoutingDesigner.py --equipment kscta01 --utility AKWW --size 20A \\
    --start 5500,28000,15495 --dest 5300,27000,15495

python AutoRoutingDesigner.py --batch_input new_pocs.json

python AutoRoutingDesigner.py --list_groups --filter_equipment kscta01
"""

import argparse
import json
import logging
import math
import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

from AnalyzeRoutingAi_V2 import (
    _bbox,
    _clamp01,
    _compute_path_arrow,
    _compute_segment_code,
    _cosine_similarity_0_1,
    _dist_xy,
    _dist_xyz,
    _extract_h_segments,
    _serialize_lengths,
    _serialize_vectors,
    detect_zones,
    pattern_similarity,
)

# ─────────────────────────────────────────────────────────────
# 0. 설정 값
# ─────────────────────────────────────────────────────────────

LOGGER = logging.getLogger("AutoRoutingDesigner")

# 피팅 세그먼트 판별 기준: 이 길이 이하의 R 세그먼트는 피팅으로 간주하여 원본 유지
FITTING_LENGTH_THRESHOLD = 150.0


@dataclass
class DesignConfig:
    """자동 경로 설계 설정"""
    # 입출력
    group_results_path: str = "./GroupPipeResults/group_pipe_results_20260405194814.json"
    output_dir: str = "./AutoRoutingResults"

    # 템플릿 매칭
    max_start_xy_distance: float = 8000.0
    min_direction_similarity: float = 0.3

    # 경로 생성
    elbow_clearance: float = 100.0
    trunk_approach_tolerance: float = 300.0
    min_segment_length: float = 50.0
    direction_angle_tolerance: float = 5.0
    fitting_length_threshold: float = FITTING_LENGTH_THRESHOLD

    # 검증
    max_length_ratio: float = 2.5
    min_length_ratio: float = 0.2


# ─────────────────────────────────────────────────────────────
# 1. TemplateSelector - 그룹 및 대표 경로 선택
# ─────────────────────────────────────────────────────────────


class TemplateSelector:
    """GroupPipeResults에서 새 PoC에 가장 적합한 그룹과 대표 경로를 선택합니다."""

    def __init__(self, groups: List[Dict[str, Any]], config: DesignConfig):
        self.groups = groups
        self.config = config

    def find_matching_group(
        self,
        equipment_name: str,
        utility: str,
        size: str,
        start_pos: Tuple[float, float, float],
    ) -> Optional[Dict[str, Any]]:
        """
        1차: equipment_name + utility + size 완전 일치 필터
        2차: start_pos XY 거리 기준 필터
        3차: (avg_similarity × path_count) 가중 점수로 최적 그룹 선택
        """
        candidates: List[Tuple[float, Dict[str, Any]]] = []

        for g in self.groups:
            if g["equipment_name"] != equipment_name:
                continue
            if g["utility"] != utility:
                continue
            if g["size"] != size:
                continue

            # 그룹 내 시작점들과의 최소 XY 거리
            min_xy = float("inf")
            for p in g["paths"]:
                sp = p["start_pos"]
                d = _dist_xy(start_pos, sp)
                if d < min_xy:
                    min_xy = d

            if min_xy > self.config.max_start_xy_distance:
                continue

            # 점수: 유사도 × 경로 수 (많을수록 신뢰도 높음), 거리 페널티
            score = g["avg_similarity"] * math.log2(g["path_count"] + 1)
            score *= _clamp01(1.0 - min_xy / self.config.max_start_xy_distance)
            candidates.append((score, g))

        if not candidates:
            # Fallback: equipment_name만 일치
            return self._fallback_match(equipment_name, utility, start_pos)

        candidates.sort(key=lambda x: -x[0])
        best = candidates[0]
        LOGGER.info(
            "매칭 그룹: #%d (%s/%s/%s) score=%.4f",
            best[1]["group_id"], equipment_name, utility, size, best[0],
        )
        return best[1]

    def _fallback_match(
        self,
        equipment_name: str,
        utility: str,
        start_pos: Tuple[float, float, float],
    ) -> Optional[Dict[str, Any]]:
        """완전 일치 실패 시 equipment_name만으로 최선의 그룹을 찾습니다."""
        candidates: List[Tuple[float, Dict[str, Any]]] = []

        for g in self.groups:
            if g["equipment_name"] != equipment_name:
                continue

            min_xy = min(
                _dist_xy(start_pos, p["start_pos"]) for p in g["paths"]
            )
            if min_xy > self.config.max_start_xy_distance * 1.5:
                continue

            score = g["avg_similarity"] * math.log2(g["path_count"] + 1) * 0.5
            candidates.append((score, g))

        if not candidates:
            LOGGER.warning("매칭 그룹을 찾을 수 없습니다: equipment=%s utility=%s", equipment_name, utility)
            return None

        candidates.sort(key=lambda x: -x[0])
        LOGGER.warning(
            "Fallback 매칭: #%d (utility=%s, size=%s)",
            candidates[0][1]["group_id"],
            candidates[0][1]["utility"],
            candidates[0][1]["size"],
        )
        return candidates[0][1]

    def find_representative_path(
        self,
        group: Dict[str, Any],
        start_pos: Tuple[float, float, float],
        dest_pos: Tuple[float, float, float],
    ) -> Dict[str, Any]:
        """
        그룹 내에서 새 경로와 가장 유사한 대표 경로를 선정합니다.
        - displacement_vector 코사인 유사도 (0.4)
        - start_pos XY 근접성 (0.3)
        - path_total_length 비율 유사도 (0.3)
        """
        new_disp = {
            "x": dest_pos[0] - start_pos[0],
            "y": dest_pos[1] - start_pos[1],
            "z": dest_pos[2] - start_pos[2],
        }
        new_total_est = _dist_xyz(start_pos, dest_pos) * 2.5  # 경로는 직선의 ~2.5배 추정

        best_score = -1.0
        best_path = group["paths"][0]

        for p in group["paths"]:
            dv = p.get("displacement_vector", {"x": 0, "y": 0, "z": 0})
            # 코사인 유사도
            cos_sim = _cosine_similarity_0_1(new_disp, dv)

            # XY 근접성
            sp = p["start_pos"]
            xy_dist = _dist_xy(start_pos, sp)
            max_xy = max(
                _dist_xy(start_pos, pp["start_pos"]) for pp in group["paths"]
            )
            xy_score = _clamp01(1.0 - xy_dist / max(max_xy, 1.0))

            # 길이 유사도
            tl = p.get("path_total_length", 1.0)
            len_score = _clamp01(1.0 - abs(tl - new_total_est) / max(tl, new_total_est, 1.0))

            score = cos_sim * 0.4 + xy_score * 0.3 + len_score * 0.3
            if score > best_score:
                best_score = score
                best_path = p

        LOGGER.info("대표 경로 선정: poc=%s score=%.4f", best_path.get("poc_id", "?"), best_score)
        return best_path


# ─────────────────────────────────────────────────────────────
# 2. ZoneAwarePathBuilder - Zone 기반 경로 구성
# ─────────────────────────────────────────────────────────────


def _reconstruct_positions(start_pos: Sequence[float], vectors: List[Dict[str, float]]) -> List[Tuple[float, float, float]]:
    """start_pos + step_vectors 누적으로 전체 좌표열을 복원합니다."""
    positions = [tuple(start_pos)]
    x, y, z = float(start_pos[0]), float(start_pos[1]), float(start_pos[2])
    for v in vectors:
        x += v["x"]
        y += v["y"]
        z += v["z"]
        positions.append((round(x, 1), round(y, 1), round(z, 1)))
    return positions


def _vec_length(v: Dict[str, float]) -> float:
    return math.sqrt(v["x"] ** 2 + v["y"] ** 2 + v["z"] ** 2)


def _vec_add(a: Dict[str, float], b: Dict[str, float]) -> Dict[str, float]:
    return {"x": a["x"] + b["x"], "y": a["y"] + b["y"], "z": a["z"] + b["z"]}


def _vec_scale(v: Dict[str, float], s: float) -> Dict[str, float]:
    return {"x": round(v["x"] * s, 1), "y": round(v["y"] * s, 1), "z": round(v["z"] * s, 1)}


def _segment_type(v: Dict[str, float], tol_deg: float = 5.0) -> str:
    """벡터의 R/H/D 유형을 판별합니다."""
    total = _vec_length(v)
    if total < 1e-9:
        return "H"
    v_angle = math.degrees(math.asin(min(abs(v["z"]) / total, 1.0)))
    if v_angle > (90.0 - tol_deg):
        return "R"
    if v_angle < tol_deg:
        return "H"
    return "D"


def _is_fitting_segment(v: Dict[str, float], threshold: float) -> bool:
    """짧은 R 세그먼트를 피팅(ELBOW, FLANGE 등)으로 판별합니다."""
    if _segment_type(v) != "R":
        return False
    return _vec_length(v) <= threshold


def _rotate_xy(vx: float, vy: float, angle_rad: float) -> Tuple[float, float]:
    """XY 평면에서 벡터를 회전합니다."""
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (
        round(vx * cos_a - vy * sin_a, 1),
        round(vx * sin_a + vy * cos_a, 1),
    )


def _snap_to_orthogonal(vx: float, vy: float) -> Tuple[float, float]:
    """벡터를 가장 가까운 직교 방향(X 또는 Y축)으로 스냅합니다."""
    if abs(vx) >= abs(vy):
        return (vx, 0.0)
    return (0.0, vy)


class ZoneAwarePathBuilder:
    """Zone 구조(fan_in → trunk → fan_out)를 기반으로 3단계 경로를 구성합니다."""

    def __init__(self, config: DesignConfig):
        self.config = config

    def identify_zone_segments(
        self,
        template_path: Dict[str, Any],
        trunk_z: float,
        z_band: float,
    ) -> Dict[str, List[Dict[str, float]]]:
        """
        템플릿 경로의 step_vectors를 fan_in / trunk / fan_out으로 3분할합니다.

        방법: 좌표를 따라가며 trunk Z 대역에 처음 진입하는 인덱스와
        마지막으로 나가는 인덱스를 찾아 3구간으로 분리합니다.
        """
        vectors = template_path.get("path_step_vectors", [])
        positions = _reconstruct_positions(template_path["start_pos"], vectors)

        # trunk 대역에 속하는 position 인덱스 찾기
        trunk_indices = [
            i for i, pos in enumerate(positions)
            if abs(pos[2] - trunk_z) <= z_band
        ]

        if not trunk_indices:
            # trunk 대역 진입 없음 → 전체를 fan_in으로 처리
            return {"fan_in": vectors, "trunk": [], "fan_out": []}

        first_trunk = trunk_indices[0]
        last_trunk = trunk_indices[-1]

        # vectors[i]는 positions[i] → positions[i+1] 이동
        fan_in_vecs = vectors[:max(first_trunk, 0)]
        trunk_vecs = vectors[max(first_trunk, 0):last_trunk]
        fan_out_vecs = vectors[last_trunk:]

        return {
            "fan_in": fan_in_vecs,
            "trunk": trunk_vecs,
            "fan_out": fan_out_vecs,
        }

    def build_path(
        self,
        new_start: Tuple[float, float, float],
        destination: Tuple[float, float, float],
        group: Dict[str, Any],
        template_path: Dict[str, Any],
    ) -> Dict[str, Any]:
        """
        3단계 경로를 구성합니다:
        1. fan_in: new_start → trunk 진입점
        2. trunk: 수평 주행
        3. fan_out: trunk 이탈점 → destination
        """
        trunk_zone = group["zones"].get("trunk")
        if not trunk_zone:
            return self._build_direct_path(new_start, destination, template_path)

        trunk_z = trunk_zone["mean_z"]
        z_band = self.config.trunk_approach_tolerance

        # 템플릿의 zone별 세그먼트 분류
        zone_segs = self.identify_zone_segments(template_path, trunk_z, z_band)

        # 템플릿의 XY 주행 방향 각도
        tmpl_disp = template_path.get("displacement_vector", {"x": 0, "y": 0, "z": 0})
        tmpl_angle = math.atan2(tmpl_disp["y"], tmpl_disp["x"])
        new_disp_x = destination[0] - new_start[0]
        new_disp_y = destination[1] - new_start[1]
        new_angle = math.atan2(new_disp_y, new_disp_x)
        rotation = new_angle - tmpl_angle

        # trunk 진입/이탈점 계산
        trunk_entry = self._compute_trunk_entry(new_start, trunk_z, zone_segs["fan_in"], rotation)
        trunk_exit = self._compute_trunk_exit(destination, trunk_z, zone_segs["fan_out"], rotation)

        # 3단계 구성
        fan_in_vecs = self._build_fan_in(new_start, trunk_entry, zone_segs["fan_in"], rotation)
        trunk_vecs = self._build_trunk(trunk_entry, trunk_exit, zone_segs["trunk"], rotation)
        fan_out_vecs = self._build_fan_out(trunk_exit, destination, zone_segs["fan_out"], rotation)

        all_vectors = fan_in_vecs + trunk_vecs + fan_out_vecs
        return self._assemble_path_result(new_start, destination, all_vectors)

    def _compute_trunk_entry(
        self,
        start_pos: Tuple[float, float, float],
        trunk_z: float,
        fan_in_vecs: List[Dict[str, float]],
        rotation: float,
    ) -> Tuple[float, float, float]:
        """fan_in 템플릿을 기반으로 trunk 진입점을 추정합니다."""
        if not fan_in_vecs:
            return (start_pos[0], start_pos[1], trunk_z)

        # 템플릿 fan_in의 총 XY 변위
        total_dx = sum(v["x"] for v in fan_in_vecs)
        total_dy = sum(v["y"] for v in fan_in_vecs)

        # 회전 적용
        rx, ry = _rotate_xy(total_dx, total_dy, rotation)
        rx, ry = _snap_to_orthogonal(rx, ry)

        return (
            round(start_pos[0] + rx, 1),
            round(start_pos[1] + ry, 1),
            trunk_z,
        )

    def _compute_trunk_exit(
        self,
        dest_pos: Tuple[float, float, float],
        trunk_z: float,
        fan_out_vecs: List[Dict[str, float]],
        rotation: float,
    ) -> Tuple[float, float, float]:
        """fan_out 템플릿을 역산하여 trunk 이탈점을 추정합니다."""
        if not fan_out_vecs:
            return (dest_pos[0], dest_pos[1], trunk_z)

        # fan_out은 trunk→dest 방향이므로 역산: dest - fan_out_total
        total_dx = sum(v["x"] for v in fan_out_vecs)
        total_dy = sum(v["y"] for v in fan_out_vecs)

        rx, ry = _rotate_xy(total_dx, total_dy, rotation)
        rx, ry = _snap_to_orthogonal(rx, ry)

        return (
            round(dest_pos[0] - rx, 1),
            round(dest_pos[1] - ry, 1),
            trunk_z,
        )

    def _build_fan_in(
        self,
        start_pos: Tuple[float, float, float],
        trunk_entry: Tuple[float, float, float],
        template_fan_in: List[Dict[str, float]],
        rotation: float,
    ) -> List[Dict[str, float]]:
        """
        start_pos에서 trunk_entry까지의 경로를 생성합니다.
        - R 세그먼트: Z 높이차에 맞게 조정
        - H 세그먼트: XY 거리에 비례하여 스케일링 + 회전
        - 피팅 세그먼트: 원본 유지
        """
        if not template_fan_in:
            return self._build_simple_connection(start_pos, trunk_entry)

        dz_needed = trunk_entry[2] - start_pos[2]
        dx_needed = trunk_entry[0] - start_pos[0]
        dy_needed = trunk_entry[1] - start_pos[1]

        # 템플릿의 R(비피팅) 세그먼트 총 Z 변위
        tmpl_r_dz = sum(
            v["z"] for v in template_fan_in
            if _segment_type(v) == "R" and not _is_fitting_segment(v, self.config.fitting_length_threshold)
        )
        # 템플릿의 H 세그먼트 총 XY 변위
        tmpl_h_dx = sum(v["x"] for v in template_fan_in if _segment_type(v) == "H")
        tmpl_h_dy = sum(v["y"] for v in template_fan_in if _segment_type(v) == "H")
        tmpl_h_dist = math.sqrt(tmpl_h_dx ** 2 + tmpl_h_dy ** 2)

        # 스케일링 비율
        scale_z = dz_needed / tmpl_r_dz if abs(tmpl_r_dz) > 1e-6 else 1.0
        needed_h_dist = math.sqrt(dx_needed ** 2 + dy_needed ** 2)
        scale_xy = needed_h_dist / tmpl_h_dist if tmpl_h_dist > 1e-6 else 1.0

        result: List[Dict[str, float]] = []
        for v in template_fan_in:
            seg_type = _segment_type(v)

            if _is_fitting_segment(v, self.config.fitting_length_threshold):
                # 피팅: 원본 유지 (방향 부호는 Z 방향에 맞게 조정)
                if dz_needed >= 0 and v["z"] < 0:
                    result.append({"x": v["x"], "y": v["y"], "z": -v["z"]})
                elif dz_needed < 0 and v["z"] > 0:
                    result.append({"x": v["x"], "y": v["y"], "z": -v["z"]})
                else:
                    result.append(dict(v))
            elif seg_type == "R":
                # 주행 R: Z 스케일링
                new_z = round(v["z"] * scale_z, 1)
                result.append({"x": 0.0, "y": 0.0, "z": new_z})
            elif seg_type == "H":
                # H: 회전 + 스케일링
                rx, ry = _rotate_xy(v["x"], v["y"], rotation)
                scaled = _vec_scale({"x": rx, "y": ry, "z": 0.0}, scale_xy)
                sx, sy = _snap_to_orthogonal(scaled["x"], scaled["y"])
                result.append({"x": sx, "y": sy, "z": 0.0})
            else:
                # D(대각): R + H 성분 분해 후 개별 스케일링
                h_part_x, h_part_y = _rotate_xy(v["x"], v["y"], rotation)
                h_part_x, h_part_y = _snap_to_orthogonal(
                    round(h_part_x * scale_xy, 1),
                    round(h_part_y * scale_xy, 1),
                )
                result.append({"x": h_part_x, "y": h_part_y, "z": round(v["z"] * scale_z, 1)})

        return result

    def _build_trunk(
        self,
        trunk_entry: Tuple[float, float, float],
        trunk_exit: Tuple[float, float, float],
        template_trunk: List[Dict[str, float]],
        rotation: float,
    ) -> List[Dict[str, float]]:
        """
        trunk 구간: 수평 레벨 고정, 피팅 세그먼트 유지, H 세그먼트 길이만 재분배.
        """
        dx = trunk_exit[0] - trunk_entry[0]
        dy = trunk_exit[1] - trunk_entry[1]

        if not template_trunk:
            # 템플릿 trunk이 없으면 직선 이동
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                return []
            sx, sy = _snap_to_orthogonal(dx, dy)
            return [{"x": round(sx, 1), "y": round(sy, 1), "z": 0.0}]

        # 피팅과 H 세그먼트 분리
        fitting_vecs: List[Dict[str, float]] = []
        h_indices: List[int] = []
        result: List[Dict[str, float]] = []

        for i, v in enumerate(template_trunk):
            if _is_fitting_segment(v, self.config.fitting_length_threshold):
                fitting_vecs.append(v)
                result.append(dict(v))
            elif _segment_type(v) in ("H", "D"):
                h_indices.append(i)
                result.append(None)  # placeholder
            else:
                # 비피팅 R 세그먼트 (trunk 내부 높이 조정)
                result.append(dict(v))

        # 피팅의 XY 소모량 계산
        fitting_dx = sum(v["x"] for v in fitting_vecs)
        fitting_dy = sum(v["y"] for v in fitting_vecs)

        # H 세그먼트에 배분할 XY 잔여량
        remaining_dx = dx - fitting_dx
        remaining_dy = dy - fitting_dy

        if h_indices:
            # 템플릿 H 세그먼트들의 XY 비율에 따라 잔여량 배분
            tmpl_h_lengths = []
            for idx in h_indices:
                v = template_trunk[idx]
                tmpl_h_lengths.append(math.sqrt(v["x"] ** 2 + v["y"] ** 2))
            total_h = sum(tmpl_h_lengths) or 1.0

            for j, idx in enumerate(h_indices):
                ratio = tmpl_h_lengths[j] / total_h
                hx = round(remaining_dx * ratio, 1)
                hy = round(remaining_dy * ratio, 1)
                sx, sy = _snap_to_orthogonal(hx, hy)
                result[idx] = {"x": sx, "y": sy, "z": 0.0}
        else:
            # H 세그먼트가 없으면 잔여량을 하나의 H로 추가
            if abs(remaining_dx) > 1e-6 or abs(remaining_dy) > 1e-6:
                sx, sy = _snap_to_orthogonal(round(remaining_dx, 1), round(remaining_dy, 1))
                result.append({"x": sx, "y": sy, "z": 0.0})

        # None이 남아있으면 제거
        return [v for v in result if v is not None]

    def _build_fan_out(
        self,
        trunk_exit: Tuple[float, float, float],
        dest_pos: Tuple[float, float, float],
        template_fan_out: List[Dict[str, float]],
        rotation: float,
    ) -> List[Dict[str, float]]:
        """trunk_exit에서 dest_pos까지의 경로를 생성합니다. fan_in과 유사한 로직."""
        if not template_fan_out:
            return self._build_simple_connection(trunk_exit, dest_pos)

        dz_needed = dest_pos[2] - trunk_exit[2]
        dx_needed = dest_pos[0] - trunk_exit[0]
        dy_needed = dest_pos[1] - trunk_exit[1]

        tmpl_r_dz = sum(
            v["z"] for v in template_fan_out
            if _segment_type(v) == "R" and not _is_fitting_segment(v, self.config.fitting_length_threshold)
        )
        tmpl_h_dx = sum(v["x"] for v in template_fan_out if _segment_type(v) == "H")
        tmpl_h_dy = sum(v["y"] for v in template_fan_out if _segment_type(v) == "H")
        tmpl_h_dist = math.sqrt(tmpl_h_dx ** 2 + tmpl_h_dy ** 2)

        scale_z = dz_needed / tmpl_r_dz if abs(tmpl_r_dz) > 1e-6 else 1.0
        needed_h_dist = math.sqrt(dx_needed ** 2 + dy_needed ** 2)
        scale_xy = needed_h_dist / tmpl_h_dist if tmpl_h_dist > 1e-6 else 1.0

        result: List[Dict[str, float]] = []
        for v in template_fan_out:
            seg_type = _segment_type(v)

            if _is_fitting_segment(v, self.config.fitting_length_threshold):
                if dz_needed >= 0 and v["z"] < 0:
                    result.append({"x": v["x"], "y": v["y"], "z": -v["z"]})
                elif dz_needed < 0 and v["z"] > 0:
                    result.append({"x": v["x"], "y": v["y"], "z": -v["z"]})
                else:
                    result.append(dict(v))
            elif seg_type == "R":
                new_z = round(v["z"] * scale_z, 1)
                result.append({"x": 0.0, "y": 0.0, "z": new_z})
            elif seg_type == "H":
                rx, ry = _rotate_xy(v["x"], v["y"], rotation)
                scaled = _vec_scale({"x": rx, "y": ry, "z": 0.0}, scale_xy)
                sx, sy = _snap_to_orthogonal(scaled["x"], scaled["y"])
                result.append({"x": sx, "y": sy, "z": 0.0})
            else:
                h_part_x, h_part_y = _rotate_xy(v["x"], v["y"], rotation)
                h_part_x, h_part_y = _snap_to_orthogonal(
                    round(h_part_x * scale_xy, 1),
                    round(h_part_y * scale_xy, 1),
                )
                result.append({"x": h_part_x, "y": h_part_y, "z": round(v["z"] * scale_z, 1)})

        return result

    def _build_simple_connection(
        self,
        from_pos: Tuple[float, float, float],
        to_pos: Tuple[float, float, float],
    ) -> List[Dict[str, float]]:
        """두 점 사이의 L자형 연결 경로를 생성합니다 (R → H 또는 H → R)."""
        dx = round(to_pos[0] - from_pos[0], 1)
        dy = round(to_pos[1] - from_pos[1], 1)
        dz = round(to_pos[2] - from_pos[2], 1)

        segments: List[Dict[str, float]] = []

        # 수직 이동이 있으면 R 세그먼트
        if abs(dz) > self.config.min_segment_length:
            segments.append({"x": 0.0, "y": 0.0, "z": dz})

        # 수평 이동이 있으면 H 세그먼트 (직교 분해)
        if abs(dx) > self.config.min_segment_length:
            segments.append({"x": dx, "y": 0.0, "z": 0.0})
        if abs(dy) > self.config.min_segment_length:
            segments.append({"x": 0.0, "y": dy, "z": 0.0})

        if not segments:
            # 거리가 너무 짧으면 단일 벡터
            segments.append({"x": dx, "y": dy, "z": dz})

        return segments

    def _build_direct_path(
        self,
        new_start: Tuple[float, float, float],
        destination: Tuple[float, float, float],
        template_path: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Trunk이 없는 경우 직선형 경로를 생성합니다."""
        vectors = self._build_simple_connection(new_start, destination)
        return self._assemble_path_result(new_start, destination, vectors)

    def _assemble_path_result(
        self,
        start_pos: Tuple[float, float, float],
        dest_pos: Tuple[float, float, float],
        vectors: List[Dict[str, float]],
    ) -> Dict[str, Any]:
        """벡터 리스트에서 최종 경로 결과 딕셔너리를 조립합니다."""
        positions = _reconstruct_positions(start_pos, vectors)
        lengths = [round(_vec_length(v), 1) for v in vectors]
        arrow = _compute_path_arrow(positions, self.config.direction_angle_tolerance)
        bbox = _bbox(positions)
        h_segments = _extract_h_segments(positions, arrow)

        actual_end = positions[-1] if positions else start_pos
        return {
            "start_pos": list(start_pos),
            "end_pos": list(actual_end),
            "displacement_vector": {
                "x": round(actual_end[0] - start_pos[0], 1),
                "y": round(actual_end[1] - start_pos[1], 1),
                "z": round(actual_end[2] - start_pos[2], 1),
            },
            "node_positions": positions,
            "path_step_vectors": vectors,
            "path_step_lengths": lengths,
            "path_total_length": round(sum(lengths), 1),
            "path_arrow": arrow,
            "path_bbox": bbox,
            "path_range": {
                "x": bbox["x_range"] if bbox else 0.0,
                "y": bbox["y_range"] if bbox else 0.0,
                "z": bbox["z_range"] if bbox else 0.0,
            },
            "h_segments": h_segments,
            "path_summary": "AUTO",
        }


# ─────────────────────────────────────────────────────────────
# 3. PathAdapter - 좌표 보정
# ─────────────────────────────────────────────────────────────


class PathAdapter:
    """생성된 경로의 좌표 보정 및 연속성 보장을 수행합니다."""

    def __init__(self, config: DesignConfig):
        self.config = config

    def adapt(
        self,
        path_result: Dict[str, Any],
        new_start: Tuple[float, float, float],
        destination: Tuple[float, float, float],
    ) -> Dict[str, Any]:
        """
        1. 시작점이 new_start와 일치하도록 보정
        2. 끝점이 destination에 도달하도록 보정 세그먼트 추가
        3. 최소 길이 미만 세그먼트 병합
        """
        vectors = list(path_result["path_step_vectors"])

        # 1. 끝점 보정: 실제 끝점과 destination 간 갭 계산
        positions = _reconstruct_positions(new_start, vectors)
        actual_end = positions[-1]
        gap_x = round(destination[0] - actual_end[0], 1)
        gap_y = round(destination[1] - actual_end[1], 1)
        gap_z = round(destination[2] - actual_end[2], 1)

        gap_dist = math.sqrt(gap_x ** 2 + gap_y ** 2 + gap_z ** 2)
        if gap_dist > 1.0:
            # 보정 세그먼트 추가
            correction = self._create_correction_segments(gap_x, gap_y, gap_z)
            vectors.extend(correction)
            LOGGER.debug("끝점 보정: gap=%.1f, segments=%d", gap_dist, len(correction))

        # 2. 최소 길이 미만 세그먼트 병합
        vectors = self._merge_short_segments(vectors)

        # 3. 영벡터 제거
        vectors = [v for v in vectors if _vec_length(v) > 0.1]

        # 결과 재조립
        positions = _reconstruct_positions(new_start, vectors)
        lengths = [round(_vec_length(v), 1) for v in vectors]
        arrow = _compute_path_arrow(positions, self.config.direction_angle_tolerance)
        bbox = _bbox(positions)

        actual_end = positions[-1] if positions else new_start
        path_result["start_pos"] = list(new_start)
        path_result["end_pos"] = list(actual_end)
        path_result["displacement_vector"] = {
            "x": round(actual_end[0] - new_start[0], 1),
            "y": round(actual_end[1] - new_start[1], 1),
            "z": round(actual_end[2] - new_start[2], 1),
        }
        path_result["node_positions"] = positions
        path_result["path_step_vectors"] = vectors
        path_result["path_step_lengths"] = lengths
        path_result["path_total_length"] = round(sum(lengths), 1)
        path_result["path_arrow"] = arrow
        path_result["path_bbox"] = bbox
        path_result["path_range"] = {
            "x": bbox["x_range"] if bbox else 0.0,
            "y": bbox["y_range"] if bbox else 0.0,
            "z": bbox["z_range"] if bbox else 0.0,
        }
        path_result["h_segments"] = _extract_h_segments(positions, arrow)

        return path_result

    def _create_correction_segments(self, dx: float, dy: float, dz: float) -> List[Dict[str, float]]:
        """갭을 메우는 보정 세그먼트를 생성합니다."""
        segments: List[Dict[str, float]] = []
        if abs(dz) > 1.0:
            segments.append({"x": 0.0, "y": 0.0, "z": dz})
        if abs(dx) > 1.0:
            segments.append({"x": dx, "y": 0.0, "z": 0.0})
        if abs(dy) > 1.0:
            segments.append({"x": 0.0, "y": dy, "z": 0.0})
        if not segments and (abs(dx) > 0.1 or abs(dy) > 0.1 or abs(dz) > 0.1):
            segments.append({"x": dx, "y": dy, "z": dz})
        return segments

    def _merge_short_segments(self, vectors: List[Dict[str, float]]) -> List[Dict[str, float]]:
        """연속된 동일 방향의 짧은 세그먼트를 병합합니다."""
        if len(vectors) <= 1:
            return vectors

        merged: List[Dict[str, float]] = [dict(vectors[0])]
        for v in vectors[1:]:
            prev = merged[-1]
            prev_type = _segment_type(prev)
            curr_type = _segment_type(v)

            # 같은 방향이고, 현재 세그먼트가 짧으면 병합
            if prev_type == curr_type and _vec_length(v) < self.config.min_segment_length:
                merged[-1] = _vec_add(prev, v)
            else:
                merged.append(dict(v))

        return merged


# ─────────────────────────────────────────────────────────────
# 4. PathValidator - 경로 검증
# ─────────────────────────────────────────────────────────────


class PathValidator:
    """생성된 경로의 품질을 검증합니다."""

    def __init__(self, config: DesignConfig):
        self.config = config

    def validate(
        self,
        generated_path: Dict[str, Any],
        template_path: Dict[str, Any],
        group: Dict[str, Any],
    ) -> Dict[str, Any]:
        """종합 검증 결과를 반환합니다."""
        warnings: List[str] = []
        checks: Dict[str, float] = {}

        # 1. 좌표 연속성
        checks["continuity"] = 1.0 if self._check_continuity(generated_path) else 0.0
        if checks["continuity"] < 1.0:
            warnings.append("좌표 연속성 검증 실패")

        # 2. 길이 합리성
        gen_len = generated_path.get("path_total_length", 0.0)
        tmpl_len = template_path.get("path_total_length", 1.0)
        ratio = gen_len / tmpl_len if tmpl_len > 0 else 0.0
        in_range = self.config.min_length_ratio <= ratio <= self.config.max_length_ratio
        checks["length_ratio"] = _clamp01(1.0 - abs(1.0 - ratio) / 2.0) if in_range else 0.2
        if not in_range:
            warnings.append(f"길이 비율 초과: {ratio:.2f} (허용: {self.config.min_length_ratio}~{self.config.max_length_ratio})")

        # 3. 방향 패턴 유사도
        gen_arrow = generated_path.get("path_arrow", "")
        tmpl_arrow = template_path.get("path_arrow", "")
        checks["pattern"] = pattern_similarity(gen_arrow, tmpl_arrow)
        if checks["pattern"] < 0.3:
            warnings.append(f"방향 패턴 유사도 낮음: {checks['pattern']:.3f}")

        # 4. Zone 범위 준수
        checks["zone_compliance"] = self._check_zone_compliance(generated_path, group)
        if checks["zone_compliance"] < 0.5:
            warnings.append("경로가 Zone 범위를 크게 벗어남")

        # 5. 자기 교차 검사
        has_intersection = self._check_self_intersection(generated_path)
        checks["no_intersection"] = 0.0 if has_intersection else 1.0
        if has_intersection:
            warnings.append("경로 자기 교차 감지")

        # 종합 점수
        score = (
            checks["continuity"] * 0.25
            + checks["length_ratio"] * 0.25
            + checks["pattern"] * 0.20
            + checks["zone_compliance"] * 0.15
            + checks["no_intersection"] * 0.15
        )

        return {
            "is_valid": score >= 0.4 and checks["continuity"] >= 1.0,
            "quality_score": round(score, 4),
            "checks": {k: round(v, 4) for k, v in checks.items()},
            "warnings": warnings,
        }

    def _check_continuity(self, path: Dict[str, Any]) -> bool:
        """start_pos + cumsum(vectors) == node_positions 인지 확인."""
        vectors = path.get("path_step_vectors", [])
        start = path.get("start_pos", [0, 0, 0])
        positions = path.get("node_positions", [])

        if not positions or not vectors:
            return True

        reconstructed = _reconstruct_positions(start, vectors)
        if len(reconstructed) != len(positions):
            return False

        for r, p in zip(reconstructed, positions):
            if any(abs(r[i] - p[i]) > 1.0 for i in range(3)):
                return False
        return True

    def _check_zone_compliance(self, path: Dict[str, Any], group: Dict[str, Any]) -> float:
        """경로 BBox가 그룹 zone의 확장 범위 이내인지 평가합니다."""
        path_bbox = path.get("path_bbox")
        trunk = group["zones"].get("trunk")
        if not path_bbox or not trunk:
            return 0.5  # 비교 불가 → 중립

        # trunk의 BBox를 1.5배 확장
        margin = 1.5
        score = 1.0

        for axis in ("x", "y"):
            t_min = trunk.get(f"{axis}_min", 0)
            t_max = trunk.get(f"{axis}_max", 0)
            t_range = (t_max - t_min) or 1000.0
            p_min = path_bbox.get(f"{axis}_min", 0)
            p_max = path_bbox.get(f"{axis}_max", 0)

            expanded_min = t_min - t_range * (margin - 1)
            expanded_max = t_max + t_range * (margin - 1)

            if p_min < expanded_min or p_max > expanded_max:
                overshoot = max(expanded_min - p_min, p_max - expanded_max, 0)
                penalty = _clamp01(overshoot / t_range)
                score -= penalty * 0.5

        return _clamp01(score)

    def _check_self_intersection(self, path: Dict[str, Any]) -> bool:
        """경로의 노드 좌표 중 중복이 있는지 검사합니다 (시작/끝 제외)."""
        positions = path.get("node_positions", [])
        if len(positions) <= 2:
            return False

        # 중간 노드만 검사 (시작과 끝이 같을 수 있으므로)
        middle = positions[1:-1]
        seen = set()
        for p in middle:
            key = (round(p[0], 0), round(p[1], 0), round(p[2], 0))
            if key in seen:
                return True
            seen.add(key)
        return False


# ─────────────────────────────────────────────────────────────
# 5. AutoRoutingDesigner - 메인 오케스트레이터
# ─────────────────────────────────────────────────────────────


class AutoRoutingDesigner:
    """자동 배관 경로 설계 메인 클래스"""

    def __init__(self, config: DesignConfig):
        self.config = config
        self.groups: List[Dict[str, Any]] = []
        self.selector: Optional[TemplateSelector] = None
        self.builder = ZoneAwarePathBuilder(config)
        self.adapter = PathAdapter(config)
        self.validator = PathValidator(config)

    def load_group_data(self, path: Optional[str] = None) -> None:
        """GroupPipeResults JSON을 로드합니다."""
        load_path = path or self.config.group_results_path
        LOGGER.info("그룹 데이터 로딩: %s", load_path)

        with open(load_path, "r", encoding="utf-8") as f:
            self.groups = json.load(f)

        self.selector = TemplateSelector(self.groups, self.config)
        LOGGER.info("로드 완료: %d개 그룹", len(self.groups))

    def design_route(
        self,
        equipment_name: str,
        utility: str,
        size: str,
        start_pos: Tuple[float, float, float],
        dest_pos: Tuple[float, float, float],
    ) -> Optional[Dict[str, Any]]:
        """단일 경로를 자동 설계합니다."""
        if not self.selector:
            LOGGER.error("그룹 데이터가 로드되지 않았습니다.")
            return None

        # 1. 그룹 매칭
        group = self.selector.find_matching_group(equipment_name, utility, size, start_pos)
        if not group:
            LOGGER.error("매칭 그룹 없음: %s/%s/%s", equipment_name, utility, size)
            return None

        # 2. 대표 경로 선정
        template = self.selector.find_representative_path(group, start_pos, dest_pos)

        # 3. 경로 구성
        raw_path = self.builder.build_path(start_pos, dest_pos, group, template)

        # 4. 좌표 보정
        adapted_path = self.adapter.adapt(raw_path, start_pos, dest_pos)

        # 5. 검증
        validation = self.validator.validate(adapted_path, template, group)

        # 결과 조립
        result = {
            "group_id": f"AUTO_{group['group_id']:03d}",
            "source_group_id": group["group_id"],
            "is_auto_generated": True,
            "equipment_process": group.get("equipment_process", "-"),
            "equipment_maker": group.get("equipment_maker", "-"),
            "equipment_name": equipment_name,
            "equipment_id": group.get("equipment_id", "-"),
            "utility": utility,
            "size": size,
            "path_count": 1,
            "unique_poc_count": 1,
            "avg_similarity": None,
            "max_start_xy_dist": 0.0,
            "common_z_levels": group.get("common_z_levels", []),
            "zones": group.get("zones", {}),
            "quality_score": validation["quality_score"],
            "validation": validation,
            "template_poc_id": template.get("poc_id"),
            "paths": [{
                "poc_id": f"auto_{datetime.now().strftime('%Y%m%d%H%M%S')}",
                "terminal_label": "자동 생성 경로",
                **adapted_path,
            }],
        }

        status = "PASS" if validation["is_valid"] else "WARN"
        LOGGER.info(
            "[%s] 경로 생성: %s/%s/%s score=%.4f length=%.1f warnings=%d",
            status, equipment_name, utility, size,
            validation["quality_score"],
            adapted_path["path_total_length"],
            len(validation["warnings"]),
        )
        for w in validation["warnings"]:
            LOGGER.warning("  - %s", w)

        return result

    def design_routes_batch(self, poc_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """복수 PoC를 일괄 처리합니다."""
        results: List[Dict[str, Any]] = []
        for i, poc in enumerate(poc_list):
            LOGGER.info("[배치 %d/%d] %s/%s/%s", i + 1, len(poc_list),
                        poc["equipment_name"], poc["utility"], poc["size"])

            start = tuple(poc["start_pos"])
            dest = tuple(poc["dest_pos"])
            result = self.design_route(
                equipment_name=poc["equipment_name"],
                utility=poc["utility"],
                size=poc["size"],
                start_pos=start,
                dest_pos=dest,
            )
            if result:
                results.append(result)

        LOGGER.info("배치 완료: %d/%d 성공", len(results), len(poc_list))
        return results

    def save_results(self, results: List[Dict[str, Any]], output_path: Optional[str] = None) -> str:
        """결과를 JSON 파일로 저장합니다."""
        os.makedirs(self.config.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

        if output_path:
            out_path = output_path
        else:
            out_path = os.path.join(
                self.config.output_dir,
                f"auto_routing_results_{timestamp}.json",
            )

        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)

        LOGGER.info("결과 저장: %s (%d개 경로)", out_path, len(results))
        return out_path

    def list_groups(
        self,
        filter_equipment: Optional[str] = None,
        filter_utility: Optional[str] = None,
    ) -> None:
        """사용 가능한 그룹 목록을 출력합니다."""
        print(f"\n{'ID':>4} | {'장비명':<25} | {'유틸리티':<12} | {'사이즈':<8} | {'경로수':>5} | {'유사도':>6} | {'Trunk Z':>10}")
        print("-" * 95)

        for g in self.groups:
            if filter_equipment and g["equipment_name"] != filter_equipment:
                continue
            if filter_utility and g["utility"] != filter_utility:
                continue

            trunk_z = g["zones"]["trunk"]["mean_z"] if g["zones"].get("trunk") else "-"
            print(
                f"{g['group_id']:>4} | {g['equipment_name']:<25} | {g['utility']:<12} | "
                f"{g['size']:<8} | {g['path_count']:>5} | {g['avg_similarity']:>6.3f} | "
                f"{trunk_z:>10}"
            )

        print()


# ─────────────────────────────────────────────────────────────
# 6. CLI
# ─────────────────────────────────────────────────────────────


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="자동 배관 경로 설계 시스템")

    parser.add_argument("--group_results", default="./GroupPipeResults/group_pipe_results_20260405194814.json",
                        help="GroupPipeResults JSON 파일 경로")
    parser.add_argument("--output", default="./AutoRoutingResults", help="출력 디렉토리")
    parser.add_argument("--log_level", default="INFO", help="로그 레벨")

    # 단일 경로 생성
    parser.add_argument("--equipment", help="장비명")
    parser.add_argument("--utility", help="유틸리티")
    parser.add_argument("--size", help="사이즈")
    parser.add_argument("--start", help="시작 좌표 (x,y,z)")
    parser.add_argument("--dest", help="목적지 좌표 (x,y,z)")

    # 배치 입력
    parser.add_argument("--batch_input", help="배치 입력 JSON 파일")

    # 그룹 목록 조회
    parser.add_argument("--list_groups", action="store_true", help="그룹 목록 조회")
    parser.add_argument("--filter_equipment", help="장비명 필터")
    parser.add_argument("--filter_utility", help="유틸리티 필터")

    return parser


def _parse_coords(s: str) -> Tuple[float, float, float]:
    parts = [float(x.strip()) for x in s.split(",")]
    if len(parts) != 3:
        raise ValueError(f"좌표는 x,y,z 형식이어야 합니다: {s}")
    return (parts[0], parts[1], parts[2])


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(message)s",
    )

    config = DesignConfig(
        group_results_path=args.group_results,
        output_dir=args.output,
    )

    designer = AutoRoutingDesigner(config)
    designer.load_group_data()

    # 그룹 목록 조회
    if args.list_groups:
        designer.list_groups(
            filter_equipment=args.filter_equipment,
            filter_utility=args.filter_utility,
        )
        return

    # 배치 입력
    if args.batch_input:
        with open(args.batch_input, "r", encoding="utf-8") as f:
            poc_list = json.load(f)
        results = designer.design_routes_batch(poc_list)
        if results:
            designer.save_results(results)
        return

    # 단일 경로 생성
    if args.equipment and args.utility and args.size and args.start and args.dest:
        start_pos = _parse_coords(args.start)
        dest_pos = _parse_coords(args.dest)

        result = designer.design_route(
            equipment_name=args.equipment,
            utility=args.utility,
            size=args.size,
            start_pos=start_pos,
            dest_pos=dest_pos,
        )

        if result:
            out_path = designer.save_results([result])
            print(f"\n결과 저장: {out_path}")
            print(f"품질 점수: {result['quality_score']:.4f}")
            print(f"경로 길이: {result['paths'][0]['path_total_length']:.1f} mm")
            print(f"방향 패턴: {result['paths'][0]['path_arrow']}")

            validation = result["validation"]
            if validation["warnings"]:
                print("\n경고:")
                for w in validation["warnings"]:
                    print(f"  - {w}")
        else:
            print("경로 생성 실패")
        return

    parser.print_help()


if __name__ == "__main__":
    main()
