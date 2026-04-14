from __future__ import annotations

# =====================================================================================
# [실행 명령어]
#   (단일 경로 자동설계)
#   python AutoRoutingDesigner_V2.py --json_input ./data-v10/CMP_KSCTA01_*.json \
#       --group_results ./GroupPipeResults/group_pipe_results_20260405194814.json \
#       --equipment kscta01 --utility AKWW --size 20A \
#       --start 5500,28200,15495 --dest 5300,27000,15495
#
#   (그룹 목록 조회)
#   python AutoRoutingDesigner_V2.py --json_input ./data-v10/CMP_KSCTA01_*.json \
#       --group_results ./GroupPipeResults/group_pipe_results_20260405194814.json \
#       --list_groups --filter_equipment kscta01
#
#   (공간 컨텍스트 요약)
#   python AutoRoutingDesigner_V2.py --json_input ./data-v10/CMP_KSCTA01_*.json \
#       --group_results ./GroupPipeResults/group_pipe_results_20260405194814.json \
#       --spatial_summary
#
#   (배치 자동설계)
#   python AutoRoutingDesigner_V2.py --json_input ./data-v10/CMP_KSCTA01_*.json \
#       --group_results ./GroupPipeResults/group_pipe_results_20260405194814.json \
#       --batch_input ./batch_poc_list.json
# =====================================================================================
"""
AutoRoutingDesigner_V2.py  — 장애물 인식 자동 배관 경로 설계 시스템 (V2)
========================================================================

[프로그램 개요]
  장비 형상, 장애물(구조기둥/포스트/H-빔/그레이팅), 종단점(부대장비/Duct/Lateral)
  위치를 모두 고려하여 기존 설계 데이터 기반으로 자동 배관 경로를 설계합니다.

[전체 흐름도]
  ┌──────────────────────────────────────────────────────────────────────────┐
  │  1. 데이터 로딩 (AutoRoutingDesignerV2.load_data)                        │
  │  ┌─────────────────────────────────────┐                                │
  │  │ 원본 설계 JSON 로드                  │                                │
  │  │  └→ SpatialContext 구축              │                                │
  │  │     ├─ EquipmentInfo (장비 BBox, POC)│                                │
  │  │     ├─ ObstacleInfo × N (6종 분류)   │ ← _classify_obstacle()        │
  │  │     │  ├─ STRUCTURAL_COLUMN (구조기둥)│                                │
  │  │     │  ├─ POST (포스트)              │                                │
  │  │     │  ├─ H_BEAM (H빔)              │                                │
  │  │     │  ├─ STRUCTURAL_BEAM (구조보)   │                                │
  │  │     │  ├─ GRATING (그레이팅)         │                                │
  │  │     │  └─ CEILING (천장)             │                                │
  │  │     └─ SpaceLevelInfo (CSF/A/F/CR)  │                                │
  │  │ GroupPipeResults JSON 로드           │ ← Phase 2 결과                 │
  │  └─────────────────────────────────────┘                                │
  ├──────────────────────────────────────────────────────────────────────────┤
  │  2. 템플릿 매칭 (find_matching_group)                                    │
  │  ┌─────────────────────────────────────┐                                │
  │  │ 새 경로의 대략적 벡터 추정           │ ← _estimate_rough_vectors()    │
  │  │ EnhancedFeatureExtractor로 8차원 특징│                                │
  │  │ 그룹 내 모든 템플릿과 8차원 유사도:  │                                │
  │  │  0.12×arrow + 0.12×vector           │                                │
  │  │  0.08×range + 0.08×length           │                                │
  │  │  0.15×장비상대 + 0.15×종단           │                                │
  │  │  0.20×장애물관계 + 0.10×레벨         │ ← SpatialSimilarity.compute() │
  │  │ 최고 유사도 템플릿 선택              │                                │
  │  └─────────────────────────────────────┘                                │
  ├──────────────────────────────────────────────────────────────────────────┤
  │  3. 경로 생성 (ObstacleAwarePathBuilder.build_path)                      │
  │  ┌─────────────────────────────────────┐                                │
  │  │ Zone 분류 (fan-in/trunk/fan-out)    │                                │
  │  │ 회전각도 계산 (템플릿→새방향)        │                                │
  │  │ 3단계 구간별 벡터 생성:              │                                │
  │  │   Stage 1: FAN-IN  (시작→트렁크)    │ ← _build_zone_segment()       │
  │  │   Stage 2: TRUNK   (주 통로)        │ ← _build_trunk()              │
  │  │   Stage 3: FAN-OUT (트렁크→목적지)  │ ← _build_zone_segment()       │
  │  │ 장애물 회피 (구조기둥만, 최대5회)    │ ← _apply_obstacle_avoidance() │
  │  │ 끝점 보정 + 벡터 정리               │                                │
  │  └─────────────────────────────────────┘                                │
  ├──────────────────────────────────────────────────────────────────────────┤
  │  4. 검증 (PathValidatorV2.validate)                                      │
  │  ┌─────────────────────────────────────┐                                │
  │  │ ① 좌표 연속성 (0.20)               │                                │
  │  │ ② 길이 합리성 (0.20)               │                                │
  │  │ ③ 방향 패턴 (0.15)                 │                                │
  │  │ ④ 장애물 충돌 검사 (0.25)           │ ← COLUMN_STRUCTURE만 대상     │
  │  │ ⑤ Zone 준수 (0.20)                 │                                │
  │  │ 종합 quality ≥ 0.4 → PASS          │                                │
  │  └─────────────────────────────────────┘                                │
  │  5. 결과 저장 → AutoRoutingResults/*.json                               │
  └──────────────────────────────────────────────────────────────────────────┘

[V1 대비 핵심 개선 사항]
  1. Equipment BBox, Obstacle boundary, SpaceInfo를 원본 JSON에서 직접 로딩
  2. POC 위치를 장비 상대 좌표(0~1)로 정규화 → 장비 간 비교 가능
  3. 종단점(endPocs) 위치·타입을 경로 특징량에 포함
  4. 장애물 유형별 18개 공간관계 특징량 + 유사도 계산에 20% 반영
  5. 장애물 회피 경로 생성 (Slab Method 충돌 감지 + 우회 세그먼트 삽입)
  6. SpaceInfo 레벨(CSF/A/F/CR) 기반 경로 구간 제약

[주요 클래스/함수 목록]
  - ObstacleCategory(Enum)        : 장애물 6종 분류 열거형
  - ObstacleInfo                  : 장애물 정보 (BBox, 유형, 거리계산)
  - EquipmentInfo                 : 장비 정보 (BBox, POC, 상대좌표 변환)
  - SpaceLevelInfo                : 공간 레벨 정보 (이름, Z범위)
  - SpatialContext                : 공간 데이터 통합 관리 (장비/장애물/레벨)
    .load_from_json()             : 설계 JSON에서 공간 컨텍스트 로드
    .find_obstacles_in_path()     : 두 점 사이 경로의 장애물 충돌 감지 (Slab Method)
    .find_nearby_obstacles()      : 반경 내 장애물 검색
    .compute_obstacle_density()   : 가중 장애물 밀도 계산 (기둥=3배)
    ._segment_intersects_box()    : 선분-BBox 교차 판정 (Slab Method)
  - ObstacleRelationFeatures      : 장애물 유형별 18개 공간관계 특징량 데이터클래스
  - EnhancedPathFeatures          : 경로의 8차원 확장 특징량 데이터클래스
  - EnhancedFeatureExtractor      : 확장 특징량 추출기
    .extract()                    : 경로에서 8차원 특징량 추출
    ._classify_face()             : 점이 장비 BBox 어느 면에 가까운지 판별
  - ObstacleRelationExtractor     : 유형별 장애물 공간관계 추출기
    .extract()                    : 경로 좌표로부터 18개 특징량 추출
    ._compute_column_relations()  : 구조기둥 관계 (5개 특징)
    ._compute_lr_pattern()        : 기둥 좌/우/사이 패턴 (법선벡터 투영)
    ._compute_post_relations()    : 포스트 관계 (3개 특징)
    ._compute_grid_alignment()    : 그리드 규칙성 + 경로 정렬도
    ._compute_beam_relations()    : H-빔 관계 (3개 특징)
    ._compute_grating_relations() : 그레이팅 관계 (3개 특징)
  - SpatialSimilarity             : 8차원 확장 유사도 계산
    .compute()                    : 복합 유사도 점수 반환
    .compute_detail()             : 항목별 상세 유사도 반환
    ._obstacle_sim()              : 장애물관계 유사도 (기둥0.35/빔0.30/그레이팅0.20/포스트0.15)
  - ObstacleAwarePathBuilder      : 장애물 회피 경로 생성기
    .build_path()                 : 3단계(fan-in/trunk/fan-out) 경로 구성
    ._apply_obstacle_avoidance()  : 구조기둥 회피 (H세그먼트만, 최대5회)
    ._compute_detour()            : 우회 세그먼트 생성
  - PathValidatorV2               : 경로 검증기 (연속성/길이/패턴/충돌/Zone)
  - AutoRoutingDesignerV2         : 메인 오케스트레이터
    .load_data()                  : 원본 JSON + 그룹 결과 로드
    .find_matching_group()        : 8차원 유사도 기반 최적 템플릿 매칭
    .design_route()               : 단일 경로 자동설계
    .design_routes_batch()        : 배치 자동설계

[주요 변수 설명]
  - DesignConfigV2                : 설계 설정 (클리어런스, 유사도 가중치 등)
  - OBSTACLE_CLEARANCE            : 배관-장애물 최소 이격거리 (200mm)
  - TERMINAL_TYPES                : 종단점 타입 분류 매핑
  - _DDWORKS_TO_CATEGORY          : BIM ddworksType → ObstacleCategory 매핑
"""

import argparse
import glob
import json
import logging
import math
import os
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from AnalyzeRoutingAi_V2 import (
    _bbox,
    _clamp01,
    _compute_path_arrow,
    _cosine_similarity_0_1,
    _dist_xy,
    _dist_xyz,
    _extract_h_segments,
    _serialize_lengths,
    _serialize_vectors,
    _xy_spread,
    detect_zones,
    pattern_similarity,
)

# ─────────────────────────────────────────────────────────────
# 0. 설정
# ─────────────────────────────────────────────────────────────

LOGGER = logging.getLogger("AutoRoutingV2")

FITTING_LENGTH_THRESHOLD = 150.0

# 장애물 클리어런스 (mm) - 배관이 장애물로부터 유지해야 할 최소 거리
OBSTACLE_CLEARANCE = 200.0

# 종단점 타입 분류
TERMINAL_TYPES = {
    "BRANCH PIPE": "BRANCH",
    "DUCT": "DUCT",
    "LATERAL": "LATERAL",
    "EQUIPMENT": "EQUIPMENT",
    "disconnected pipe": "DISCONNECTED",
}


class ObstacleCategory(str, Enum):
    """장애물 유형 분류"""
    STRUCTURAL_COLUMN = "STRUCTURAL_COLUMN"   # 구조 기둥 (COLUMN_STRUCTURE)
    POST = "POST"                              # 포스트 (COLUMN_ARCHITECTURE)
    H_BEAM = "H_BEAM"                          # H-Beam (BEAM_ARCHITECTURE)
    STRUCTURAL_BEAM = "STRUCTURAL_BEAM"        # 구조 보 (BEAM_STRUCTURE)
    GRATING = "GRATING"                        # 그레이팅 (FLOOR_ARCHITECTURE)
    CEILING = "CEILING"                        # 천장 (CEILING_ARCHITECTURE)
    OTHER = "OTHER"


_DDWORKS_TO_CATEGORY = {
    "COLUMN_STRUCTURE": ObstacleCategory.STRUCTURAL_COLUMN,
    "COLUMN_ARCHITECTURE": ObstacleCategory.POST,
    "BEAM_ARCHITECTURE": ObstacleCategory.H_BEAM,
    "BEAM_STRUCTURE": ObstacleCategory.STRUCTURAL_BEAM,
    "FLOOR_ARCHITECTURE": ObstacleCategory.GRATING,
    "CEILING_ARCHITECTURE": ObstacleCategory.CEILING,
}


def _classify_obstacle(ddworks_type: str) -> ObstacleCategory:
    """ddworksType 문자열을 ObstacleCategory로 변환합니다."""
    return _DDWORKS_TO_CATEGORY.get(ddworks_type, ObstacleCategory.OTHER)


@dataclass
class DesignConfigV2:
    """V2 자동 경로 설계 설정"""
    # 입출력
    json_input_dir: str = "./data-v10"
    group_results_path: str = "./GroupPipeResults/group_pipe_results_20260405194814.json"
    output_dir: str = "./AutoRoutingResults"

    # 장애물
    obstacle_clearance: float = OBSTACLE_CLEARANCE
    obstacle_detour_margin: float = 300.0

    # 템플릿 매칭
    max_start_xy_distance: float = 8000.0
    fitting_length_threshold: float = FITTING_LENGTH_THRESHOLD

    # 경로 생성
    min_segment_length: float = 50.0
    direction_angle_tolerance: float = 5.0
    trunk_approach_tolerance: float = 300.0

    # 유사도 가중치 (합계 = 1.0)
    w_arrow: float = 0.12
    w_vector: float = 0.12
    w_range: float = 0.08
    w_length: float = 0.08
    w_equip_relative: float = 0.15   # 장비 상대 좌표 유사도
    w_terminal: float = 0.15         # 종단점 유사도
    w_obstacle_proximity: float = 0.20  # 장애물 유형별 공간관계 유사도
    w_level: float = 0.10            # SpaceInfo 레벨 유사도

    # 검증
    max_length_ratio: float = 2.5
    min_length_ratio: float = 0.2


# ─────────────────────────────────────────────────────────────
# 1. SpatialContext - 공간 컨텍스트 로딩
# ─────────────────────────────────────────────────────────────


@dataclass
class ObstacleInfo:
    """장애물 정보"""
    obstacle_id: str
    ddworks_type: str
    name: str
    bbox_min: Tuple[float, float, float]
    bbox_max: Tuple[float, float, float]
    category: ObstacleCategory = ObstacleCategory.OTHER

    @property
    def center(self) -> Tuple[float, float, float]:
        return (
            (self.bbox_min[0] + self.bbox_max[0]) / 2,
            (self.bbox_min[1] + self.bbox_max[1]) / 2,
            (self.bbox_min[2] + self.bbox_max[2]) / 2,
        )

    @property
    def size(self) -> Tuple[float, float, float]:
        return (
            self.bbox_max[0] - self.bbox_min[0],
            self.bbox_max[1] - self.bbox_min[1],
            self.bbox_max[2] - self.bbox_min[2],
        )

    def contains_point(self, pt: Tuple[float, float, float], margin: float = 0.0) -> bool:
        return (
            self.bbox_min[0] - margin <= pt[0] <= self.bbox_max[0] + margin
            and self.bbox_min[1] - margin <= pt[1] <= self.bbox_max[1] + margin
            and self.bbox_min[2] - margin <= pt[2] <= self.bbox_max[2] + margin
        )

    def distance_to_point(self, pt: Tuple[float, float, float]) -> float:
        dx = max(self.bbox_min[0] - pt[0], 0, pt[0] - self.bbox_max[0])
        dy = max(self.bbox_min[1] - pt[1], 0, pt[1] - self.bbox_max[1])
        dz = max(self.bbox_min[2] - pt[2], 0, pt[2] - self.bbox_max[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)


@dataclass
class EquipmentInfo:
    """장비 정보"""
    guid: str
    name: str
    process: str
    maker: str
    position: Tuple[float, float, float]
    bbox_min: Tuple[float, float, float]
    bbox_max: Tuple[float, float, float]
    poc_list: List[Dict[str, Any]]
    ends: List[Dict[str, Any]]

    @property
    def center(self) -> Tuple[float, float, float]:
        return (
            (self.bbox_min[0] + self.bbox_max[0]) / 2,
            (self.bbox_min[1] + self.bbox_max[1]) / 2,
            (self.bbox_min[2] + self.bbox_max[2]) / 2,
        )

    @property
    def size(self) -> Tuple[float, float, float]:
        return (
            self.bbox_max[0] - self.bbox_min[0],
            self.bbox_max[1] - self.bbox_min[1],
            self.bbox_max[2] - self.bbox_min[2],
        )

    def poc_relative_position(self, poc_pos: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """POC 위치를 장비 BBox 기준 정규화 좌표(0~1)로 변환합니다."""
        sx, sy, sz = self.size
        return (
            (poc_pos[0] - self.bbox_min[0]) / max(sx, 1.0),
            (poc_pos[1] - self.bbox_min[1]) / max(sy, 1.0),
            (poc_pos[2] - self.bbox_min[2]) / max(sz, 1.0),
        )


@dataclass
class SpaceLevelInfo:
    """공간 레벨 정보"""
    name: str
    z_min: float
    z_max: float
    bbox_min: Tuple[float, float, float]
    bbox_max: Tuple[float, float, float]


class SpatialContext:
    """원본 JSON에서 장비, 장애물, 공간 레벨 정보를 로드하고 공간 쿼리를 제공합니다."""

    def __init__(self):
        self.equipment: Optional[EquipmentInfo] = None
        self.obstacles: List[ObstacleInfo] = []
        self.columns: List[ObstacleInfo] = []       # 하위 호환 (COLUMN 전체)
        self.beams: List[ObstacleInfo] = []          # 하위 호환 (BEAM 전체)
        # 유형별 세분화 리스트
        self.structural_columns: List[ObstacleInfo] = []
        self.posts: List[ObstacleInfo] = []
        self.h_beams: List[ObstacleInfo] = []
        self.structural_beams: List[ObstacleInfo] = []
        self.gratings: List[ObstacleInfo] = []
        self.levels: List[SpaceLevelInfo] = []
        self.ends_map: Dict[str, Dict[str, Any]] = {}  # guid → end info

    def load_from_json(self, file_path: str) -> bool:
        """원본 설계 JSON에서 공간 컨텍스트를 로드합니다."""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            LOGGER.exception("JSON 로딩 실패: %s", file_path)
            return False

        # Equipment
        eq_raw = data.get("Equipment")
        if eq_raw:
            eq = eq_raw[0] if isinstance(eq_raw, list) else eq_raw
            bb = eq.get("boundaryBox", {})
            bb_min = bb.get("min", {})
            bb_max = bb.get("max", {})
            pos = eq.get("position", [0, 0, 0])

            self.equipment = EquipmentInfo(
                guid=eq.get("guid", ""),
                name=eq.get("name", ""),
                process=eq.get("process", ""),
                maker=eq.get("maker", ""),
                position=(float(pos[0]), float(pos[1]), float(pos[2])),
                bbox_min=(float(bb_min.get("x", 0)), float(bb_min.get("y", 0)), float(bb_min.get("z", 0))),
                bbox_max=(float(bb_max.get("x", 0)), float(bb_max.get("y", 0)), float(bb_max.get("z", 0))),
                poc_list=eq.get("pocList", []),
                ends=eq.get("ends", []),
            )

            # ends 맵 구축
            for end in self.equipment.ends:
                end_guid = end.get("guid", "")
                if end_guid:
                    self.ends_map[end_guid] = end
                for poc in end.get("pocList", []):
                    pid = poc.get("id", "")
                    if pid:
                        self.ends_map[pid] = end

        # Obstacles
        self.obstacles.clear()
        self.columns.clear()
        self.beams.clear()
        self.structural_columns.clear()
        self.posts.clear()
        self.h_beams.clear()
        self.structural_beams.clear()
        self.gratings.clear()
        for obs in data.get("Obstacles", []):
            bnd = obs.get("boundary", {})
            bnd_min = bnd.get("min", {})
            bnd_max = bnd.get("max", {})
            if not bnd_min or not bnd_max:
                continue

            ddworks_type = obs.get("ddworksType", "")
            info = ObstacleInfo(
                obstacle_id=obs.get("obstacleId", ""),
                ddworks_type=ddworks_type,
                name=obs.get("name", ""),
                bbox_min=(float(bnd_min.get("x", 0)), float(bnd_min.get("y", 0)), float(bnd_min.get("z", 0))),
                bbox_max=(float(bnd_max.get("x", 0)), float(bnd_max.get("y", 0)), float(bnd_max.get("z", 0))),
                category=_classify_obstacle(ddworks_type),
            )
            self.obstacles.append(info)
            # 하위 호환 분류
            if "COLUMN" in ddworks_type:
                self.columns.append(info)
            elif "BEAM" in ddworks_type:
                self.beams.append(info)
            # 유형별 세분화 분류
            if info.category == ObstacleCategory.STRUCTURAL_COLUMN:
                self.structural_columns.append(info)
            elif info.category == ObstacleCategory.POST:
                self.posts.append(info)
            elif info.category == ObstacleCategory.H_BEAM:
                self.h_beams.append(info)
            elif info.category == ObstacleCategory.STRUCTURAL_BEAM:
                self.structural_beams.append(info)
            elif info.category == ObstacleCategory.GRATING:
                self.gratings.append(info)

        # SpaceInfo
        self.levels.clear()
        for si in data.get("FileInfo", {}).get("SpaceInfo", []):
            bnd = si.get("boundary", {})
            bnd_min = bnd.get("min", {})
            bnd_max = bnd.get("max", {})
            self.levels.append(SpaceLevelInfo(
                name=si.get("levelName", ""),
                z_min=float(bnd_min.get("z", 0)),
                z_max=float(bnd_max.get("z", 0)),
                bbox_min=(float(bnd_min.get("x", 0)), float(bnd_min.get("y", 0)), float(bnd_min.get("z", 0))),
                bbox_max=(float(bnd_max.get("x", 0)), float(bnd_max.get("y", 0)), float(bnd_max.get("z", 0))),
            ))
        self.levels.sort(key=lambda lv: lv.z_min)

        LOGGER.info(
            "SpatialContext 로드: equipment=%s, obstacles=%d "
            "(struct_col=%d, post=%d, h_beam=%d, struct_beam=%d, grating=%d), levels=%d",
            self.equipment.name if self.equipment else "None",
            len(self.obstacles),
            len(self.structural_columns), len(self.posts),
            len(self.h_beams), len(self.structural_beams),
            len(self.gratings), len(self.levels),
        )
        return True

    def get_level_at_z(self, z: float) -> Optional[SpaceLevelInfo]:
        """Z 좌표에 해당하는 공간 레벨을 반환합니다."""
        for lv in self.levels:
            if lv.z_min <= z <= lv.z_max:
                return lv
        return None

    def get_level_name_at_z(self, z: float) -> str:
        lv = self.get_level_at_z(z)
        return lv.name if lv else "UNKNOWN"

    def find_obstacles_in_path(
        self,
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
        clearance: float,
    ) -> List[ObstacleInfo]:
        """두 점 사이 경로에 간섭하는 장애물 목록을 반환합니다."""
        collisions: List[ObstacleInfo] = []
        for obs in self.obstacles:
            if self._segment_intersects_box(p1, p2, obs, clearance):
                collisions.append(obs)
        return collisions

    def find_nearby_obstacles(
        self,
        point: Tuple[float, float, float],
        radius: float,
    ) -> List[Tuple[float, ObstacleInfo]]:
        """주어진 점 근처의 장애물을 (거리, 장애물) 튜플로 반환합니다."""
        results: List[Tuple[float, ObstacleInfo]] = []
        for obs in self.obstacles:
            d = obs.distance_to_point(point)
            if d <= radius:
                results.append((d, obs))
        results.sort(key=lambda x: x[0])
        return results

    def compute_obstacle_density(
        self,
        center: Tuple[float, float, float],
        radius: float,
    ) -> float:
        """주어진 영역 내 장애물 밀도를 0~1로 정규화하여 반환합니다."""
        nearby = self.find_nearby_obstacles(center, radius)
        # 기둥은 가중치 3, 빔은 가중치 1
        score = 0.0
        for d, obs in nearby:
            proximity = _clamp01(1.0 - d / radius)
            weight = 3.0 if "COLUMN" in obs.ddworks_type else 1.0
            score += proximity * weight
        return _clamp01(score / max(len(self.obstacles) * 0.05, 1.0))

    @staticmethod
    def _segment_intersects_box(
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
        obs: ObstacleInfo,
        margin: float,
    ) -> bool:
        """선분과 장애물 BBox(+margin) 교차 판정 (Slab method)"""
        bmin = (obs.bbox_min[0] - margin, obs.bbox_min[1] - margin, obs.bbox_min[2] - margin)
        bmax = (obs.bbox_max[0] + margin, obs.bbox_max[1] + margin, obs.bbox_max[2] + margin)

        d = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
        t_min_val = 0.0
        t_max_val = 1.0

        for i in range(3):
            if abs(d[i]) < 1e-9:
                if p1[i] < bmin[i] or p1[i] > bmax[i]:
                    return False
            else:
                inv_d = 1.0 / d[i]
                t1 = (bmin[i] - p1[i]) * inv_d
                t2 = (bmax[i] - p1[i]) * inv_d
                if t1 > t2:
                    t1, t2 = t2, t1
                t_min_val = max(t_min_val, t1)
                t_max_val = min(t_max_val, t2)
                if t_min_val > t_max_val:
                    return False
        return True


# ─────────────────────────────────────────────────────────────
# 2. EnhancedFeatureExtractor - 확장 특징량 추출
# ─────────────────────────────────────────────────────────────


@dataclass
class ObstacleRelationFeatures:
    """장애물 유형별 공간관계 특징량"""
    # 구조 기둥 (STRUCTURAL_COLUMN)
    col_count_nearby: int = 0
    col_min_distance: float = float("inf")
    col_avg_distance: float = float("inf")
    col_crossings: int = 0
    col_relative_pattern: str = ""       # L/R/BETWEEN 패턴 문자열

    # 포스트 (POST)
    post_count_nearby: int = 0
    post_density: float = 0.0            # 포스트 수 / BBox 면적 (정규화)
    post_grid_alignment: float = 0.0     # 포스트 그리드와 경로 정렬도 (0~1)

    # H-Beam (H_BEAM + STRUCTURAL_BEAM)
    beam_count_crossing: int = 0
    beam_min_clearance: float = float("inf")  # 최소 수직 간격
    beam_parallel_ratio: float = 0.0     # 경로와 평행한 빔 비율

    # 그레이팅 (GRATING)
    grating_coverage: float = 0.0        # 경로 하부 커버리지 (0~1)
    grating_count_below: int = 0
    grating_gap_count: int = 0           # 그레이팅 빈틈 수


@dataclass
class EnhancedPathFeatures:
    """경로의 확장 특징량"""
    # 기존 특징량
    path_arrow: str = ""
    path_range: Dict[str, float] = field(default_factory=dict)
    path_step_vectors: List[Dict[str, float]] = field(default_factory=list)
    path_step_lengths: List[float] = field(default_factory=list)
    path_total_length: float = 0.0

    # 장비 상대 특징량
    start_relative_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    end_relative_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    start_face: str = ""       # 장비의 어느 면에서 출발하는지 (N/S/E/W/T/B)
    end_direction: str = ""     # 종단 방향 (N/S/E/W/T/B)

    # 종단점 특징량
    terminal_type: str = ""     # BRANCH/DUCT/LATERAL/EQUIPMENT/DISCONNECTED
    terminal_distance: float = 0.0
    terminal_level: str = ""    # CSF/A/F/CR

    # 장애물 특징량 (기존 - 하위 호환)
    obstacle_count_nearby: int = 0
    obstacle_density: float = 0.0
    obstacle_min_distance: float = float("inf")
    column_crossings: int = 0

    # 장애물 유형별 공간관계 특징량
    obstacle_relations: ObstacleRelationFeatures = field(default_factory=ObstacleRelationFeatures)

    # 레벨 특징량
    start_level: str = ""
    end_level: str = ""
    levels_traversed: List[str] = field(default_factory=list)
    level_change_count: int = 0


class EnhancedFeatureExtractor:
    """장비·장애물·종단점 정보를 포함하는 확장 특징량을 추출합니다."""

    def __init__(self, spatial: SpatialContext, config: DesignConfigV2):
        self.spatial = spatial
        self.config = config

    def extract(
        self,
        start_pos: Tuple[float, float, float],
        end_pos: Tuple[float, float, float],
        step_vectors: List[Dict[str, float]],
        poc_info: Optional[Dict[str, Any]] = None,
    ) -> EnhancedPathFeatures:
        """경로에서 확장 특징량을 추출합니다."""
        feat = EnhancedPathFeatures()

        # 좌표 복원
        positions = _reconstruct_positions(start_pos, step_vectors)

        # 기존 특징량
        feat.path_arrow = _compute_path_arrow(positions, self.config.direction_angle_tolerance)
        bbox = _bbox(positions)
        feat.path_range = {
            "x": bbox["x_range"] if bbox else 0.0,
            "y": bbox["y_range"] if bbox else 0.0,
            "z": bbox["z_range"] if bbox else 0.0,
        }
        feat.path_step_vectors = step_vectors
        feat.path_step_lengths = [_vec_length(v) for v in step_vectors]
        feat.path_total_length = sum(feat.path_step_lengths)

        # 장비 상대 특징량
        eq = self.spatial.equipment
        if eq:
            feat.start_relative_pos = eq.poc_relative_position(start_pos)
            feat.end_relative_pos = eq.poc_relative_position(end_pos)
            feat.start_face = self._classify_face(start_pos, eq)
            feat.end_direction = self._classify_face(end_pos, eq)

        # 종단점 특징량
        if poc_info:
            end_pocs = poc_info.get("endPocs", [])
            if end_pocs:
                ep = end_pocs[0]
                raw_type = ep.get("endType", "")
                feat.terminal_type = TERMINAL_TYPES.get(raw_type, raw_type[:10] if raw_type else "UNKNOWN")
                ep_pos = ep.get("endPocPosition")
                if ep_pos:
                    feat.terminal_distance = _dist_xyz(start_pos, tuple(ep_pos))
            feat.terminal_level = self.spatial.get_level_name_at_z(end_pos[2])

        # 장애물 특징량
        path_center = (
            (start_pos[0] + end_pos[0]) / 2,
            (start_pos[1] + end_pos[1]) / 2,
            (start_pos[2] + end_pos[2]) / 2,
        )
        radius = _dist_xyz(start_pos, end_pos) * 0.8
        nearby = self.spatial.find_nearby_obstacles(path_center, max(radius, 2000.0))
        feat.obstacle_count_nearby = len(nearby)
        feat.obstacle_density = self.spatial.compute_obstacle_density(path_center, max(radius, 2000.0))
        feat.obstacle_min_distance = nearby[0][0] if nearby else float("inf")

        # 경로 세그먼트별 기둥 교차 수
        crossings = 0
        for i in range(len(positions) - 1):
            cols = self.spatial.find_obstacles_in_path(
                positions[i], positions[i + 1], self.config.obstacle_clearance
            )
            crossings += sum(1 for c in cols if "COLUMN" in c.ddworks_type)
        feat.column_crossings = crossings

        # 유형별 장애물관계 특징량
        relation_extractor = ObstacleRelationExtractor(self.spatial)
        feat.obstacle_relations = relation_extractor.extract(positions, max(radius, 2000.0))

        # 레벨 특징량
        feat.start_level = self.spatial.get_level_name_at_z(start_pos[2])
        feat.end_level = self.spatial.get_level_name_at_z(end_pos[2])
        seen_levels = []
        for pos in positions:
            lv = self.spatial.get_level_name_at_z(pos[2])
            if not seen_levels or seen_levels[-1] != lv:
                seen_levels.append(lv)
        feat.levels_traversed = seen_levels
        feat.level_change_count = max(0, len(seen_levels) - 1)

        return feat

    @staticmethod
    def _classify_face(
        point: Tuple[float, float, float],
        eq: EquipmentInfo,
    ) -> str:
        """점이 장비 BBox의 어느 면에 가장 가까운지 판별합니다."""
        center = eq.center
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        dz = point[2] - center[2]
        sx, sy, sz = eq.size

        # 각 축의 정규화된 거리
        nx = dx / max(sx, 1.0) if sx > 1.0 else 0.0
        ny = dy / max(sy, 1.0) if sy > 1.0 else 0.0
        nz = dz / max(sz, 1.0) if sz > 1.0 else 0.0

        abs_vals = {"E": nx, "W": -nx, "N": ny, "S": -ny, "T": nz, "B": -nz}
        return max(abs_vals, key=abs_vals.get)


# ─────────────────────────────────────────────────────────────
# 2-1. ObstacleRelationExtractor - 유형별 장애물 공간관계 추출
# ─────────────────────────────────────────────────────────────


class ObstacleRelationExtractor:
    """장애물 유형별 공간관계 특징량을 추출합니다."""

    def __init__(self, spatial: SpatialContext):
        self.spatial = spatial

    def extract(
        self,
        positions: List[Tuple[float, float, float]],
        radius: float,
    ) -> ObstacleRelationFeatures:
        """경로 좌표 리스트로부터 유형별 장애물관계 특징량을 추출합니다."""
        feat = ObstacleRelationFeatures()
        if len(positions) < 2:
            return feat

        self._compute_column_relations(positions, radius, feat)
        self._compute_post_relations(positions, radius, feat)
        self._compute_beam_relations(positions, radius, feat)
        self._compute_grating_relations(positions, radius, feat)
        return feat

    # ── 구조 기둥 관계 ──

    def _compute_column_relations(
        self,
        positions: List[Tuple[float, float, float]],
        radius: float,
        feat: ObstacleRelationFeatures,
    ) -> None:
        """구조 기둥과 경로의 공간관계를 계산합니다."""
        columns = self.spatial.structural_columns
        if not columns:
            return

        # 경로 중심 기준 근접 기둥
        path_center = self._path_center(positions)
        distances = []
        for col in columns:
            d = col.distance_to_point(path_center)
            if d <= radius:
                distances.append(d)

        feat.col_count_nearby = len(distances)
        if distances:
            feat.col_min_distance = min(distances)
            feat.col_avg_distance = sum(distances) / len(distances)

        # 세그먼트별 기둥 교차 수
        crossings = 0
        for i in range(len(positions) - 1):
            for col in columns:
                if self.spatial._segment_intersects_box(
                    positions[i], positions[i + 1], col, 200.0
                ):
                    crossings += 1
        feat.col_crossings = crossings

        # 기둥 좌/우 배치 패턴 (경로 진행 방향 기준)
        feat.col_relative_pattern = self._compute_lr_pattern(positions, columns, radius)

    def _compute_lr_pattern(
        self,
        positions: List[Tuple[float, float, float]],
        obstacles: List[ObstacleInfo],
        radius: float,
    ) -> str:
        """경로 진행 방향 기준으로 장애물의 좌/우/사이 패턴을 문자열로 반환합니다."""
        if len(positions) < 2 or not obstacles:
            return ""

        pattern_chars = []
        for i in range(len(positions) - 1):
            p1 = positions[i]
            p2 = positions[i + 1]
            seg_center = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2)
            # 진행 방향 벡터 (XY 평면)
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            seg_len = math.sqrt(dx * dx + dy * dy)
            if seg_len < 1.0:
                continue

            # 법선 벡터 (좌측 방향)
            nx = -dy / seg_len
            ny = dx / seg_len

            left = False
            right = False
            for obs in obstacles:
                d = obs.distance_to_point(seg_center)
                if d > radius:
                    continue
                # 장애물 중심이 경로 좌측인지 우측인지
                oc = obs.center
                cross = (oc[0] - seg_center[0]) * nx + (oc[1] - seg_center[1]) * ny
                if cross > 0:
                    left = True
                else:
                    right = True

            if left and right:
                pattern_chars.append("B")  # Between
            elif left:
                pattern_chars.append("L")
            elif right:
                pattern_chars.append("R")

        return "".join(pattern_chars)

    # ── 포스트 관계 ──

    def _compute_post_relations(
        self,
        positions: List[Tuple[float, float, float]],
        radius: float,
        feat: ObstacleRelationFeatures,
    ) -> None:
        """포스트(Access Floor post)와 경로의 공간관계를 계산합니다."""
        posts = self.spatial.posts
        if not posts:
            return

        # 경로 BBox 내 포스트 수
        bbox = _bbox(positions)
        if not bbox:
            return

        nearby_posts = []
        path_center = self._path_center(positions)
        for post in posts:
            d = post.distance_to_point(path_center)
            if d <= radius:
                nearby_posts.append(post)

        feat.post_count_nearby = len(nearby_posts)

        # 포스트 밀도 (경로 BBox XY 면적 대비)
        area = max(bbox["x_range"] * bbox["y_range"], 1.0)
        feat.post_density = _clamp01(len(nearby_posts) / max(area / 1e6, 1.0))

        # 포스트 그리드 정렬도: 포스트 간 간격의 규칙성 분석
        feat.post_grid_alignment = self._compute_grid_alignment(nearby_posts, positions)

    def _compute_grid_alignment(
        self,
        posts: List[ObstacleInfo],
        positions: List[Tuple[float, float, float]],
    ) -> float:
        """포스트 배치의 규칙성과 경로의 정렬도를 계산합니다 (0~1)."""
        if len(posts) < 3:
            return 0.0

        # 포스트 중심 좌표의 X, Y 간격 분석
        centers_x = sorted(set(round(p.center[0], -1) for p in posts))  # 10mm 단위 반올림
        centers_y = sorted(set(round(p.center[1], -1) for p in posts))

        # 간격의 표준편차 / 평균 → 규칙성 (CV가 작을수록 규칙적)
        def regularity(values):
            if len(values) < 2:
                return 0.0
            gaps = [values[i + 1] - values[i] for i in range(len(values) - 1)]
            if not gaps:
                return 0.0
            avg = sum(gaps) / len(gaps)
            if avg < 1.0:
                return 0.0
            variance = sum((g - avg) ** 2 for g in gaps) / len(gaps)
            cv = math.sqrt(variance) / avg
            return _clamp01(1.0 - cv)

        grid_regularity = (regularity(centers_x) + regularity(centers_y)) / 2

        # 경로가 그리드 축과 얼마나 정렬되는지
        alignment_score = 0.0
        total_length = 0.0
        for i in range(len(positions) - 1):
            dx = abs(positions[i + 1][0] - positions[i][0])
            dy = abs(positions[i + 1][1] - positions[i][1])
            seg_len = math.sqrt(dx * dx + dy * dy)
            if seg_len < 1.0:
                continue
            # X축 또는 Y축에 정렬된 정도 (직교 비율)
            axis_ratio = max(dx, dy) / seg_len
            alignment_score += axis_ratio * seg_len
            total_length += seg_len

        path_alignment = alignment_score / max(total_length, 1.0)
        return _clamp01(grid_regularity * 0.5 + path_alignment * 0.5)

    # ── H-Beam 관계 ──

    def _compute_beam_relations(
        self,
        positions: List[Tuple[float, float, float]],
        radius: float,
        feat: ObstacleRelationFeatures,
    ) -> None:
        """H-Beam 및 구조 보와 경로의 공간관계를 계산합니다."""
        all_beams = self.spatial.h_beams + self.spatial.structural_beams
        if not all_beams:
            return

        path_center = self._path_center(positions)
        nearby_beams = [b for b in all_beams if b.distance_to_point(path_center) <= radius]
        if not nearby_beams:
            return

        # 세그먼트별 빔 교차 수
        crossing_count = 0
        min_clearance = float("inf")
        parallel_count = 0
        total_nearby = len(nearby_beams)

        for i in range(len(positions) - 1):
            p1 = positions[i]
            p2 = positions[i + 1]
            seg_dx = p2[0] - p1[0]
            seg_dy = p2[1] - p1[1]
            seg_len_xy = math.sqrt(seg_dx * seg_dx + seg_dy * seg_dy)

            for beam in nearby_beams:
                # 교차 판정
                if self.spatial._segment_intersects_box(p1, p2, beam, 100.0):
                    crossing_count += 1

                # 수직 간격 (clearance): 빔 하단과 경로 Z의 차이
                seg_z = (p1[2] + p2[2]) / 2
                beam_bottom_z = beam.bbox_min[2]
                clearance = abs(beam_bottom_z - seg_z)
                min_clearance = min(min_clearance, clearance)

        # 빔과 경로의 평행 비율
        for beam in nearby_beams:
            beam_dx = beam.bbox_max[0] - beam.bbox_min[0]
            beam_dy = beam.bbox_max[1] - beam.bbox_min[1]
            beam_len = math.sqrt(beam_dx * beam_dx + beam_dy * beam_dy)
            if beam_len < 1.0:
                continue

            # 경로 주방향과 빔 방향의 코사인 유사도
            path_dx = positions[-1][0] - positions[0][0]
            path_dy = positions[-1][1] - positions[0][1]
            path_len = math.sqrt(path_dx * path_dx + path_dy * path_dy)
            if path_len < 1.0:
                continue

            cos_angle = abs(
                (beam_dx * path_dx + beam_dy * path_dy) / (beam_len * path_len)
            )
            if cos_angle > 0.7:  # ~45도 이내면 평행으로 간주
                parallel_count += 1

        feat.beam_count_crossing = crossing_count
        feat.beam_min_clearance = min_clearance if min_clearance < float("inf") else 0.0
        feat.beam_parallel_ratio = parallel_count / max(total_nearby, 1)

    # ── 그레이팅 관계 ──

    def _compute_grating_relations(
        self,
        positions: List[Tuple[float, float, float]],
        radius: float,
        feat: ObstacleRelationFeatures,
    ) -> None:
        """그레이팅과 경로의 공간관계를 계산합니다."""
        gratings = self.spatial.gratings
        if not gratings:
            return

        path_center = self._path_center(positions)
        nearby_gratings = [g for g in gratings if g.distance_to_point(path_center) <= radius]
        feat.grating_count_below = len(nearby_gratings)
        if not nearby_gratings:
            return

        # 경로의 각 수평 세그먼트에 대해 하부 그레이팅 커버리지 계산
        covered_length = 0.0
        total_h_length = 0.0
        gap_transitions = 0
        prev_covered = False

        for i in range(len(positions) - 1):
            p1 = positions[i]
            p2 = positions[i + 1]
            # 수평 세그먼트만 (Z 변화가 작은)
            if abs(p2[2] - p1[2]) > 100.0:
                continue

            seg_len = _dist_xy(p1, p2)
            if seg_len < 1.0:
                continue
            total_h_length += seg_len

            # 세그먼트 중심점 아래에 그레이팅이 있는지
            seg_mid = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2)
            has_grating_below = False
            for g in nearby_gratings:
                # 그레이팅의 XY 범위에 세그먼트 중심이 포함되고,
                # 그레이팅이 경로보다 아래에 있는지
                if (g.bbox_min[0] <= seg_mid[0] <= g.bbox_max[0]
                        and g.bbox_min[1] <= seg_mid[1] <= g.bbox_max[1]
                        and g.bbox_max[2] <= seg_mid[2] + 500.0):  # 그레이팅 상단이 경로 아래
                    has_grating_below = True
                    break

            if has_grating_below:
                covered_length += seg_len
            # 빈틈(개구부) 전환 카운트
            if prev_covered and not has_grating_below:
                gap_transitions += 1
            prev_covered = has_grating_below

        feat.grating_coverage = _clamp01(covered_length / max(total_h_length, 1.0))
        feat.grating_gap_count = gap_transitions

    # ── 공통 유틸리티 ──

    @staticmethod
    def _path_center(positions: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """경로의 중심점을 반환합니다."""
        n = len(positions)
        return (
            sum(p[0] for p in positions) / n,
            sum(p[1] for p in positions) / n,
            sum(p[2] for p in positions) / n,
        )


# ─────────────────────────────────────────────────────────────
# 3. SpatialSimilarity - 확장 유사도 계산
# ─────────────────────────────────────────────────────────────


class SpatialSimilarity:
    """기존 경로 유사도 + 공간 컨텍스트 유사도를 통합합니다."""

    def __init__(self, config: DesignConfigV2):
        self.config = config

    def compute(
        self,
        feat1: EnhancedPathFeatures,
        feat2: EnhancedPathFeatures,
    ) -> float:
        """확장 복합 유사도를 계산합니다."""
        c = self.config

        # 기존 유사도 (AnalyzeRoutingAi_V2에서 가져온 로직)
        s_arrow = pattern_similarity(feat1.path_arrow, feat2.path_arrow)
        s_vector = self._vector_sim(feat1.path_step_vectors, feat2.path_step_vectors)
        s_range = self._range_sim(feat1.path_range, feat2.path_range)
        s_length = self._length_sim(feat1.path_total_length, feat2.path_total_length)

        # 장비 상대 좌표 유사도
        s_equip = self._equip_relative_sim(feat1, feat2)

        # 종단점 유사도
        s_terminal = self._terminal_sim(feat1, feat2)

        # 장애물 근접도 유사도
        s_obstacle = self._obstacle_sim(feat1, feat2)

        # 레벨 유사도
        s_level = self._level_sim(feat1, feat2)

        score = (
            s_arrow * c.w_arrow
            + s_vector * c.w_vector
            + s_range * c.w_range
            + s_length * c.w_length
            + s_equip * c.w_equip_relative
            + s_terminal * c.w_terminal
            + s_obstacle * c.w_obstacle_proximity
            + s_level * c.w_level
        )

        return round(_clamp01(score), 6)

    def compute_detail(
        self,
        feat1: EnhancedPathFeatures,
        feat2: EnhancedPathFeatures,
    ) -> Dict[str, float]:
        """각 유사도 항목을 개별적으로 반환합니다."""
        r1 = feat1.obstacle_relations
        r2 = feat2.obstacle_relations
        return {
            "arrow": round(pattern_similarity(feat1.path_arrow, feat2.path_arrow), 4),
            "vector": round(self._vector_sim(feat1.path_step_vectors, feat2.path_step_vectors), 4),
            "range": round(self._range_sim(feat1.path_range, feat2.path_range), 4),
            "length": round(self._length_sim(feat1.path_total_length, feat2.path_total_length), 4),
            "equip_relative": round(self._equip_relative_sim(feat1, feat2), 4),
            "terminal": round(self._terminal_sim(feat1, feat2), 4),
            "obstacle": round(self._obstacle_sim(feat1, feat2), 4),
            "obstacle_column": round(self._column_relation_sim(r1, r2), 4),
            "obstacle_post": round(self._post_relation_sim(r1, r2), 4),
            "obstacle_beam": round(self._beam_relation_sim(r1, r2), 4),
            "obstacle_grating": round(self._grating_relation_sim(r1, r2), 4),
            "level": round(self._level_sim(feat1, feat2), 4),
        }

    @staticmethod
    def _vector_sim(v1: List[Dict], v2: List[Dict]) -> float:
        if not v1 and not v2:
            return 1.0
        if not v1 or not v2:
            return 0.0
        min_len = min(len(v1), len(v2))
        max_len = max(len(v1), len(v2))
        score = sum(_cosine_similarity_0_1(v1[i], v2[i]) for i in range(min_len)) / min_len
        return _clamp01(score * min_len / max_len)

    @staticmethod
    def _range_sim(r1: Dict, r2: Dict) -> float:
        scores = []
        for axis in ("x", "y", "z"):
            a = float(r1.get(axis, 0))
            b = float(r2.get(axis, 0))
            base = max(a, b, 1.0)
            scores.append(_clamp01(1.0 - abs(a - b) / base))
        return sum(scores) / 3

    @staticmethod
    def _length_sim(l1: float, l2: float) -> float:
        if l1 == 0 and l2 == 0:
            return 1.0
        return _clamp01(1.0 - abs(l1 - l2) / max(l1, l2, 1.0))

    @staticmethod
    def _equip_relative_sim(f1: EnhancedPathFeatures, f2: EnhancedPathFeatures) -> float:
        """장비 상대 좌표의 유사도를 계산합니다."""
        # 시작점 상대 좌표 유사도
        s1 = f1.start_relative_pos
        s2 = f2.start_relative_pos
        start_dist = math.sqrt(sum((s1[i] - s2[i]) ** 2 for i in range(3)))
        start_sim = _clamp01(1.0 - start_dist)

        # 종점 상대 좌표 유사도
        e1 = f1.end_relative_pos
        e2 = f2.end_relative_pos
        end_dist = math.sqrt(sum((e1[i] - e2[i]) ** 2 for i in range(3)))
        end_sim = _clamp01(1.0 - end_dist)

        # 출발 면 일치 보너스
        face_bonus = 0.2 if f1.start_face == f2.start_face else 0.0

        return _clamp01((start_sim * 0.4 + end_sim * 0.4 + face_bonus) / 1.0)

    @staticmethod
    def _terminal_sim(f1: EnhancedPathFeatures, f2: EnhancedPathFeatures) -> float:
        """종단점 유사도를 계산합니다."""
        # 종단점 타입 일치
        type_score = 1.0 if f1.terminal_type == f2.terminal_type else 0.3

        # 종단 거리 유사도
        d1 = f1.terminal_distance
        d2 = f2.terminal_distance
        if d1 > 0 and d2 > 0:
            dist_score = _clamp01(1.0 - abs(d1 - d2) / max(d1, d2, 1.0))
        elif d1 == 0 and d2 == 0:
            dist_score = 1.0
        else:
            dist_score = 0.3

        # 종단 레벨 일치
        level_score = 1.0 if f1.terminal_level == f2.terminal_level else 0.3

        return type_score * 0.4 + dist_score * 0.3 + level_score * 0.3

    @staticmethod
    def _obstacle_sim(f1: EnhancedPathFeatures, f2: EnhancedPathFeatures) -> float:
        """장애물 유형별 공간관계 유사도를 계산합니다."""
        r1 = f1.obstacle_relations
        r2 = f2.obstacle_relations

        s_col = SpatialSimilarity._column_relation_sim(r1, r2)
        s_post = SpatialSimilarity._post_relation_sim(r1, r2)
        s_beam = SpatialSimilarity._beam_relation_sim(r1, r2)
        s_grating = SpatialSimilarity._grating_relation_sim(r1, r2)

        return s_col * 0.35 + s_post * 0.15 + s_beam * 0.30 + s_grating * 0.20

    @staticmethod
    def _column_relation_sim(r1: ObstacleRelationFeatures, r2: ObstacleRelationFeatures) -> float:
        """구조 기둥 관계 유사도"""
        # 근접 기둥 수 유사도
        c1, c2 = r1.col_count_nearby, r2.col_count_nearby
        count_sim = _clamp01(1.0 - abs(c1 - c2) / max(c1, c2, 1))

        # 최소 거리 유사도
        d1, d2 = r1.col_min_distance, r2.col_min_distance
        if d1 < float("inf") and d2 < float("inf"):
            dist_sim = _clamp01(1.0 - abs(d1 - d2) / max(d1, d2, 1.0))
        elif d1 == float("inf") and d2 == float("inf"):
            dist_sim = 1.0
        else:
            dist_sim = 0.3

        # 기둥 교차 수 유사도
        x1, x2 = r1.col_crossings, r2.col_crossings
        cross_sim = _clamp01(1.0 - abs(x1 - x2) / max(x1, x2, 1))

        # LR 패턴 유사도
        pattern_sim = pattern_similarity(r1.col_relative_pattern, r2.col_relative_pattern)

        return count_sim * 0.2 + dist_sim * 0.2 + cross_sim * 0.3 + pattern_sim * 0.3

    @staticmethod
    def _post_relation_sim(r1: ObstacleRelationFeatures, r2: ObstacleRelationFeatures) -> float:
        """포스트 관계 유사도"""
        # 밀도 유사도
        density_sim = _clamp01(1.0 - abs(r1.post_density - r2.post_density))

        # 그리드 정렬도 유사도
        align_sim = _clamp01(1.0 - abs(r1.post_grid_alignment - r2.post_grid_alignment))

        # 근접 수 유사도
        c1, c2 = r1.post_count_nearby, r2.post_count_nearby
        count_sim = _clamp01(1.0 - abs(c1 - c2) / max(c1, c2, 1))

        return density_sim * 0.3 + align_sim * 0.4 + count_sim * 0.3

    @staticmethod
    def _beam_relation_sim(r1: ObstacleRelationFeatures, r2: ObstacleRelationFeatures) -> float:
        """H-Beam 관계 유사도"""
        # 교차 수 유사도
        x1, x2 = r1.beam_count_crossing, r2.beam_count_crossing
        cross_sim = _clamp01(1.0 - abs(x1 - x2) / max(x1, x2, 1))

        # 최소 clearance 유사도
        c1, c2 = r1.beam_min_clearance, r2.beam_min_clearance
        if c1 < float("inf") and c2 < float("inf"):
            clearance_sim = _clamp01(1.0 - abs(c1 - c2) / max(c1, c2, 1.0))
        elif c1 == float("inf") and c2 == float("inf"):
            clearance_sim = 1.0
        else:
            clearance_sim = 0.3

        # 평행 비율 유사도
        parallel_sim = _clamp01(1.0 - abs(r1.beam_parallel_ratio - r2.beam_parallel_ratio))

        return cross_sim * 0.4 + clearance_sim * 0.3 + parallel_sim * 0.3

    @staticmethod
    def _grating_relation_sim(r1: ObstacleRelationFeatures, r2: ObstacleRelationFeatures) -> float:
        """그레이팅 관계 유사도"""
        # 커버리지 유사도
        coverage_sim = _clamp01(1.0 - abs(r1.grating_coverage - r2.grating_coverage))

        # 하부 수 유사도
        c1, c2 = r1.grating_count_below, r2.grating_count_below
        count_sim = _clamp01(1.0 - abs(c1 - c2) / max(c1, c2, 1))

        # 개구부 수 유사도
        g1, g2 = r1.grating_gap_count, r2.grating_gap_count
        gap_sim = _clamp01(1.0 - abs(g1 - g2) / max(g1, g2, 1))

        return coverage_sim * 0.4 + count_sim * 0.3 + gap_sim * 0.3

    @staticmethod
    def _level_sim(f1: EnhancedPathFeatures, f2: EnhancedPathFeatures) -> float:
        """레벨 유사도를 계산합니다."""
        # 시작/끝 레벨 일치
        start_match = 1.0 if f1.start_level == f2.start_level else 0.0
        end_match = 1.0 if f1.end_level == f2.end_level else 0.0

        # 레벨 변경 횟수 유사도
        c1 = f1.level_change_count
        c2 = f2.level_change_count
        change_sim = _clamp01(1.0 - abs(c1 - c2) / max(c1, c2, 1))

        # 경유 레벨 집합 Jaccard
        set1 = set(f1.levels_traversed)
        set2 = set(f2.levels_traversed)
        if set1 or set2:
            jaccard = len(set1 & set2) / len(set1 | set2)
        else:
            jaccard = 1.0

        return start_match * 0.25 + end_match * 0.25 + change_sim * 0.25 + jaccard * 0.25


# ─────────────────────────────────────────────────────────────
# 4. ObstacleAwarePathBuilder - 장애물 회피 경로 생성
# ─────────────────────────────────────────────────────────────


def _reconstruct_positions(
    start_pos: Sequence[float],
    vectors: List[Dict[str, float]],
) -> List[Tuple[float, float, float]]:
    positions = [tuple(float(c) for c in start_pos)]
    x, y, z = float(start_pos[0]), float(start_pos[1]), float(start_pos[2])
    for v in vectors:
        x += v["x"]; y += v["y"]; z += v["z"]
        positions.append((round(x, 1), round(y, 1), round(z, 1)))
    return positions


def _vec_length(v: Dict[str, float]) -> float:
    return math.sqrt(v["x"] ** 2 + v["y"] ** 2 + v["z"] ** 2)


def _vec_scale(v: Dict[str, float], s: float) -> Dict[str, float]:
    return {"x": round(v["x"] * s, 1), "y": round(v["y"] * s, 1), "z": round(v["z"] * s, 1)}


def _vec_add(a: Dict[str, float], b: Dict[str, float]) -> Dict[str, float]:
    return {"x": a["x"] + b["x"], "y": a["y"] + b["y"], "z": a["z"] + b["z"]}


def _segment_type(v: Dict[str, float], tol_deg: float = 5.0) -> str:
    total = _vec_length(v)
    if total < 1e-9:
        return "H"
    v_angle = math.degrees(math.asin(min(abs(v["z"]) / total, 1.0)))
    if v_angle > (90.0 - tol_deg):
        return "R"
    if v_angle < tol_deg:
        return "H"
    return "D"


def _is_fitting(v: Dict[str, float], threshold: float) -> bool:
    return _segment_type(v) == "R" and _vec_length(v) <= threshold


def _rotate_xy(vx: float, vy: float, angle_rad: float) -> Tuple[float, float]:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (round(vx * cos_a - vy * sin_a, 1), round(vx * sin_a + vy * cos_a, 1))


def _snap_to_orthogonal(vx: float, vy: float) -> Tuple[float, float]:
    return (vx, 0.0) if abs(vx) >= abs(vy) else (0.0, vy)


class ObstacleAwarePathBuilder:
    """장애물을 회피하면서 Zone 기반으로 경로를 구성합니다."""

    def __init__(self, spatial: SpatialContext, config: DesignConfigV2):
        self.spatial = spatial
        self.config = config

    def build_path(
        self,
        new_start: Tuple[float, float, float],
        destination: Tuple[float, float, float],
        group: Dict[str, Any],
        template_path: Dict[str, Any],
    ) -> Dict[str, Any]:
        """장애물 회피가 적용된 3단계 경로를 구성합니다."""
        trunk_zone = group["zones"].get("trunk")
        if not trunk_zone:
            return self._build_direct_path(new_start, destination, template_path)

        trunk_z = trunk_zone["mean_z"]
        z_band = self.config.trunk_approach_tolerance

        # 템플릿 zone 분류
        zone_segs = self._identify_zone_segments(template_path, trunk_z, z_band)

        # 회전 각도 계산
        tmpl_disp = template_path.get("displacement_vector", {"x": 0, "y": 0, "z": 0})
        tmpl_angle = math.atan2(tmpl_disp["y"], tmpl_disp["x"])
        new_angle = math.atan2(destination[1] - new_start[1], destination[0] - new_start[0])
        rotation = new_angle - tmpl_angle

        # trunk 진입/이탈점 계산
        trunk_entry = self._compute_trunk_point(new_start, trunk_z, zone_segs["fan_in"], rotation)
        trunk_exit = self._compute_trunk_point_exit(destination, trunk_z, zone_segs["fan_out"], rotation)

        # 3단계 구성
        fan_in_vecs = self._build_zone_segment(new_start, trunk_entry, zone_segs["fan_in"], rotation)
        trunk_vecs = self._build_trunk(trunk_entry, trunk_exit, zone_segs["trunk"], rotation)
        fan_out_vecs = self._build_zone_segment(trunk_exit, destination, zone_segs["fan_out"], rotation)

        all_vectors = fan_in_vecs + trunk_vecs + fan_out_vecs

        # 장애물 회피 적용
        all_vectors = self._apply_obstacle_avoidance(new_start, all_vectors)

        # 끝점 보정
        all_vectors = self._correct_endpoint(new_start, destination, all_vectors)

        # 영벡터 및 짧은 세그먼트 정리
        all_vectors = self._cleanup_vectors(all_vectors)

        return self._assemble_result(new_start, destination, all_vectors)

    def _identify_zone_segments(
        self,
        template: Dict[str, Any],
        trunk_z: float,
        z_band: float,
    ) -> Dict[str, List[Dict[str, float]]]:
        vectors = template.get("path_step_vectors", [])
        positions = _reconstruct_positions(template["start_pos"], vectors)

        trunk_indices = [
            i for i, pos in enumerate(positions) if abs(pos[2] - trunk_z) <= z_band
        ]

        if not trunk_indices:
            return {"fan_in": vectors, "trunk": [], "fan_out": []}

        first = trunk_indices[0]
        last = trunk_indices[-1]
        return {
            "fan_in": vectors[:max(first, 0)],
            "trunk": vectors[max(first, 0):last],
            "fan_out": vectors[last:],
        }

    def _compute_trunk_point(
        self,
        start: Tuple[float, float, float],
        trunk_z: float,
        fan_vecs: List[Dict[str, float]],
        rotation: float,
    ) -> Tuple[float, float, float]:
        if not fan_vecs:
            return (start[0], start[1], trunk_z)
        total_dx = sum(v["x"] for v in fan_vecs)
        total_dy = sum(v["y"] for v in fan_vecs)
        rx, ry = _rotate_xy(total_dx, total_dy, rotation)
        rx, ry = _snap_to_orthogonal(rx, ry)
        return (round(start[0] + rx, 1), round(start[1] + ry, 1), trunk_z)

    def _compute_trunk_point_exit(
        self,
        dest: Tuple[float, float, float],
        trunk_z: float,
        fan_vecs: List[Dict[str, float]],
        rotation: float,
    ) -> Tuple[float, float, float]:
        if not fan_vecs:
            return (dest[0], dest[1], trunk_z)
        total_dx = sum(v["x"] for v in fan_vecs)
        total_dy = sum(v["y"] for v in fan_vecs)
        rx, ry = _rotate_xy(total_dx, total_dy, rotation)
        rx, ry = _snap_to_orthogonal(rx, ry)
        return (round(dest[0] - rx, 1), round(dest[1] - ry, 1), trunk_z)

    def _build_zone_segment(
        self,
        from_pos: Tuple[float, float, float],
        to_pos: Tuple[float, float, float],
        template_vecs: List[Dict[str, float]],
        rotation: float,
    ) -> List[Dict[str, float]]:
        """fan_in 또는 fan_out 구간을 템플릿 기반으로 생성합니다."""
        if not template_vecs:
            return self._build_simple_connection(from_pos, to_pos)

        dz_needed = to_pos[2] - from_pos[2]
        dx_needed = to_pos[0] - from_pos[0]
        dy_needed = to_pos[1] - from_pos[1]

        tmpl_r_dz = sum(
            v["z"] for v in template_vecs
            if _segment_type(v) == "R" and not _is_fitting(v, self.config.fitting_length_threshold)
        )
        tmpl_h_dx = sum(v["x"] for v in template_vecs if _segment_type(v) == "H")
        tmpl_h_dy = sum(v["y"] for v in template_vecs if _segment_type(v) == "H")
        tmpl_h_dist = math.sqrt(tmpl_h_dx ** 2 + tmpl_h_dy ** 2)

        scale_z = dz_needed / tmpl_r_dz if abs(tmpl_r_dz) > 1e-6 else 1.0
        needed_h = math.sqrt(dx_needed ** 2 + dy_needed ** 2)
        scale_xy = needed_h / tmpl_h_dist if tmpl_h_dist > 1e-6 else 1.0

        result: List[Dict[str, float]] = []
        for v in template_vecs:
            st = _segment_type(v)
            if _is_fitting(v, self.config.fitting_length_threshold):
                fv = dict(v)
                if (dz_needed >= 0 and v["z"] < 0) or (dz_needed < 0 and v["z"] > 0):
                    fv["z"] = -fv["z"]
                result.append(fv)
            elif st == "R":
                result.append({"x": 0.0, "y": 0.0, "z": round(v["z"] * scale_z, 1)})
            elif st == "H":
                rx, ry = _rotate_xy(v["x"], v["y"], rotation)
                scaled = _vec_scale({"x": rx, "y": ry, "z": 0.0}, scale_xy)
                sx, sy = _snap_to_orthogonal(scaled["x"], scaled["y"])
                result.append({"x": sx, "y": sy, "z": 0.0})
            else:
                hx, hy = _rotate_xy(v["x"], v["y"], rotation)
                hx, hy = _snap_to_orthogonal(round(hx * scale_xy, 1), round(hy * scale_xy, 1))
                result.append({"x": hx, "y": hy, "z": round(v["z"] * scale_z, 1)})
        return result

    def _build_trunk(
        self,
        entry: Tuple[float, float, float],
        exit_pt: Tuple[float, float, float],
        template_vecs: List[Dict[str, float]],
        rotation: float,
    ) -> List[Dict[str, float]]:
        dx = exit_pt[0] - entry[0]
        dy = exit_pt[1] - entry[1]

        if not template_vecs:
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                return []
            sx, sy = _snap_to_orthogonal(dx, dy)
            return [{"x": round(sx, 1), "y": round(sy, 1), "z": 0.0}]

        fitting_dx = fitting_dy = 0.0
        h_indices: List[int] = []
        result: List[Optional[Dict[str, float]]] = []

        for i, v in enumerate(template_vecs):
            if _is_fitting(v, self.config.fitting_length_threshold):
                result.append(dict(v))
                fitting_dx += v["x"]; fitting_dy += v["y"]
            elif _segment_type(v) in ("H", "D"):
                h_indices.append(i)
                result.append(None)
            else:
                result.append(dict(v))

        rem_dx = dx - fitting_dx
        rem_dy = dy - fitting_dy

        if h_indices:
            tmpl_h_lens = [math.sqrt(template_vecs[idx]["x"] ** 2 + template_vecs[idx]["y"] ** 2) for idx in h_indices]
            total_h = sum(tmpl_h_lens) or 1.0
            for j, idx in enumerate(h_indices):
                ratio = tmpl_h_lens[j] / total_h
                hx, hy = _snap_to_orthogonal(round(rem_dx * ratio, 1), round(rem_dy * ratio, 1))
                result[idx] = {"x": hx, "y": hy, "z": 0.0}
        elif abs(rem_dx) > 1e-6 or abs(rem_dy) > 1e-6:
            sx, sy = _snap_to_orthogonal(round(rem_dx, 1), round(rem_dy, 1))
            result.append({"x": sx, "y": sy, "z": 0.0})

        return [v for v in result if v is not None]

    def _apply_obstacle_avoidance(
        self,
        start_pos: Tuple[float, float, float],
        vectors: List[Dict[str, float]],
    ) -> List[Dict[str, float]]:
        """
        H(수평) 세그먼트에서 기둥 충돌을 감지하고 우회 세그먼트를 삽입합니다.
        R(수직) 세그먼트는 우회하지 않습니다 (기둥은 수직이므로 수직이동은 통과).
        최대 5개 세그먼트만 우회하여 경로 과도 확장을 방지합니다.
        """
        positions = _reconstruct_positions(start_pos, vectors)
        result: List[Dict[str, float]] = []
        clearance = self.config.obstacle_clearance
        margin = self.config.obstacle_detour_margin
        detour_count = 0
        max_detours = 5

        for i in range(len(vectors)):
            p1 = positions[i]
            p2 = positions[i + 1]

            # R(수직) 세그먼트와 피팅은 우회 대상이 아님
            if _segment_type(vectors[i]) == "R" or _is_fitting(vectors[i], self.config.fitting_length_threshold):
                result.append(vectors[i])
                continue

            if detour_count >= max_detours:
                result.append(vectors[i])
                continue

            # COLUMN_STRUCTURE만 우회 대상 (COLUMN_ARCHITECTURE, BEAM, FLOOR 무시)
            collisions = self.spatial.find_obstacles_in_path(p1, p2, clearance)
            col_obstacles = [c for c in collisions if c.ddworks_type == "COLUMN_STRUCTURE"]

            if not col_obstacles:
                result.append(vectors[i])
                continue

            obs = min(col_obstacles, key=lambda o: o.distance_to_point(p1))
            detour = self._compute_detour(p1, p2, obs, margin)
            result.extend(detour)
            detour_count += 1

        return result

    def _compute_detour(
        self,
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
        obs: ObstacleInfo,
        margin: float,
    ) -> List[Dict[str, float]]:
        """장애물을 우회하는 세그먼트 시퀀스를 생성합니다."""
        center = obs.center
        size_x = obs.size[0] + margin * 2
        size_y = obs.size[1] + margin * 2

        # 이동 방향 분석
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]

        # 주 이동 방향이 X축이면 Y방향으로 우회, Y축이면 X방향으로 우회
        if abs(dx) >= abs(dy):
            # Y방향 우회: p1이 기둥 중심 대비 어느 쪽인지에 따라 방향 결정
            detour_y = size_y / 2 if p1[1] >= center[1] else -size_y / 2
            return [
                {"x": 0.0, "y": round(detour_y, 1), "z": 0.0},
                {"x": round(dx, 1), "y": 0.0, "z": round(dz, 1)},
                {"x": 0.0, "y": round(-detour_y + (p2[1] - p1[1]), 1), "z": 0.0},
            ]
        else:
            detour_x = size_x / 2 if p1[0] >= center[0] else -size_x / 2
            return [
                {"x": round(detour_x, 1), "y": 0.0, "z": 0.0},
                {"x": 0.0, "y": round(dy, 1), "z": round(dz, 1)},
                {"x": round(-detour_x + (p2[0] - p1[0]), 1), "y": 0.0, "z": 0.0},
            ]

    def _correct_endpoint(
        self,
        start: Tuple[float, float, float],
        dest: Tuple[float, float, float],
        vectors: List[Dict[str, float]],
    ) -> List[Dict[str, float]]:
        positions = _reconstruct_positions(start, vectors)
        actual_end = positions[-1]
        gx = round(dest[0] - actual_end[0], 1)
        gy = round(dest[1] - actual_end[1], 1)
        gz = round(dest[2] - actual_end[2], 1)
        gap = math.sqrt(gx * gx + gy * gy + gz * gz)
        if gap > 1.0:
            if abs(gz) > 1.0:
                vectors.append({"x": 0.0, "y": 0.0, "z": gz})
            if abs(gx) > 1.0:
                vectors.append({"x": gx, "y": 0.0, "z": 0.0})
            if abs(gy) > 1.0:
                vectors.append({"x": 0.0, "y": gy, "z": 0.0})
        return vectors

    def _cleanup_vectors(self, vectors: List[Dict[str, float]]) -> List[Dict[str, float]]:
        # 영벡터 제거
        cleaned = [v for v in vectors if _vec_length(v) > 0.1]
        # 연속 동일 방향 병합
        if len(cleaned) <= 1:
            return cleaned
        merged: List[Dict[str, float]] = [dict(cleaned[0])]
        for v in cleaned[1:]:
            prev = merged[-1]
            if _segment_type(prev) == _segment_type(v) and _vec_length(v) < self.config.min_segment_length:
                merged[-1] = _vec_add(prev, v)
            else:
                merged.append(dict(v))
        return [v for v in merged if _vec_length(v) > 0.1]

    def _build_simple_connection(self, from_pos, to_pos):
        dx = round(to_pos[0] - from_pos[0], 1)
        dy = round(to_pos[1] - from_pos[1], 1)
        dz = round(to_pos[2] - from_pos[2], 1)
        segs = []
        if abs(dz) > self.config.min_segment_length:
            segs.append({"x": 0.0, "y": 0.0, "z": dz})
        if abs(dx) > self.config.min_segment_length:
            segs.append({"x": dx, "y": 0.0, "z": 0.0})
        if abs(dy) > self.config.min_segment_length:
            segs.append({"x": 0.0, "y": dy, "z": 0.0})
        return segs or [{"x": dx, "y": dy, "z": dz}]

    def _build_direct_path(self, start, dest, template):
        vectors = self._build_simple_connection(start, dest)
        vectors = self._apply_obstacle_avoidance(start, vectors)
        vectors = self._cleanup_vectors(vectors)
        return self._assemble_result(start, dest, vectors)

    def _assemble_result(self, start, dest, vectors):
        positions = _reconstruct_positions(start, vectors)
        lengths = [round(_vec_length(v), 1) for v in vectors]
        arrow = _compute_path_arrow(positions, self.config.direction_angle_tolerance)
        bbox = _bbox(positions)
        actual_end = positions[-1] if positions else start
        return {
            "start_pos": list(start),
            "end_pos": list(actual_end),
            "displacement_vector": {
                "x": round(actual_end[0] - start[0], 1),
                "y": round(actual_end[1] - start[1], 1),
                "z": round(actual_end[2] - start[2], 1),
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
            "h_segments": _extract_h_segments(positions, arrow),
            "path_summary": "AUTO_V2",
        }


# ─────────────────────────────────────────────────────────────
# 5. PathValidator V2
# ─────────────────────────────────────────────────────────────


class PathValidatorV2:
    """장애물 충돌 검사를 포함하는 확장 검증기"""

    def __init__(self, spatial: SpatialContext, config: DesignConfigV2):
        self.spatial = spatial
        self.config = config

    def validate(
        self,
        generated: Dict[str, Any],
        template: Dict[str, Any],
        group: Dict[str, Any],
    ) -> Dict[str, Any]:
        warnings: List[str] = []
        checks: Dict[str, float] = {}

        # 1. 좌표 연속성
        vecs = generated.get("path_step_vectors", [])
        start = generated.get("start_pos", [0, 0, 0])
        positions = generated.get("node_positions", [])
        reconstructed = _reconstruct_positions(start, vecs)
        continuous = all(
            all(abs(reconstructed[j][i] - positions[j][i]) < 1.5 for i in range(3))
            for j in range(min(len(reconstructed), len(positions)))
        )
        checks["continuity"] = 1.0 if continuous else 0.0
        if not continuous:
            warnings.append("좌표 연속성 실패")

        # 2. 길이 합리성
        gen_len = generated.get("path_total_length", 0)
        tmpl_len = template.get("path_total_length", 1)
        ratio = gen_len / tmpl_len if tmpl_len > 0 else 0
        in_range = self.config.min_length_ratio <= ratio <= self.config.max_length_ratio
        checks["length_ratio"] = _clamp01(1.0 - abs(1.0 - ratio) / 2.0) if in_range else 0.2
        if not in_range:
            warnings.append(f"길이 비율: {ratio:.2f}")

        # 3. 방향 패턴
        checks["pattern"] = pattern_similarity(
            generated.get("path_arrow", ""), template.get("path_arrow", "")
        )

        # 4. 장애물 충돌 검사 (COLUMN_STRUCTURE만 대상)
        collision_count = 0
        for i in range(len(positions) - 1):
            if _segment_type(vecs[i]) == "R":
                continue  # 수직 세그먼트는 기둥 충돌 무시
            cols = self.spatial.find_obstacles_in_path(
                positions[i], positions[i + 1], self.config.obstacle_clearance
            )
            collision_count += sum(1 for c in cols if c.ddworks_type == "COLUMN_STRUCTURE")
        checks["obstacle_free"] = _clamp01(1.0 - collision_count * 0.3)
        if collision_count > 0:
            warnings.append(f"기둥 충돌: {collision_count}개 세그먼트")

        # 5. Zone 준수
        trunk = group["zones"].get("trunk")
        if trunk and generated.get("path_bbox"):
            pb = generated["path_bbox"]
            score = 1.0
            for axis in ("x", "y"):
                t_range = (trunk.get(f"{axis}_max", 0) - trunk.get(f"{axis}_min", 0)) or 1000
                p_min = pb.get(f"{axis}_min", 0)
                p_max = pb.get(f"{axis}_max", 0)
                exp_min = trunk.get(f"{axis}_min", 0) - t_range * 0.5
                exp_max = trunk.get(f"{axis}_max", 0) + t_range * 0.5
                if p_min < exp_min or p_max > exp_max:
                    overshoot = max(exp_min - p_min, p_max - exp_max, 0)
                    score -= _clamp01(overshoot / t_range) * 0.5
            checks["zone_compliance"] = _clamp01(score)
        else:
            checks["zone_compliance"] = 0.5

        # 종합 점수
        quality = (
            checks["continuity"] * 0.20
            + checks["length_ratio"] * 0.20
            + checks["pattern"] * 0.15
            + checks["obstacle_free"] * 0.25
            + checks["zone_compliance"] * 0.20
        )

        return {
            "is_valid": quality >= 0.4 and checks["continuity"] >= 1.0,
            "quality_score": round(quality, 4),
            "checks": {k: round(v, 4) for k, v in checks.items()},
            "warnings": warnings,
            "collision_count": collision_count,
        }


# ─────────────────────────────────────────────────────────────
# 6. AutoRoutingDesigner V2 - 메인 오케스트레이터
# ─────────────────────────────────────────────────────────────


class AutoRoutingDesignerV2:
    """장비·장애물·종단점을 고려한 자동 배관 경로 설계 시스템"""

    def __init__(self, config: DesignConfigV2):
        self.config = config
        self.spatial = SpatialContext()
        self.groups: List[Dict[str, Any]] = []
        self.feature_extractor: Optional[EnhancedFeatureExtractor] = None
        self.similarity = SpatialSimilarity(config)
        self.builder: Optional[ObstacleAwarePathBuilder] = None
        self.validator: Optional[PathValidatorV2] = None

    def load_data(
        self,
        json_input: Optional[str] = None,
        group_results: Optional[str] = None,
    ) -> None:
        """원본 JSON + GroupPipeResults를 로드합니다."""
        # 원본 JSON 로딩 (장비/장애물/SpaceInfo)
        json_path = json_input or self.config.json_input_dir
        if os.path.isdir(json_path):
            json_files = glob.glob(os.path.join(json_path, "**", "*.json"), recursive=True)
        else:
            json_files = glob.glob(json_path)

        if json_files:
            # 첫 번째 매칭 파일로 SpatialContext 로드
            self.spatial.load_from_json(json_files[0])

        # GroupPipeResults 로딩
        grp_path = group_results or self.config.group_results_path
        with open(grp_path, "r", encoding="utf-8") as f:
            self.groups = json.load(f)
        LOGGER.info("그룹 데이터 로드: %d개 그룹", len(self.groups))

        self.feature_extractor = EnhancedFeatureExtractor(self.spatial, self.config)
        self.builder = ObstacleAwarePathBuilder(self.spatial, self.config)
        self.validator = PathValidatorV2(self.spatial, self.config)

    def find_matching_group(
        self,
        equipment_name: str,
        utility: str,
        size: str,
        start_pos: Tuple[float, float, float],
        dest_pos: Tuple[float, float, float],
    ) -> Optional[Tuple[Dict[str, Any], Dict[str, Any], float]]:
        """
        확장 유사도 기반 그룹 + 대표 경로 매칭.
        반환: (group, best_path, best_score) 또는 None
        """
        # 새 경로의 특징량 추출
        rough_vectors = self._estimate_rough_vectors(start_pos, dest_pos)
        poc_info = self._find_poc_info(utility, start_pos)
        new_feat = self.feature_extractor.extract(start_pos, dest_pos, rough_vectors, poc_info)

        best: Optional[Tuple[Dict, Dict, float]] = None
        best_score = -1.0

        for g in self.groups:
            if g["equipment_name"] != equipment_name:
                continue
            if g["utility"] != utility:
                continue
            if g["size"] != size:
                continue

            for path in g["paths"]:
                sp = tuple(path["start_pos"])
                ep = tuple(path.get("end_pos", sp))
                xy_dist = _dist_xy(start_pos, sp)
                if xy_dist > self.config.max_start_xy_distance:
                    continue

                path_vecs = path.get("path_step_vectors", [])
                path_feat = self.feature_extractor.extract(sp, ep, path_vecs, poc_info)
                score = self.similarity.compute(new_feat, path_feat)

                # XY 근접성 보너스
                proximity_bonus = _clamp01(1.0 - xy_dist / self.config.max_start_xy_distance) * 0.1
                score = _clamp01(score + proximity_bonus)

                if score > best_score:
                    best_score = score
                    best = (g, path, score)

        if not best:
            # Fallback: equipment만 일치
            for g in self.groups:
                if g["equipment_name"] != equipment_name:
                    continue
                for path in g["paths"]:
                    sp = tuple(path["start_pos"])
                    xy_dist = _dist_xy(start_pos, sp)
                    if xy_dist <= self.config.max_start_xy_distance * 1.5:
                        if not best or g["avg_similarity"] > best[0].get("avg_similarity", 0):
                            best = (g, path, 0.3)
                            break

        if best:
            LOGGER.info(
                "매칭: group=#%d, score=%.4f, template_poc=%s",
                best[0]["group_id"], best[2], best[1].get("poc_id", "?"),
            )
        return best

    def design_route(
        self,
        equipment_name: str,
        utility: str,
        size: str,
        start_pos: Tuple[float, float, float],
        dest_pos: Tuple[float, float, float],
    ) -> Optional[Dict[str, Any]]:
        """단일 경로를 자동 설계합니다."""
        match = self.find_matching_group(equipment_name, utility, size, start_pos, dest_pos)
        if not match:
            LOGGER.error("매칭 실패: %s/%s/%s", equipment_name, utility, size)
            return None

        group, template, match_score = match

        # 경로 구성 (장애물 회피 포함)
        raw_path = self.builder.build_path(start_pos, dest_pos, group, template)

        # 검증
        validation = self.validator.validate(raw_path, template, group)

        # 확장 특징량 추출
        poc_info = self._find_poc_info(utility, start_pos)
        features = self.feature_extractor.extract(
            start_pos, dest_pos, raw_path["path_step_vectors"], poc_info
        )

        # 유사도 상세
        tmpl_feat = self.feature_extractor.extract(
            tuple(template["start_pos"]),
            tuple(template.get("end_pos", template["start_pos"])),
            template.get("path_step_vectors", []),
            poc_info,
        )
        sim_detail = self.similarity.compute_detail(features, tmpl_feat)

        result = {
            "group_id": f"AUTO_{group['group_id']:03d}",
            "source_group_id": group["group_id"],
            "is_auto_generated": True,
            "version": "V2",
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
            "match_score": match_score,
            "validation": validation,
            "similarity_detail": sim_detail,
            "spatial_context": {
                "equipment_bbox": {
                    "min": list(self.spatial.equipment.bbox_min) if self.spatial.equipment else None,
                    "max": list(self.spatial.equipment.bbox_max) if self.spatial.equipment else None,
                },
                "obstacles_nearby": features.obstacle_count_nearby,
                "obstacle_density": features.obstacle_density,
                "column_crossings_template": features.column_crossings,
                "column_collisions_result": validation.get("collision_count", 0),
                "start_level": features.start_level,
                "end_level": features.end_level,
                "levels_traversed": features.levels_traversed,
                "start_face": features.start_face,
                "terminal_type": features.terminal_type,
            },
            "paths": [{
                "poc_id": f"auto_v2_{datetime.now().strftime('%Y%m%d%H%M%S')}",
                "terminal_label": "자동 생성 경로 (V2)",
                **raw_path,
            }],
        }

        status = "PASS" if validation["is_valid"] else "WARN"
        LOGGER.info(
            "[%s] V2 경로: %s/%s/%s  quality=%.4f match=%.4f length=%.1f "
            "collisions=%d obstacles_nearby=%d levels=%s",
            status, equipment_name, utility, size,
            validation["quality_score"], match_score,
            raw_path["path_total_length"],
            validation.get("collision_count", 0),
            features.obstacle_count_nearby,
            "→".join(features.levels_traversed),
        )
        for w in validation["warnings"]:
            LOGGER.warning("  - %s", w)

        return result

    def design_routes_batch(self, poc_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        results = []
        for i, poc in enumerate(poc_list):
            LOGGER.info("[배치 %d/%d] %s/%s/%s", i + 1, len(poc_list),
                        poc["equipment_name"], poc["utility"], poc["size"])
            r = self.design_route(
                poc["equipment_name"], poc["utility"], poc["size"],
                tuple(poc["start_pos"]), tuple(poc["dest_pos"]),
            )
            if r:
                results.append(r)
        LOGGER.info("배치 완료: %d/%d 성공", len(results), len(poc_list))
        return results

    def save_results(self, results: List[Dict[str, Any]], output_path: Optional[str] = None) -> str:
        os.makedirs(self.config.output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d%H%M%S")
        out = output_path or os.path.join(self.config.output_dir, f"auto_routing_v2_{ts}.json")
        with open(out, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        LOGGER.info("결과 저장: %s (%d개)", out, len(results))
        return out

    def list_groups(self, filter_eq: Optional[str] = None, filter_ut: Optional[str] = None) -> None:
        print(f"\n{'ID':>4} | {'장비명':<25} | {'유틸리티':<12} | {'사이즈':<8} | {'경로수':>5} | {'유사도':>6} | {'Trunk Z':>10}")
        print("-" * 95)
        for g in self.groups:
            if filter_eq and g["equipment_name"] != filter_eq:
                continue
            if filter_ut and g["utility"] != filter_ut:
                continue
            tz = g["zones"]["trunk"]["mean_z"] if g["zones"].get("trunk") else "-"
            print(f"{g['group_id']:>4} | {g['equipment_name']:<25} | {g['utility']:<12} | "
                  f"{g['size']:<8} | {g['path_count']:>5} | {g['avg_similarity']:>6.3f} | {tz:>10}")

    def print_spatial_summary(self) -> None:
        """로드된 공간 컨텍스트 요약을 출력합니다."""
        eq = self.spatial.equipment
        if eq:
            print(f"\n장비: {eq.name} ({eq.process})")
            print(f"  위치: {eq.position}")
            print(f"  BBox: {eq.bbox_min} ~ {eq.bbox_max}")
            print(f"  크기: {eq.size}")
            print(f"  POC: {len(eq.poc_list)}개, Ends: {len(eq.ends)}개")

        print(f"\n장애물: {len(self.spatial.obstacles)}개")
        print(f"  기둥: {len(self.spatial.columns)}개")
        print(f"  빔: {len(self.spatial.beams)}개")

        print(f"\n공간 레벨:")
        for lv in self.spatial.levels:
            print(f"  {lv.name}: Z {lv.z_min} ~ {lv.z_max}")

    def _find_poc_info(self, utility: str, start_pos: Tuple[float, float, float]) -> Optional[Dict]:
        """장비 POC 목록에서 utility + 위치가 가장 가까운 POC를 찾습니다."""
        eq = self.spatial.equipment
        if not eq:
            return None
        best_poc = None
        best_dist = float("inf")
        for poc in eq.poc_list:
            if poc.get("utility") != utility:
                continue
            pp = poc.get("pocPosition")
            if not pp:
                continue
            d = _dist_xyz(start_pos, tuple(pp))
            if d < best_dist:
                best_dist = d
                best_poc = poc
        return best_poc

    def _estimate_rough_vectors(
        self,
        start: Tuple[float, float, float],
        dest: Tuple[float, float, float],
    ) -> List[Dict[str, float]]:
        """대략적인 경로 벡터를 추정합니다 (특징량 비교용)."""
        dx = dest[0] - start[0]
        dy = dest[1] - start[1]
        dz = dest[2] - start[2]
        vecs = []
        if abs(dz) > 50:
            vecs.append({"x": 0.0, "y": 0.0, "z": round(dz / 2, 1)})
        if abs(dx) > 50:
            vecs.append({"x": round(dx, 1), "y": 0.0, "z": 0.0})
        if abs(dy) > 50:
            vecs.append({"x": 0.0, "y": round(dy, 1), "z": 0.0})
        if abs(dz) > 50:
            vecs.append({"x": 0.0, "y": 0.0, "z": round(dz / 2, 1)})
        return vecs or [{"x": round(dx, 1), "y": round(dy, 1), "z": round(dz, 1)}]


# ─────────────────────────────────────────────────────────────
# 7. CLI
# ─────────────────────────────────────────────────────────────


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="자동 배관 경로 설계 V2 (장비·장애물·종단점 고려)")
    p.add_argument("--json_input", default="./data-v10", help="원본 설계 JSON (파일 또는 디렉토리)")
    p.add_argument("--group_results", default="./GroupPipeResults/group_pipe_results_20260405194814.json")
    p.add_argument("--output", default="./AutoRoutingResults")
    p.add_argument("--log_level", default="INFO")

    p.add_argument("--equipment", help="장비명")
    p.add_argument("--utility", help="유틸리티")
    p.add_argument("--size", help="사이즈")
    p.add_argument("--start", help="시작 좌표 (x,y,z)")
    p.add_argument("--dest", help="목적지 좌표 (x,y,z)")

    p.add_argument("--batch_input", help="배치 입력 JSON")
    p.add_argument("--list_groups", action="store_true")
    p.add_argument("--spatial_summary", action="store_true", help="공간 컨텍스트 요약 출력")
    p.add_argument("--filter_equipment", help="장비명 필터")
    p.add_argument("--filter_utility", help="유틸리티 필터")
    return p


def _parse_coords(s: str) -> Tuple[float, float, float]:
    parts = [float(x.strip()) for x in s.split(",")]
    return (parts[0], parts[1], parts[2])


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(message)s",
    )

    config = DesignConfigV2(
        json_input_dir=args.json_input,
        group_results_path=args.group_results,
        output_dir=args.output,
    )

    designer = AutoRoutingDesignerV2(config)
    designer.load_data(json_input=args.json_input, group_results=args.group_results)

    if args.spatial_summary:
        designer.print_spatial_summary()
        return

    if args.list_groups:
        designer.list_groups(args.filter_equipment, args.filter_utility)
        return

    if args.batch_input:
        with open(args.batch_input, "r", encoding="utf-8") as f:
            poc_list = json.load(f)
        results = designer.design_routes_batch(poc_list)
        if results:
            designer.save_results(results)
        return

    if args.equipment and args.utility and args.size and args.start and args.dest:
        result = designer.design_route(
            args.equipment, args.utility, args.size,
            _parse_coords(args.start), _parse_coords(args.dest),
        )
        if result:
            out = designer.save_results([result])
            v = result["validation"]
            sc = result.get("spatial_context", {})
            print(f"\n결과 저장: {out}")
            print(f"품질 점수: {v['quality_score']:.4f}")
            print(f"매칭 점수: {result['match_score']:.4f}")
            print(f"경로 길이: {result['paths'][0]['path_total_length']:.1f} mm")
            print(f"방향 패턴: {result['paths'][0]['path_arrow']}")
            print(f"기둥 충돌: {v.get('collision_count', 0)}개")
            print(f"주변 장애물: {sc.get('obstacles_nearby', 0)}개")
            print(f"레벨 경유: {' → '.join(sc.get('levels_traversed', []))}")
            print(f"시작 면: {sc.get('start_face', '-')}")
            print(f"종단 타입: {sc.get('terminal_type', '-')}")
            if v["warnings"]:
                print("\n경고:")
                for w in v["warnings"]:
                    print(f"  - {w}")
        return

    parser.print_help()


if __name__ == "__main__":
    main()
