from __future__ import annotations

# =====================================================================================
# [실행 명령어]
#
#   (1) 로컬 Top-K 검색 테스트 — DB 없이 GroupPipeResults JSON으로 검색
#   python TopKRoutingSearch.py test \
#       --group_results ../data/GroupPipeResults/group_pipe_results_20260405194814.json \
#       --k 5
#
#   (2) pgvector DB 기반 Top-K 검색 — 시작/종단 좌표로 유사 경로 검색
#   python TopKRoutingSearch.py search \
#       --start 205117.7,15457.4,15495.0 \
#       --end 208364.8,8689.6,12372.7 \
#       --utility NW --size 3/4B --k 10 \
#       --dbname AUTOROUTINGV7 --user postgres --password dinno \
#       --host localhost --port 5432 \
#       --norm_params ../data/FeatureVectors/db_norm_params.json
#
# =====================================================================================

# DB 기반 검색
# python TopKRoutingSearch.py search --start 5500,28200,15495 --end 5300,27000,15495 --utility AKWW --size 20A --k 5 --dbname AUTOROUTINGV7 --user postgres --password xxx#
#
# =====================================================================================
"""
TopKRoutingSearch.py  — Top-K 유사 배관 경로 검색 모듈
=====================================================

[프로그램 개요]
  기존 배관 경로 데이터를 30차원 특징벡터로 인코딩하고,
  pgvector 기반 ANN(Approximate Nearest Neighbor) 검색으로
  유사한 경로 Top-K개를 빠르게 반환합니다.

[30D 특징벡터 구성 (확정안)]
  ┌──────────┬─────────────────────────┬──────────┐
  │ Index    │ 특성                    │ 가중치   │
  ├──────────┼─────────────────────────┼──────────┤
  │ [0~2]    │ 시작 위상 (출발 방향)   │ 0.20     │
  │ [3~5]    │ 종단 위상 (진입 방향)   │ 0.20     │
  │ [6~8]    │ 공간 변위 (상대위치)    │ 0.15     │
  │ [9~11]   │ 바운딩 박스 (3D 비율)   │ 0.15     │
  │ [12~14]  │ 구간 1 (초기 33% 꺾임)  │ 0.06     │
  │ [15~17]  │ 구간 2 (중기 33% 꺾임)  │ 0.06     │
  │ [18~20]  │ 구간 3 (후기 33% 꺾임)  │ 0.06     │
  │ [21~24]  │ 환경/비용 (4D)          │ 0.12     │
  │ [25~29]  │ 예비 (0.0 패딩)         │ -        │
  └──────────┴─────────────────────────┴──────────┘

[주요 클래스]
  - RoutePathLoader          : DB에서 경로 데이터 로드 (TB_ROUTE_PATH + SEGMENT_DETAIL)
  - PathFeatureEncoder       : 경로 → 30D 벡터 인코딩
  - NormalizationParams      : 정규화 파라미터 관리
  - FeatureVectorDB          : DB CRUD + pgvector 검색
  - TopKSearchEngine         : 통합 검색 엔진

[주요 변수 설명]
  - VECTOR_DIM               : 벡터 차원 수 (30)
  - WEIGHT_MAP               : 차원 그룹별 가중치
  - NORMALIZATION_DEFAULTS   : 기본 정규화 파라미터
"""

import argparse
import json
import logging
import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────
# 0. 설정
# ─────────────────────────────────────────────────────────────

LOGGER = logging.getLogger("TopKRoutingSearch")

VECTOR_DIM = 30

# 차원 그룹별 가중치 및 인덱스 범위
WEIGHT_MAP = {
    "start_topology":  {"indices": (0, 3),   "weight": 0.20},
    "end_topology":    {"indices": (3, 6),   "weight": 0.20},
    "displacement":    {"indices": (6, 9),   "weight": 0.15},
    "bounding_box":    {"indices": (9, 12),  "weight": 0.15},
    "segment_1":       {"indices": (12, 15), "weight": 0.06},
    "segment_2":       {"indices": (15, 18), "weight": 0.06},
    "segment_3":       {"indices": (18, 21), "weight": 0.06},
    "env_cost":        {"indices": (21, 25), "weight": 0.12},
    "reserved":        {"indices": (25, 30), "weight": 0.00},
}

# 기본 정규화 파라미터 (mm 단위 기준)
NORMALIZATION_DEFAULTS = {
    "bbox_max": {"x": 30000.0, "y": 30000.0, "z": 20000.0},
    "displacement_max": 50000.0,
    "total_length_max": 50000.0,
    "bend_count_max": 30,
    "obstacle_score_max": 1.0,
    "support_score_max": 1.0,
}


# ─────────────────────────────────────────────────────────────
# 1. RoutePathLoader - DB에서 경로 데이터 로드
# ─────────────────────────────────────────────────────────────

# path_arrow 방향 분류 각도 허용 범위 (도)
_DIRECTION_ANGLE_TOLERANCE = 5.0


def _compute_segment_code_from_vector(dx: float, dy: float, dz: float) -> str:
    """변위 벡터에서 방향 코드(R/H/D)를 계산합니다."""
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-6:
        return ""
    vert_angle = math.degrees(math.asin(min(abs(dz) / length, 1.0)))
    if vert_angle > (90.0 - _DIRECTION_ANGLE_TOLERANCE):
        return "R"  # Riser (수직)
    elif vert_angle < _DIRECTION_ANGLE_TOLERANCE:
        return "H"  # Header (수평)
    else:
        return "D"  # Diagonal


class RoutePathLoader:
    """PostgreSQL DB에서 경로 데이터를 로드하여 인코더 입력 형태로 변환합니다.

    테이블 관계 (정식 경로)
    ----------------------
    TB_ROUTE_PATH (메인)
      └→ TB_ROUTE_PATH_SEGMENT_MAP (경로↔구간 매핑, SEGMENT_ORDER 순)
           └→ TB_ROUTE_SEGMENTS (경로 구간)
                └→ TB_ROUTE_SEGMENT_DETAIL (배관/자재 상세, ORDER 순)

    폴백 경로 (MAP 데이터 미존재 시)
    --------------------------------
    TB_ROUTE_PATH (메인)
      └→ TB_ROUTE_SEGMENTS (ROUTE_PATH_GUID 직접 조인, ORDER 순)
           └→ TB_ROUTE_SEGMENT_DETAIL (배관/자재 상세, ORDER 순)
    """

    # segment_detail 조회 결과의 컬럼 인덱스 (두 쿼리 모두 동일 구조로 SELECT)
    #   0: ROUTE_PATH_ID, 1: detail_order, 2: TYPE,
    #   3: FROM_POSX, 4: FROM_POSY, 5: FROM_POSZ,
    #   6: TO_POSX, 7: TO_POSY, 8: TO_POSZ,
    #   9: LEVEL, 10: SIZE, 11: UTILITY_GROUP, 12: UTILITY,
    #   13: segment_order

    # MAP 경유 일괄 조회 SQL
    _SQL_DETAIL_VIA_MAP = """
        SELECT
            sm."ROUTE_PATH_ID",
            sd."ORDER"          AS detail_order,
            sd."TYPE",
            sd."FROM_POSX", sd."FROM_POSY", sd."FROM_POSZ",
            sd."TO_POSX",   sd."TO_POSY",   sd."TO_POSZ",
            sd."LEVEL",
            sd."SIZE",
            sd."UTILITY_GROUP",
            sd."UTILITY",
            sm."SEGMENT_ORDER"  AS segment_order
        FROM "TB_ROUTE_PATH_SEGMENT_MAP" sm
        JOIN "TB_ROUTE_SEGMENTS" rs ON sm."SEGMENT_GUID" = rs."SEGMENT_GUID"
        JOIN "TB_ROUTE_SEGMENT_DETAIL" sd ON sd."SEGMENT_GUID" = rs."SEGMENT_GUID"
        ORDER BY sm."ROUTE_PATH_ID", sm."SEGMENT_ORDER", sd."ORDER";
    """

    # 직접 조인 일괄 조회 SQL (폴백)
    _SQL_DETAIL_DIRECT = """
        SELECT
            rs."ROUTE_PATH_GUID",
            sd."ORDER"   AS detail_order,
            sd."TYPE",
            sd."FROM_POSX", sd."FROM_POSY", sd."FROM_POSZ",
            sd."TO_POSX",   sd."TO_POSY",   sd."TO_POSZ",
            sd."LEVEL",
            sd."SIZE",
            sd."UTILITY_GROUP",
            sd."UTILITY",
            rs."ORDER"  AS segment_order
        FROM "TB_ROUTE_SEGMENT_DETAIL" sd
        JOIN "TB_ROUTE_SEGMENTS" rs ON sd."SEGMENT_GUID" = rs."SEGMENT_GUID"
        ORDER BY rs."ROUTE_PATH_GUID", rs."ORDER", sd."ORDER";
    """

    # MAP 경유 단건 조회 SQL
    _SQL_DETAIL_VIA_MAP_SINGLE = """
        SELECT
            sm."ROUTE_PATH_ID",
            sd."ORDER"          AS detail_order,
            sd."TYPE",
            sd."FROM_POSX", sd."FROM_POSY", sd."FROM_POSZ",
            sd."TO_POSX",   sd."TO_POSY",   sd."TO_POSZ",
            sd."LEVEL",
            sd."SIZE",
            sd."UTILITY_GROUP",
            sd."UTILITY",
            sm."SEGMENT_ORDER"  AS segment_order
        FROM "TB_ROUTE_PATH_SEGMENT_MAP" sm
        JOIN "TB_ROUTE_SEGMENTS" rs ON sm."SEGMENT_GUID" = rs."SEGMENT_GUID"
        JOIN "TB_ROUTE_SEGMENT_DETAIL" sd ON sd."SEGMENT_GUID" = rs."SEGMENT_GUID"
        WHERE sm."ROUTE_PATH_ID" = %s
        ORDER BY sm."SEGMENT_ORDER", sd."ORDER";
    """

    # 직접 조인 단건 조회 SQL (폴백)
    _SQL_DETAIL_DIRECT_SINGLE = """
        SELECT
            rs."ROUTE_PATH_GUID",
            sd."ORDER"   AS detail_order,
            sd."TYPE",
            sd."FROM_POSX", sd."FROM_POSY", sd."FROM_POSZ",
            sd."TO_POSX",   sd."TO_POSY",   sd."TO_POSZ",
            sd."LEVEL",
            sd."SIZE",
            sd."UTILITY_GROUP",
            sd."UTILITY",
            rs."ORDER"  AS segment_order
        FROM "TB_ROUTE_SEGMENT_DETAIL" sd
        JOIN "TB_ROUTE_SEGMENTS" rs ON sd."SEGMENT_GUID" = rs."SEGMENT_GUID"
        WHERE rs."ROUTE_PATH_GUID" = %s
        ORDER BY rs."ORDER", sd."ORDER";
    """

    def __init__(self, db_params: Dict[str, str]):
        self.db_params = db_params
        self._conn = None
        self._use_map = False  # MAP 경유 여부 (connect 시 자동 판별)

    def connect(self) -> None:
        import psycopg2
        self._conn = psycopg2.connect(**self.db_params)
        self._conn.set_client_encoding("UTF8")
        self._use_map = self._detect_map_available()
        mode = "MAP 경유" if self._use_map else "직접 조인 (폴백)"
        LOGGER.info("RoutePathLoader DB 연결 성공 [%s]", mode)

    def close(self) -> None:
        if self._conn:
            self._conn.close()
            self._conn = None

    def _detect_map_available(self) -> bool:
        """TB_ROUTE_PATH_SEGMENT_MAP이 사용 가능한지 판별합니다.

        MAP 테이블에 데이터가 있고, SEGMENTS 테이블과 GUID 매칭이 되면 True.
        """
        cur = self._conn.cursor()
        try:
            cur.execute("""
                SELECT COUNT(*) FROM "TB_ROUTE_PATH_SEGMENT_MAP" sm
                JOIN "TB_ROUTE_SEGMENTS" rs ON sm."SEGMENT_GUID" = rs."SEGMENT_GUID"
                LIMIT 1;
            """)
            count = cur.fetchone()[0]
            return count > 0
        except Exception:
            return False
        finally:
            cur.close()

    def load_all_paths(self) -> List[Dict[str, Any]]:
        """전체 경로를 로드하여 인코더 입력 레코드로 변환합니다."""
        cur = self._conn.cursor()
        try:
            # 1. 모든 경로 메타 로드
            cur.execute("""
                SELECT "ROUTE_PATH_GUID", "PROCESS_NAME", "BAY", "MAKER",
                       "UTILITY_GROUP", "SOURCE_OWNER_NAME", "SOURCE_UTILITY",
                       "SOURCE_SIZE", "SOURCE_LEVEL",
                       "SOURCE_POSX", "SOURCE_POSY", "SOURCE_POSZ",
                       "TARGET_OWNER_NAME", "TARGET_UTILITY", "TARGET_SIZE",
                       "TARGET_LEVEL",
                       "TARGET_POSX", "TARGET_POSY", "TARGET_POSZ",
                       "PR_BRANCH_COUNT", "PR_BEND_COUNT",
                       "PR_PATH_EFFICIENCY", "PR_TOTAL_LENGTH"
                FROM "TB_ROUTE_PATH";
            """)
            path_rows = cur.fetchall()
            path_cols = [desc[0] for desc in cur.description]
            LOGGER.info("TB_ROUTE_PATH 로드: %d건", len(path_rows))

            # 2. 전체 segment_detail 일괄 로드 (MAP 경유 또는 직접 조인)
            if self._use_map:
                cur.execute(self._SQL_DETAIL_VIA_MAP)
                LOGGER.info("TB_ROUTE_PATH_SEGMENT_MAP 경유 조회")
            else:
                cur.execute(self._SQL_DETAIL_DIRECT)
                LOGGER.info("TB_ROUTE_SEGMENTS 직접 조인 조회")

            detail_rows = cur.fetchall()
            LOGGER.info("TB_ROUTE_SEGMENT_DETAIL 로드: %d건", len(detail_rows))

            # 3. 경로별 detail 그룹핑
            from collections import defaultdict
            details_by_path: Dict[str, List[tuple]] = defaultdict(list)
            for row in detail_rows:
                details_by_path[row[0]].append(row)

            # 4. 각 경로를 인코더 입력 레코드로 변환
            records = []
            for path_row in path_rows:
                path_dict = dict(zip(path_cols, path_row))
                guid = path_dict["ROUTE_PATH_GUID"]
                details = details_by_path.get(guid, [])

                record = self._build_record(path_dict, details)
                if record:
                    records.append(record)

            LOGGER.info("인코더 입력 레코드 변환 완료: %d건", len(records))
            return records

        finally:
            cur.close()

    def load_path(self, route_path_guid: str) -> Optional[Dict[str, Any]]:
        """단일 경로를 로드합니다."""
        cur = self._conn.cursor()
        try:
            cur.execute("""
                SELECT "ROUTE_PATH_GUID", "PROCESS_NAME", "BAY", "MAKER",
                       "UTILITY_GROUP", "SOURCE_OWNER_NAME", "SOURCE_UTILITY",
                       "SOURCE_SIZE", "SOURCE_LEVEL",
                       "SOURCE_POSX", "SOURCE_POSY", "SOURCE_POSZ",
                       "TARGET_OWNER_NAME", "TARGET_UTILITY", "TARGET_SIZE",
                       "TARGET_LEVEL",
                       "TARGET_POSX", "TARGET_POSY", "TARGET_POSZ",
                       "PR_BRANCH_COUNT", "PR_BEND_COUNT",
                       "PR_PATH_EFFICIENCY", "PR_TOTAL_LENGTH"
                FROM "TB_ROUTE_PATH"
                WHERE "ROUTE_PATH_GUID" = %s;
            """, (route_path_guid,))
            row = cur.fetchone()
            if not row:
                return None

            path_cols = [desc[0] for desc in cur.description]
            path_dict = dict(zip(path_cols, row))

            if self._use_map:
                cur.execute(self._SQL_DETAIL_VIA_MAP_SINGLE, (route_path_guid,))
            else:
                cur.execute(self._SQL_DETAIL_DIRECT_SINGLE, (route_path_guid,))
            details = cur.fetchall()

            return self._build_record(path_dict, details)
        finally:
            cur.close()

    def _build_record(
        self, path_dict: Dict[str, Any], details: List[tuple]
    ) -> Optional[Dict[str, Any]]:
        """DB 행을 인코더 입력 레코드로 변환합니다.

        detail 행 구조 (인덱스):
          0: ROUTE_PATH_GUID, 1: detail_order, 2: TYPE,
          3: FROM_POSX, 4: FROM_POSY, 5: FROM_POSZ,
          6: TO_POSX, 7: TO_POSY, 8: TO_POSZ,
          9: LEVEL, 10: SIZE, 11: UTILITY_GROUP, 12: UTILITY,
          13: segment_order
        """
        if not details:
            return None

        # step_vectors 추출: 각 detail의 FROM→TO 변위
        step_vectors = []
        positions = []  # 좌표 시퀀스 (BBox 계산용)

        for d in details:
            fx, fy, fz = d[3], d[4], d[5]
            tx, ty, tz = d[6], d[7], d[8]

            if not positions:
                positions.append((fx, fy, fz))

            dx = tx - fx
            dy = ty - fy
            dz = tz - fz
            seg_len = math.sqrt(dx * dx + dy * dy + dz * dz)

            # 길이가 0인 세그먼트(POC 등 자기자신 좌표)는 건너뜀
            if seg_len < 1e-3:
                continue

            step_vectors.append({"x": dx, "y": dy, "z": dz})
            positions.append((tx, ty, tz))

        if not step_vectors:
            return None

        # path_arrow 생성
        arrow_codes = []
        for sv in step_vectors:
            code = _compute_segment_code_from_vector(sv["x"], sv["y"], sv["z"])
            if code:
                arrow_codes.append(code)
        path_arrow = "-".join(arrow_codes)

        # BBox 계산
        if positions:
            xs = [p[0] for p in positions]
            ys = [p[1] for p in positions]
            zs = [p[2] for p in positions]
            path_range = {
                "x": max(xs) - min(xs),
                "y": max(ys) - min(ys),
                "z": max(zs) - min(zs),
            }
        else:
            path_range = {"x": 0.0, "y": 0.0, "z": 0.0}

        # 총 길이
        total_length = path_dict.get("PR_TOTAL_LENGTH", 0.0) or 0.0
        if total_length <= 0:
            total_length = sum(
                math.sqrt(sv["x"] ** 2 + sv["y"] ** 2 + sv["z"] ** 2)
                for sv in step_vectors
            )

        # 시작/종단 레벨
        start_level = path_dict.get("SOURCE_LEVEL", "") or ""
        end_level = path_dict.get("TARGET_LEVEL", "") or ""

        return {
            "route_path_guid": path_dict["ROUTE_PATH_GUID"],
            "process_name": path_dict.get("PROCESS_NAME", "") or "",
            "equipment_process": path_dict.get("PROCESS_NAME", "") or "",
            "equipment_name": path_dict.get("SOURCE_OWNER_NAME", "") or "",
            "utility_group": path_dict.get("UTILITY_GROUP", "") or "",
            "utility": path_dict.get("SOURCE_UTILITY", "") or "",
            "size": path_dict.get("SOURCE_SIZE", "") or "",
            "start_pos": [
                path_dict.get("SOURCE_POSX", 0.0) or 0.0,
                path_dict.get("SOURCE_POSY", 0.0) or 0.0,
                path_dict.get("SOURCE_POSZ", 0.0) or 0.0,
            ],
            "end_pos": [
                path_dict.get("TARGET_POSX", 0.0) or 0.0,
                path_dict.get("TARGET_POSY", 0.0) or 0.0,
                path_dict.get("TARGET_POSZ", 0.0) or 0.0,
            ],
            "start_level": start_level,
            "end_level": end_level,
            "path_step_vectors": step_vectors,
            "path_arrow": path_arrow,
            "path_range": path_range,
            "path_total_length": total_length,
            "step_count": len(step_vectors),
            "direction_pattern": path_arrow,
            "obstacle_score": 0.0,
            "support_score": 0.0,
            "pr_bend_count": path_dict.get("PR_BEND_COUNT", 0.0) or 0.0,
            "pr_branch_count": path_dict.get("PR_BRANCH_COUNT", 0.0) or 0.0,
            "pr_path_efficiency": path_dict.get("PR_PATH_EFFICIENCY", 0.0) or 0.0,
        }


# ─────────────────────────────────────────────────────────────
# 2. NormalizationParams - 정규화 파라미터 관리
# ─────────────────────────────────────────────────────────────


@dataclass
class NormalizationParams:
    """벡터 인코딩 전 각 차원을 0~1 범위로 정규화하기 위한 파라미터."""

    bbox_max: Dict[str, float] = field(
        default_factory=lambda: dict(NORMALIZATION_DEFAULTS["bbox_max"])
    )
    displacement_max: float = NORMALIZATION_DEFAULTS["displacement_max"]
    total_length_max: float = NORMALIZATION_DEFAULTS["total_length_max"]
    bend_count_max: float = NORMALIZATION_DEFAULTS["bend_count_max"]
    obstacle_score_max: float = NORMALIZATION_DEFAULTS["obstacle_score_max"]
    support_score_max: float = NORMALIZATION_DEFAULTS["support_score_max"]

    def save(self, path: str) -> None:
        """JSON으로 저장합니다."""
        data = {
            "bbox_max": self.bbox_max,
            "displacement_max": self.displacement_max,
            "total_length_max": self.total_length_max,
            "bend_count_max": self.bend_count_max,
            "obstacle_score_max": self.obstacle_score_max,
            "support_score_max": self.support_score_max,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        LOGGER.info("정규화 파라미터 저장: %s", path)

    @classmethod
    def load(cls, path: str) -> "NormalizationParams":
        """JSON에서 로드합니다."""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        params = cls()
        params.bbox_max = data.get("bbox_max", params.bbox_max)
        params.displacement_max = data.get("displacement_max", params.displacement_max)
        params.total_length_max = data.get("total_length_max", params.total_length_max)
        params.bend_count_max = data.get("bend_count_max", params.bend_count_max)
        params.obstacle_score_max = data.get("obstacle_score_max", params.obstacle_score_max)
        params.support_score_max = data.get("support_score_max", params.support_score_max)
        return params

    @classmethod
    def from_dataset(cls, records: Sequence[Dict[str, Any]]) -> "NormalizationParams":
        """데이터셋에서 통계적으로 정규화 파라미터를 산출합니다."""
        params = cls()
        if not records:
            return params

        bbox_xs, bbox_ys, bbox_zs = [], [], []
        displacements, total_lengths, bend_counts = [], [], []

        for rec in records:
            pr = rec.get("path_range", {})
            bbox_xs.append(abs(pr.get("x", 0.0)))
            bbox_ys.append(abs(pr.get("y", 0.0)))
            bbox_zs.append(abs(pr.get("z", 0.0)))

            sp = rec.get("start_pos", [0, 0, 0])
            ep = rec.get("end_pos", [0, 0, 0])
            disp = math.sqrt(sum((a - b) ** 2 for a, b in zip(sp, ep)))
            displacements.append(disp)

            total_lengths.append(rec.get("path_total_length", 0.0))

            arrow = rec.get("path_arrow", "")
            bend_counts.append(arrow.count("-"))

        # 99퍼센타일 사용 (이상치 영향 최소화)
        def percentile_99(values):
            if not values:
                return 1.0
            sorted_vals = sorted(values)
            idx = min(int(len(sorted_vals) * 0.99), len(sorted_vals) - 1)
            return max(sorted_vals[idx], 1.0)

        params.bbox_max = {
            "x": percentile_99(bbox_xs),
            "y": percentile_99(bbox_ys),
            "z": percentile_99(bbox_zs),
        }
        params.displacement_max = percentile_99(displacements)
        params.total_length_max = percentile_99(total_lengths)
        params.bend_count_max = percentile_99(bend_counts)

        return params


# ─────────────────────────────────────────────────────────────
# 2. PathFeatureEncoder - 30D 벡터 인코딩
# ─────────────────────────────────────────────────────────────


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _normalize_vector(vec: Sequence[float]) -> List[float]:
    """3D 벡터를 단위벡터로 정규화합니다. 영벡터이면 [0,0,0]."""
    mag = math.sqrt(sum(v * v for v in vec))
    if mag < 1e-9:
        return [0.0] * len(vec)
    return [v / mag for v in vec]


def _compute_bend_direction(
    vec_before: List[float], vec_after: List[float]
) -> List[float]:
    """두 연속 벡터 사이의 꺾임 방향(외적)을 단위벡터로 반환합니다."""
    # 외적: before × after
    cx = vec_before[1] * vec_after[2] - vec_before[2] * vec_after[1]
    cy = vec_before[2] * vec_after[0] - vec_before[0] * vec_after[2]
    cz = vec_before[0] * vec_after[1] - vec_before[1] * vec_after[0]
    return _normalize_vector([cx, cy, cz])


class PathFeatureEncoder:
    """경로 레코드를 30D 특징벡터로 인코딩합니다.

    인코딩 순서
    -----------
    [0~2]   시작 위상 : 첫 번째 세그먼트 방향 단위벡터
    [3~5]   종단 위상 : 마지막 세그먼트 방향 단위벡터
    [6~8]   공간 변위 : start→end 상대위치 (정규화)
    [9~11]  바운딩 박스 : path_range XYZ (정규화)
    [12~14] 구간 1    : 초기 33% 구간의 지배적 꺾임 방향
    [15~17] 구간 2    : 중기 33% 구간의 지배적 꺾임 방향
    [18~20] 구간 3    : 후기 33% 구간의 지배적 꺾임 방향
    [21~24] 환경/비용 : 총길이, 꺾임횟수, 장애물, 서포트 (각 정규화)
    [25~29] 예비      : 0.0 패딩
    """

    def __init__(self, norm_params: Optional[NormalizationParams] = None):
        self.norm_params = norm_params or NormalizationParams()
        self._build_scale_factors()

    def _build_scale_factors(self) -> None:
        """가중치를 벡터 스케일 팩터로 변환합니다.

        코사인 거리에서 가중치를 반영하기 위해
        각 차원 그룹에 sqrt(weight * total_dim / group_dim) 스케일을 적용합니다.
        """
        self.scale_factors = np.ones(VECTOR_DIM, dtype=np.float64)
        for group_name, info in WEIGHT_MAP.items():
            start, end = info["indices"]
            weight = info["weight"]
            dim_count = end - start
            if weight > 0 and dim_count > 0:
                scale = math.sqrt(weight * VECTOR_DIM / dim_count)
                self.scale_factors[start:end] = scale

    def encode(self, record: Dict[str, Any]) -> np.ndarray:
        """경로 레코드를 30D 벡터로 인코딩합니다.

        Parameters
        ----------
        record : dict
            GroupPipeResults의 개별 경로 레코드. 필수 필드:
            - path_step_vectors: [{x,y,z}, ...]
            - start_pos: [x,y,z]
            - end_pos: [x,y,z]
            - path_range: {x,y,z}
            - path_total_length: float
            - path_arrow: str (선택)
            - obstacle_score: float (선택, 기본 0.0)
            - support_score: float (선택, 기본 0.0)

        Returns
        -------
        np.ndarray : shape (30,)
        """
        vec = np.zeros(VECTOR_DIM, dtype=np.float64)

        step_vectors = self._parse_step_vectors(record.get("path_step_vectors", []))

        # [0~2] 시작 위상: 첫 번째 세그먼트 방향 단위벡터
        vec[0:3] = self._encode_start_topology(step_vectors)

        # [3~5] 종단 위상: 마지막 세그먼트 방향 단위벡터
        vec[3:6] = self._encode_end_topology(step_vectors)

        # [6~8] 공간 변위: start→end 정규화된 방향
        vec[6:9] = self._encode_displacement(record)

        # [9~11] 바운딩 박스: XYZ 범위 정규화
        vec[9:12] = self._encode_bounding_box(record)

        # [12~20] 구간별 꺾임 방향 (3구간)
        seg_bends = self._encode_segment_bends(step_vectors)
        vec[12:15] = seg_bends[0]
        vec[15:18] = seg_bends[1]
        vec[18:21] = seg_bends[2]

        # [21~24] 환경/비용 (총길이, 꺾임횟수, 장애물, 서포트)
        vec[21:25] = self._encode_env_cost(record)

        # [25~29] 예비: 0.0 (이미 0으로 초기화됨)

        # 가중치 스케일 적용
        vec *= self.scale_factors

        return vec

    def normalize_l2(self, vec: np.ndarray) -> np.ndarray:
        """L2 정규화합니다. 영벡터이면 그대로 반환합니다."""
        norm = np.linalg.norm(vec)
        if norm < 1e-12:
            return vec
        return vec / norm

    def decode_explain(self, vec: np.ndarray) -> Dict[str, Any]:
        """벡터를 사람이 읽을 수 있는 형태로 해석합니다."""
        # 스케일 역변환
        unscaled = vec.copy()
        for group_name, info in WEIGHT_MAP.items():
            start, end = info["indices"]
            scale = self.scale_factors[start]
            if scale > 1e-12:
                unscaled[start:end] /= scale

        return {
            "start_topology": unscaled[0:3].tolist(),
            "end_topology": unscaled[3:6].tolist(),
            "displacement": unscaled[6:9].tolist(),
            "bounding_box": unscaled[9:12].tolist(),
            "segment_1_bend": unscaled[12:15].tolist(),
            "segment_2_bend": unscaled[15:18].tolist(),
            "segment_3_bend": unscaled[18:21].tolist(),
            "env_cost": {
                "total_length_norm": unscaled[21],
                "bend_count_norm": unscaled[22],
                "obstacle_score": unscaled[23],
                "support_score": unscaled[24],
            },
            "reserved": unscaled[25:30].tolist(),
        }

    # ── 내부 인코딩 메서드 ──────────────────────────────────

    def _parse_step_vectors(
        self, raw: Any
    ) -> List[List[float]]:
        """path_step_vectors를 [[x,y,z], ...] 형태로 파싱합니다."""
        if not raw:
            return []
        result = []
        for item in raw:
            if isinstance(item, dict):
                result.append([
                    float(item.get("x", 0.0)),
                    float(item.get("y", 0.0)),
                    float(item.get("z", 0.0)),
                ])
            elif isinstance(item, (list, tuple)) and len(item) >= 3:
                result.append([float(item[0]), float(item[1]), float(item[2])])
        return result

    def _encode_start_topology(self, step_vectors: List[List[float]]) -> List[float]:
        """시작 위상: 첫 번째 세그먼트의 방향 단위벡터."""
        if not step_vectors:
            return [0.0, 0.0, 0.0]
        return _normalize_vector(step_vectors[0])

    def _encode_end_topology(self, step_vectors: List[List[float]]) -> List[float]:
        """종단 위상: 마지막 세그먼트의 방향 단위벡터."""
        if not step_vectors:
            return [0.0, 0.0, 0.0]
        return _normalize_vector(step_vectors[-1])

    def _encode_displacement(self, record: Dict[str, Any]) -> List[float]:
        """공간 변위: start→end 정규화된 방향벡터."""
        sp = record.get("start_pos", [0, 0, 0])
        ep = record.get("end_pos", [0, 0, 0])
        if isinstance(sp, dict):
            sp = [sp.get("x", 0), sp.get("y", 0), sp.get("z", 0)]
        if isinstance(ep, dict):
            ep = [ep.get("x", 0), ep.get("y", 0), ep.get("z", 0)]

        dx = ep[0] - sp[0]
        dy = ep[1] - sp[1]
        dz = ep[2] - sp[2]

        d_max = self.norm_params.displacement_max
        return [
            _clamp01(dx / d_max) if d_max > 0 else 0.0,
            _clamp01(dy / d_max) if d_max > 0 else 0.0,
            _clamp01(dz / d_max) if d_max > 0 else 0.0,
        ]

    def _encode_bounding_box(self, record: Dict[str, Any]) -> List[float]:
        """바운딩 박스: path_range XYZ를 정규화합니다."""
        pr = record.get("path_range", {})
        if isinstance(pr, dict):
            rx = abs(pr.get("x", 0.0))
            ry = abs(pr.get("y", 0.0))
            rz = abs(pr.get("z", 0.0))
        else:
            rx, ry, rz = 0.0, 0.0, 0.0

        bm = self.norm_params.bbox_max
        return [
            _clamp01(rx / bm["x"]) if bm["x"] > 0 else 0.0,
            _clamp01(ry / bm["y"]) if bm["y"] > 0 else 0.0,
            _clamp01(rz / bm["z"]) if bm["z"] > 0 else 0.0,
        ]

    def _encode_segment_bends(
        self, step_vectors: List[List[float]]
    ) -> Tuple[List[float], List[float], List[float]]:
        """경로를 3등분하여 각 구간의 지배적 꺾임 방향을 계산합니다.

        길이 기준으로 3등분하고, 각 구간 내 꺾임(외적)의 합산 방향을
        단위벡터로 정규화합니다.
        """
        empty = [0.0, 0.0, 0.0]
        if len(step_vectors) < 2:
            return (empty, empty, empty)

        # 각 세그먼트의 길이 계산
        lengths = [math.sqrt(sum(v ** 2 for v in sv)) for sv in step_vectors]
        total_length = sum(lengths)
        if total_length < 1e-9:
            return (empty, empty, empty)

        # 누적 길이로 구간 경계 결정
        cumulative = []
        running = 0.0
        for length in lengths:
            running += length
            cumulative.append(running)

        boundary_1 = total_length / 3.0
        boundary_2 = total_length * 2.0 / 3.0

        # 각 꺾임점의 외적을 구간별로 누적
        seg_bends = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

        for i in range(len(step_vectors) - 1):
            bend = _compute_bend_direction(step_vectors[i], step_vectors[i + 1])

            # 꺾임점의 위치 = cumulative[i] (i번째 세그먼트 끝)
            pos = cumulative[i]
            if pos <= boundary_1:
                seg_idx = 0
            elif pos <= boundary_2:
                seg_idx = 1
            else:
                seg_idx = 2

            seg_bends[seg_idx][0] += bend[0]
            seg_bends[seg_idx][1] += bend[1]
            seg_bends[seg_idx][2] += bend[2]

        # 각 구간을 단위벡터로 정규화
        return (
            _normalize_vector(seg_bends[0]),
            _normalize_vector(seg_bends[1]),
            _normalize_vector(seg_bends[2]),
        )

    def _encode_env_cost(self, record: Dict[str, Any]) -> List[float]:
        """환경/비용: [총길이, 꺾임횟수, 장애물, 서포트] 정규화."""
        np_ = self.norm_params

        # 총 길이
        total_length = record.get("path_total_length", 0.0)
        norm_length = _clamp01(total_length / np_.total_length_max) if np_.total_length_max > 0 else 0.0

        # 꺾임 횟수 (path_arrow의 '-' 개수 = 세그먼트 수 - 1 ≈ 꺾임 횟수)
        arrow = record.get("path_arrow", "")
        bend_count = arrow.count("-") if arrow else 0
        norm_bends = _clamp01(bend_count / np_.bend_count_max) if np_.bend_count_max > 0 else 0.0

        # 장애물 점수 (데이터 결측 시 0.0)
        obstacle_score = record.get("obstacle_score", 0.0)
        norm_obstacle = _clamp01(obstacle_score / np_.obstacle_score_max) if np_.obstacle_score_max > 0 else 0.0

        # 서포트 점수 (데이터 결측 시 0.0)
        support_score = record.get("support_score", 0.0)
        norm_support = _clamp01(support_score / np_.support_score_max) if np_.support_score_max > 0 else 0.0

        return [norm_length, norm_bends, norm_obstacle, norm_support]


# ─────────────────────────────────────────────────────────────
# 3. FeatureVectorDB - DB CRUD + pgvector 검색
# ─────────────────────────────────────────────────────────────


@dataclass
class SearchResult:
    """단일 검색 결과."""
    rank: int
    route_path_guid: str
    equipment_name: str
    process_name: str
    utility: str
    size: str
    direction_pattern: str
    total_length_mm: float
    step_count: int
    start_level: str
    end_level: str
    cosine_distance: float
    similarity_score: float
    feature_vector: Optional[np.ndarray] = None


@dataclass
class TopKResult:
    """Top-K 검색 결과 집합."""
    query_vector: np.ndarray
    results: List[SearchResult]
    search_time_ms: float
    filters_applied: Dict[str, Any] = field(default_factory=dict)
    total_candidates: int = 0


class FeatureVectorDB:
    """TB_ROUTE_FEATURE_VECTOR 테이블 CRUD 및 pgvector 검색."""

    TABLE_NAME = '"TB_ROUTE_FEATURE_VECTOR"'

    def __init__(self, db_params: Dict[str, str]):
        self.db_params = db_params
        self._conn = None

    def connect(self) -> None:
        """DB 연결을 수립합니다."""
        import psycopg2
        self._conn = psycopg2.connect(**self.db_params)
        self._conn.set_client_encoding("UTF8")
        LOGGER.info("DB 연결 성공: %s@%s/%s",
                     self.db_params.get("user", ""),
                     self.db_params.get("host", "localhost"),
                     self.db_params.get("database", ""))

    def close(self) -> None:
        if self._conn:
            self._conn.close()
            self._conn = None

    def ensure_schema(self) -> None:
        """pgvector 확장 및 테이블이 없으면 생성합니다."""
        cur = self._conn.cursor()
        try:
            cur.execute("CREATE EXTENSION IF NOT EXISTS vector;")

            cur.execute(f"""
                CREATE TABLE IF NOT EXISTS {self.TABLE_NAME} (
                    "FEATURE_VECTOR_ID"    SERIAL          PRIMARY KEY,
                    "ROUTE_PATH_GUID"      CHAR(36)        NOT NULL,
                    "PROCESS_NAME"         VARCHAR(50),
                    "EQUIPMENT_NAME"       VARCHAR(100)    NOT NULL,
                    "UTILITY_GROUP"        VARCHAR(50),
                    "UTILITY"              VARCHAR(50),
                    "SIZE"                 VARCHAR(20),
                    "START_POSX"           DOUBLE PRECISION,
                    "START_POSY"           DOUBLE PRECISION,
                    "START_POSZ"           DOUBLE PRECISION,
                    "END_POSX"             DOUBLE PRECISION,
                    "END_POSY"             DOUBLE PRECISION,
                    "END_POSZ"             DOUBLE PRECISION,
                    "DIRECTION_PATTERN"    VARCHAR(200),
                    "TOTAL_LENGTH_MM"      DOUBLE PRECISION,
                    "STEP_COUNT"           INTEGER,
                    "START_LEVEL"          VARCHAR(10),
                    "END_LEVEL"            VARCHAR(10),
                    "FEATURE_VECTOR"       vector({VECTOR_DIM}) NOT NULL,
                    "ENCODER_VERSION"      VARCHAR(20)     DEFAULT 'v1.0',
                    "ENCODED_AT"           TIMESTAMPTZ     DEFAULT NOW(),
                    "RAW_FEATURES_JSON"    JSONB
                );
            """)

            # 필터링용 인덱스
            cur.execute(f"""
                CREATE INDEX IF NOT EXISTS "IDX_FV_PROCESS_EQUIP"
                    ON {self.TABLE_NAME} ("PROCESS_NAME", "EQUIPMENT_NAME");
            """)
            cur.execute(f"""
                CREATE INDEX IF NOT EXISTS "IDX_FV_UTILITY"
                    ON {self.TABLE_NAME} ("UTILITY_GROUP", "UTILITY");
            """)
            cur.execute(f"""
                CREATE INDEX IF NOT EXISTS "IDX_FV_SIZE"
                    ON {self.TABLE_NAME} ("SIZE");
            """)

            self._conn.commit()
            LOGGER.info("스키마 확인/생성 완료")
        except Exception:
            self._conn.rollback()
            raise
        finally:
            cur.close()

    def ensure_hnsw_index(self) -> None:
        """HNSW 코사인 거리 인덱스를 생성합니다 (이미 있으면 무시)."""
        cur = self._conn.cursor()
        try:
            cur.execute(f"""
                CREATE INDEX IF NOT EXISTS "IDX_FEATURE_VECTOR_HNSW"
                    ON {self.TABLE_NAME}
                    USING hnsw ("FEATURE_VECTOR" vector_cosine_ops)
                    WITH (m = 16, ef_construction = 200);
            """)
            self._conn.commit()
            LOGGER.info("HNSW 인덱스 확인/생성 완료")
        except Exception:
            self._conn.rollback()
            raise
        finally:
            cur.close()

    def insert_vector(
        self,
        record: Dict[str, Any],
        vector: np.ndarray,
        encoder_version: str = "v1.0",
    ) -> int:
        """단건 벡터를 저장합니다. 생성된 ID를 반환합니다."""
        cur = self._conn.cursor()
        try:
            vec_str = "[" + ",".join(str(v) for v in vector.tolist()) + "]"
            sp = record.get("start_pos", [0, 0, 0])
            ep = record.get("end_pos", [0, 0, 0])

            cur.execute(f"""
                INSERT INTO {self.TABLE_NAME} (
                    "ROUTE_PATH_GUID", "PROCESS_NAME", "EQUIPMENT_NAME",
                    "UTILITY_GROUP", "UTILITY", "SIZE",
                    "START_POSX", "START_POSY", "START_POSZ",
                    "END_POSX", "END_POSY", "END_POSZ",
                    "DIRECTION_PATTERN", "TOTAL_LENGTH_MM", "STEP_COUNT",
                    "START_LEVEL", "END_LEVEL",
                    "FEATURE_VECTOR", "ENCODER_VERSION", "RAW_FEATURES_JSON"
                ) VALUES (
                    %s, %s, %s, %s, %s, %s,
                    %s, %s, %s, %s, %s, %s,
                    %s, %s, %s, %s, %s,
                    %s::vector, %s, %s::jsonb
                ) RETURNING "FEATURE_VECTOR_ID";
            """, (
                record.get("route_path_guid", record.get("poc_id", "")),
                record.get("equipment_process", record.get("process_name", "")),
                record.get("equipment_name", ""),
                record.get("utility_group", ""),
                record.get("utility", ""),
                record.get("size", ""),
                sp[0] if isinstance(sp, (list, tuple)) else sp.get("x", 0),
                sp[1] if isinstance(sp, (list, tuple)) else sp.get("y", 0),
                sp[2] if isinstance(sp, (list, tuple)) else sp.get("z", 0),
                ep[0] if isinstance(ep, (list, tuple)) else ep.get("x", 0),
                ep[1] if isinstance(ep, (list, tuple)) else ep.get("y", 0),
                ep[2] if isinstance(ep, (list, tuple)) else ep.get("z", 0),
                record.get("path_arrow", record.get("direction_pattern", "")),
                record.get("path_total_length", 0.0),
                record.get("step_count", len(record.get("path_step_vectors", []))),
                record.get("start_level", ""),
                record.get("end_level", ""),
                vec_str,
                encoder_version,
                json.dumps(record.get("raw_features", {}), ensure_ascii=False),
            ))
            row = cur.fetchone()
            self._conn.commit()
            return row[0]
        except Exception:
            self._conn.rollback()
            raise
        finally:
            cur.close()

    def bulk_insert(
        self,
        records: Sequence[Dict[str, Any]],
        vectors: Sequence[np.ndarray],
        encoder_version: str = "v1.0",
        batch_size: int = 500,
    ) -> int:
        """일괄 저장합니다. 저장된 총 건수를 반환합니다."""
        total = 0
        for i in range(0, len(records), batch_size):
            batch_recs = records[i:i + batch_size]
            batch_vecs = vectors[i:i + batch_size]
            for rec, vec in zip(batch_recs, batch_vecs):
                self.insert_vector(rec, vec, encoder_version)
                total += 1
            LOGGER.info("일괄 저장 진행: %d / %d", total, len(records))
        return total

    def search_top_k(
        self,
        query_vector: np.ndarray,
        k: int = 5,
        filters: Optional[Dict[str, str]] = None,
    ) -> TopKResult:
        """pgvector 코사인 거리 기반 Top-K 검색."""
        start_time = time.time()
        cur = self._conn.cursor()
        try:
            vec_str = "[" + ",".join(str(v) for v in query_vector.tolist()) + "]"

            # 동적 필터 조건
            where_clauses = []
            params = [vec_str]

            if filters:
                if filters.get("utility_group"):
                    where_clauses.append('"UTILITY_GROUP" = %s')
                    params.append(filters["utility_group"])
                if filters.get("utility"):
                    where_clauses.append('"UTILITY" = %s')
                    params.append(filters["utility"])
                if filters.get("size"):
                    where_clauses.append('"SIZE" = %s')
                    params.append(filters["size"])
                if filters.get("process_name"):
                    where_clauses.append('"PROCESS_NAME" = %s')
                    params.append(filters["process_name"])

            where_sql = ""
            if where_clauses:
                where_sql = "WHERE " + " AND ".join(where_clauses)

            params.append(k)

            sql = f"""
                SELECT
                    "ROUTE_PATH_GUID",
                    "EQUIPMENT_NAME",
                    "PROCESS_NAME",
                    "UTILITY",
                    "SIZE",
                    "DIRECTION_PATTERN",
                    "TOTAL_LENGTH_MM",
                    "STEP_COUNT",
                    "START_LEVEL",
                    "END_LEVEL",
                    ("FEATURE_VECTOR" <=> %s::vector) AS cosine_dist
                FROM {self.TABLE_NAME}
                {where_sql}
                ORDER BY "FEATURE_VECTOR" <=> %s::vector
                LIMIT %s;
            """

            # 쿼리 벡터를 두 번 바인딩 (SELECT절 + ORDER BY절)
            final_params = [vec_str] + params[1:-1] + [vec_str, k]
            cur.execute(sql, final_params)
            rows = cur.fetchall()

            elapsed_ms = (time.time() - start_time) * 1000.0
            results = []
            for rank_idx, row in enumerate(rows, start=1):
                results.append(SearchResult(
                    rank=rank_idx,
                    route_path_guid=row[0].strip() if row[0] else "",
                    equipment_name=row[1] or "",
                    process_name=row[2] or "",
                    utility=row[3] or "",
                    size=row[4] or "",
                    direction_pattern=row[5] or "",
                    total_length_mm=row[6] or 0.0,
                    step_count=row[7] or 0,
                    start_level=row[8] or "",
                    end_level=row[9] or "",
                    cosine_distance=row[10],
                    similarity_score=1.0 - row[10],
                ))

            return TopKResult(
                query_vector=query_vector,
                results=results,
                search_time_ms=elapsed_ms,
                filters_applied=filters or {},
                total_candidates=len(rows),
            )
        finally:
            cur.close()

    def get_vector(self, route_path_guid: str) -> Optional[np.ndarray]:
        """단건 벡터 조회."""
        cur = self._conn.cursor()
        try:
            cur.execute(
                f'SELECT "FEATURE_VECTOR" FROM {self.TABLE_NAME} WHERE "ROUTE_PATH_GUID" = %s;',
                (route_path_guid,),
            )
            row = cur.fetchone()
            if row and row[0]:
                # pgvector가 반환하는 문자열 "[0.1,0.2,...]" 파싱
                vec_str = row[0]
                if isinstance(vec_str, str):
                    vals = [float(v) for v in vec_str.strip("[]").split(",")]
                    return np.array(vals, dtype=np.float64)
                return np.array(vec_str, dtype=np.float64)
            return None
        finally:
            cur.close()

    def delete_by_route(self, route_path_guid: str) -> int:
        """경로 GUID로 벡터 삭제. 삭제된 건수를 반환합니다."""
        cur = self._conn.cursor()
        try:
            cur.execute(
                f'DELETE FROM {self.TABLE_NAME} WHERE "ROUTE_PATH_GUID" = %s;',
                (route_path_guid,),
            )
            count = cur.rowcount
            self._conn.commit()
            return count
        finally:
            cur.close()

    def get_stats(self) -> Dict[str, Any]:
        """테이블 통계를 반환합니다."""
        cur = self._conn.cursor()
        try:
            cur.execute(f"SELECT COUNT(*) FROM {self.TABLE_NAME};")
            total = cur.fetchone()[0]

            cur.execute(f"""
                SELECT "PROCESS_NAME", COUNT(*)
                FROM {self.TABLE_NAME}
                GROUP BY "PROCESS_NAME"
                ORDER BY COUNT(*) DESC;
            """)
            by_process = {row[0]: row[1] for row in cur.fetchall()}

            return {
                "total_vectors": total,
                "by_process": by_process,
            }
        finally:
            cur.close()


# ─────────────────────────────────────────────────────────────
# 4. TopKSearchEngine - 통합 검색 엔진
# ─────────────────────────────────────────────────────────────


class TopKSearchEngine:
    """Top-K 유사 경로 검색 통합 엔진.

    PathFeatureEncoder로 쿼리 벡터를 생성하고,
    FeatureVectorDB로 pgvector 기반 ANN 검색을 수행합니다.
    """

    def __init__(
        self,
        encoder: PathFeatureEncoder,
        db: Optional[FeatureVectorDB] = None,
    ):
        self.encoder = encoder
        self.db = db

    def search(
        self,
        start_pos: Sequence[float],
        end_pos: Sequence[float],
        utility: str = "",
        size: str = "",
        process_name: str = "",
        utility_group: str = "",
        k: int = 5,
        obstacle_score: float = 0.0,
        support_score: float = 0.0,
        path_step_vectors: Optional[List[Dict[str, float]]] = None,
        path_arrow: str = "",
        path_range: Optional[Dict[str, float]] = None,
        path_total_length: float = 0.0,
    ) -> TopKResult:
        """새 PoC 조건으로 Top-K 유사 경로를 검색합니다.

        path_step_vectors가 제공되면 정밀 인코딩,
        없으면 start/end 위치로 간이 인코딩합니다.
        """
        query_record = self._build_query_record(
            start_pos=start_pos,
            end_pos=end_pos,
            path_step_vectors=path_step_vectors,
            path_arrow=path_arrow,
            path_range=path_range,
            path_total_length=path_total_length,
            obstacle_score=obstacle_score,
            support_score=support_score,
        )

        query_vector = self.encoder.encode(query_record)
        query_vector = self.encoder.normalize_l2(query_vector)

        filters = {}
        if utility_group:
            filters["utility_group"] = utility_group
        if utility:
            filters["utility"] = utility
        if size:
            filters["size"] = size
        if process_name:
            filters["process_name"] = process_name

        if self.db:
            return self.db.search_top_k(query_vector, k=k, filters=filters)

        # DB 없이 로컬 검색 (테스트용)
        return TopKResult(
            query_vector=query_vector,
            results=[],
            search_time_ms=0.0,
            filters_applied=filters,
        )

    def search_by_record(
        self,
        query_record: Dict[str, Any],
        k: int = 5,
        filters: Optional[Dict[str, str]] = None,
    ) -> TopKResult:
        """경로 레코드로 직접 검색합니다."""
        query_vector = self.encoder.encode(query_record)
        query_vector = self.encoder.normalize_l2(query_vector)

        if self.db:
            return self.db.search_top_k(query_vector, k=k, filters=filters)

        return TopKResult(
            query_vector=query_vector,
            results=[],
            search_time_ms=0.0,
            filters_applied=filters or {},
        )

    def search_local(
        self,
        query_record: Dict[str, Any],
        candidate_records: Sequence[Dict[str, Any]],
        k: int = 5,
    ) -> TopKResult:
        """DB 없이 로컬 메모리에서 Top-K 검색합니다 (테스트/평가용)."""
        start_time = time.time()

        query_vector = self.encoder.encode(query_record)
        query_vector = self.encoder.normalize_l2(query_vector)

        # 모든 후보 인코딩 + 코사인 거리 계산
        scored = []
        for idx, rec in enumerate(candidate_records):
            cand_vector = self.encoder.encode(rec)
            cand_vector = self.encoder.normalize_l2(cand_vector)

            cos_dist = 1.0 - float(np.dot(query_vector, cand_vector))
            cos_dist = max(0.0, cos_dist)  # 부동소수점 오차 보정
            scored.append((cos_dist, idx, rec, cand_vector))

        scored.sort(key=lambda x: x[0])
        top_k = scored[:k]

        elapsed_ms = (time.time() - start_time) * 1000.0
        results = []
        for rank_idx, (cos_dist, idx, rec, cand_vec) in enumerate(top_k, start=1):
            results.append(SearchResult(
                rank=rank_idx,
                route_path_guid=rec.get("route_path_guid", rec.get("poc_id", f"local_{idx}")),
                equipment_name=rec.get("equipment_name", ""),
                process_name=rec.get("equipment_process", rec.get("process_name", "")),
                utility=rec.get("utility", ""),
                size=rec.get("size", ""),
                direction_pattern=rec.get("path_arrow", ""),
                total_length_mm=rec.get("path_total_length", 0.0),
                step_count=len(rec.get("path_step_vectors", [])),
                start_level=rec.get("start_level", ""),
                end_level=rec.get("end_level", ""),
                cosine_distance=cos_dist,
                similarity_score=1.0 - cos_dist,
                feature_vector=cand_vec,
            ))

        return TopKResult(
            query_vector=query_vector,
            results=results,
            search_time_ms=elapsed_ms,
            filters_applied={},
            total_candidates=len(candidate_records),
        )

    def _build_query_record(
        self,
        start_pos: Sequence[float],
        end_pos: Sequence[float],
        path_step_vectors: Optional[List[Dict[str, float]]] = None,
        path_arrow: str = "",
        path_range: Optional[Dict[str, float]] = None,
        path_total_length: float = 0.0,
        obstacle_score: float = 0.0,
        support_score: float = 0.0,
    ) -> Dict[str, Any]:
        """검색 조건을 경로 레코드 형태로 구성합니다."""
        sp = list(start_pos)
        ep = list(end_pos)

        # path_step_vectors가 없으면 start→end 단일 벡터로 간이 생성
        if not path_step_vectors:
            path_step_vectors = [{
                "x": ep[0] - sp[0],
                "y": ep[1] - sp[1],
                "z": ep[2] - sp[2],
            }]

        # path_range가 없으면 start/end에서 추정
        if not path_range:
            path_range = {
                "x": abs(ep[0] - sp[0]),
                "y": abs(ep[1] - sp[1]),
                "z": abs(ep[2] - sp[2]),
            }

        # path_total_length가 없으면 직선 거리로 추정
        if path_total_length <= 0:
            path_total_length = math.sqrt(sum((a - b) ** 2 for a, b in zip(sp, ep)))

        return {
            "start_pos": sp,
            "end_pos": ep,
            "path_step_vectors": path_step_vectors,
            "path_arrow": path_arrow,
            "path_range": path_range,
            "path_total_length": path_total_length,
            "obstacle_score": obstacle_score,
            "support_score": support_score,
        }


# ─────────────────────────────────────────────────────────────
# 5. 유틸리티 - GroupPipeResults 로딩
# ─────────────────────────────────────────────────────────────


def load_group_pipe_records(group_results_path: str) -> List[Dict[str, Any]]:
    """GroupPipeResults JSON에서 개별 경로 레코드를 로드합니다.

    그룹 레벨 메타데이터(equipment_name, utility, size 등)를
    각 경로 레코드에 병합합니다.
    """
    with open(group_results_path, "r", encoding="utf-8") as f:
        groups = json.load(f)

    records = []
    for group in groups:
        group_meta = {
            "equipment_name": group.get("equipment_name", ""),
            "equipment_process": group.get("equipment_process", ""),
            "equipment_id": group.get("equipment_id", ""),
            "utility": group.get("utility", ""),
            "size": group.get("size", ""),
            "group_id": group.get("group_id", 0),
        }

        for path in group.get("paths", []):
            record = {**group_meta, **path}
            records.append(record)

    LOGGER.info("GroupPipeResults 로드: %d개 경로 (%s)", len(records), group_results_path)
    return records


# ─────────────────────────────────────────────────────────────
# 6. CLI
# ─────────────────────────────────────────────────────────────


def _run_local_test(args: argparse.Namespace) -> None:
    """DB 없이 로컬에서 Top-K 검색을 테스트합니다."""
    records = load_group_pipe_records(args.group_results)
    if not records:
        print("경로 레코드가 없습니다.")
        return

    # 데이터셋에서 정규화 파라미터 산출
    norm_params = NormalizationParams.from_dataset(records)
    encoder = PathFeatureEncoder(norm_params=norm_params)
    engine = TopKSearchEngine(encoder=encoder)

    # 첫 번째 경로를 쿼리로 사용
    query = records[0]
    print(f"\n=== Top-K 로컬 검색 테스트 ===")
    print(f"쿼리 경로: {query.get('equipment_name', '')} / "
          f"{query.get('utility', '')} / {query.get('size', '')}")
    print(f"패턴: {query.get('path_arrow', '')}")
    print(f"총 후보: {len(records)}개")

    # 쿼리 벡터 생성 및 해석
    query_vec = encoder.encode(query)
    print(f"\n쿼리 벡터 (30D): {np.round(query_vec, 4).tolist()}")
    explain = encoder.decode_explain(query_vec)
    print(f"  시작 위상: {[round(v, 3) for v in explain['start_topology']]}")
    print(f"  종단 위상: {[round(v, 3) for v in explain['end_topology']]}")
    print(f"  공간 변위: {[round(v, 3) for v in explain['displacement']]}")
    print(f"  바운딩 박스: {[round(v, 3) for v in explain['bounding_box']]}")

    result = engine.search_local(query, records, k=args.k)

    print(f"\n검색 시간: {result.search_time_ms:.1f}ms")
    print(f"\n{'#':>3} {'유사도':>8} {'거리':>8} {'장비':<15} {'패턴':<25} {'길이(mm)':>10}")
    print("-" * 75)
    for sr in result.results:
        print(f"{sr.rank:>3} {sr.similarity_score:>8.4f} {sr.cosine_distance:>8.4f} "
              f"{sr.equipment_name:<15} {sr.direction_pattern:<25} {sr.total_length_mm:>10.1f}")


def main():
    logging.basicConfig(level=logging.INFO, format="%(name)s | %(levelname)s | %(message)s")

    parser = argparse.ArgumentParser(description="Top-K 유사 배관 경로 검색")
    subparsers = parser.add_subparsers(dest="command")

    # 로컬 테스트
    test_parser = subparsers.add_parser("test", help="로컬 Top-K 검색 테스트")
    test_parser.add_argument("--group_results", required=True, help="GroupPipeResults JSON 경로")
    test_parser.add_argument("--k", type=int, default=5, help="Top-K 개수 (기본: 5)")

    # DB 검색
    search_parser = subparsers.add_parser("search", help="DB 기반 Top-K 검색")
    search_parser.add_argument("--start", required=True, help="시작 좌표 (x,y,z)")
    search_parser.add_argument("--end", required=True, help="종단 좌표 (x,y,z)")
    search_parser.add_argument("--utility", default="", help="유틸리티")
    search_parser.add_argument("--size", default="", help="사이즈")
    search_parser.add_argument("--process", default="", help="공정명")
    search_parser.add_argument("--k", type=int, default=5, help="Top-K 개수")
    search_parser.add_argument("--dbname", required=True, help="DB명")
    search_parser.add_argument("--user", required=True, help="DB 사용자")
    search_parser.add_argument("--password", required=True, help="DB 비밀번호")
    search_parser.add_argument("--host", default="localhost", help="DB 호스트")
    search_parser.add_argument("--port", default="5432", help="DB 포트")
    search_parser.add_argument("--norm_params", default=None, help="정규화 파라미터 JSON")

    args = parser.parse_args()

    if args.command == "test":
        _run_local_test(args)
    elif args.command == "search":
        sp = [float(v) for v in args.start.split(",")]
        ep = [float(v) for v in args.end.split(",")]

        norm_params = NormalizationParams()
        if args.norm_params:
            norm_params = NormalizationParams.load(args.norm_params)

        encoder = PathFeatureEncoder(norm_params=norm_params)
        db = FeatureVectorDB({
            "host": args.host, "database": args.dbname,
            "user": args.user, "password": args.password, "port": args.port,
        })
        db.connect()

        try:
            engine = TopKSearchEngine(encoder=encoder, db=db)
            result = engine.search(
                start_pos=sp, end_pos=ep,
                utility=args.utility, size=args.size,
                process_name=args.process, k=args.k,
            )

            print(f"\n=== Top-{args.k} 검색 결과 ===")
            print(f"검색 시간: {result.search_time_ms:.1f}ms")
            print(f"\n{'#':>3} {'유사도':>8} {'장비':<15} {'패턴':<25} {'길이(mm)':>10}")
            print("-" * 65)
            for sr in result.results:
                print(f"{sr.rank:>3} {sr.similarity_score:>8.4f} "
                      f"{sr.equipment_name:<15} {sr.direction_pattern:<25} {sr.total_length_mm:>10.1f}")
        finally:
            db.close()
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
