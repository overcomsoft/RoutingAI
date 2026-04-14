from __future__ import annotations

# =====================================================================================
# [실행 명령어]
#
#   (1) DB 경로 → DB 벡터 저장 (TB_ROUTE_PATH → TB_ROUTE_FEATURE_VECTOR)
#   python BuildFeatureVectors.py from-db \
#       --dbname AUTOROUTINGV7 --user postgres --password dinno \
#       --host localhost --port 5432 \
#       --save_norm ../data/FeatureVectors/db_norm_params.json \
#       --save_json ../data/FeatureVectors/db_feature_vectors.json
#
#   (2) 로컬 벡터 생성 + JSON 저장 (GroupPipeResults JSON 소스)
#   python BuildFeatureVectors.py local \
#       --group_results ../data/GroupPipeResults/group_pipe_results_20260405194814.json \
#       --output ../data/FeatureVectors/feature_vectors.json
#
#   (3) JSON 소스 → DB 벡터 저장
#   python BuildFeatureVectors.py db \
#       --group_results ../data/GroupPipeResults/group_pipe_results_20260405194814.json \
#       --dbname AUTOROUTINGV7 --user postgres --password dinno
#
#   (4) 정규화 파라미터만 산출
#   python BuildFeatureVectors.py norm \
#       --group_results ../data/GroupPipeResults/group_pipe_results_20260405194814.json \
#       --output ../data/FeatureVectors/normalization_params.json
#
# =====================================================================================
"""
BuildFeatureVectors.py  — 기존 경로 일괄 벡터화 배치 스크립트
============================================================

[프로그램 개요]
  GroupPipeResults JSON에서 모든 경로 레코드를 로드하여
  PathFeatureEncoder로 30D 특징벡터를 생성하고,
  로컬 JSON 파일 또는 PostgreSQL DB에 저장합니다.

[처리 흐름]
  [1] GroupPipeResults JSON 로드
  [2] NormalizationParams.from_dataset() → 정규화 파라미터 산출
  [3] PathFeatureEncoder.encode() → 30D 벡터 생성
  [4] 저장 (JSON 또는 DB INSERT)
"""

import argparse
import json
import logging
import os
import time
from datetime import datetime
from typing import Any, Dict, List

import numpy as np

from TopKRoutingSearch import (
    FeatureVectorDB,
    NormalizationParams,
    PathFeatureEncoder,
    RoutePathLoader,
    load_group_pipe_records,
)

LOGGER = logging.getLogger("BuildFeatureVectors")


def build_vectors(
    records: List[Dict[str, Any]],
    norm_params: NormalizationParams,
) -> List[np.ndarray]:
    """전체 경로 레코드를 30D 벡터로 인코딩합니다."""
    encoder = PathFeatureEncoder(norm_params=norm_params)
    vectors = []
    for rec in records:
        vec = encoder.encode(rec)
        vec = encoder.normalize_l2(vec)
        vectors.append(vec)
    return vectors


def save_vectors_json(
    records: List[Dict[str, Any]],
    vectors: List[np.ndarray],
    norm_params: NormalizationParams,
    output_path: str,
) -> None:
    """벡터를 JSON 파일로 저장합니다."""
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    encoder = PathFeatureEncoder(norm_params=norm_params)
    entries = []
    for rec, vec in zip(records, vectors):
        entries.append({
            "route_path_guid": rec.get("route_path_guid", rec.get("poc_id", "")),
            "equipment_name": rec.get("equipment_name", ""),
            "equipment_process": rec.get("equipment_process", ""),
            "utility": rec.get("utility", ""),
            "size": rec.get("size", ""),
            "group_id": rec.get("group_id", 0),
            "path_arrow": rec.get("path_arrow", ""),
            "path_total_length": rec.get("path_total_length", 0.0),
            "start_pos": rec.get("start_pos", [0, 0, 0]),
            "end_pos": rec.get("end_pos", [0, 0, 0]),
            "feature_vector": vec.tolist(),
            "feature_explain": encoder.decode_explain(vec),
        })

    output = {
        "created_at": datetime.now().isoformat(),
        "encoder_version": "v1.0",
        "vector_dim": 30,
        "total_records": len(entries),
        "normalization_params": {
            "bbox_max": norm_params.bbox_max,
            "displacement_max": norm_params.displacement_max,
            "total_length_max": norm_params.total_length_max,
            "bend_count_max": norm_params.bend_count_max,
        },
        "vectors": entries,
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)

    LOGGER.info("벡터 JSON 저장 완료: %s (%d건)", output_path, len(entries))


def save_vectors_db(
    records: List[Dict[str, Any]],
    vectors: List[np.ndarray],
    db_params: Dict[str, str],
) -> None:
    """벡터를 PostgreSQL DB에 저장합니다."""
    db = FeatureVectorDB(db_params)
    db.connect()
    try:
        db.ensure_schema()
        total = db.bulk_insert(records, vectors, encoder_version="v1.0")
        LOGGER.info("DB 저장 완료: %d건", total)

        db.ensure_hnsw_index()
        stats = db.get_stats()
        LOGGER.info("DB 통계: %s", stats)
    finally:
        db.close()


def main():
    logging.basicConfig(level=logging.INFO, format="%(name)s | %(levelname)s | %(message)s")

    parser = argparse.ArgumentParser(description="기존 경로 일괄 벡터화")
    subparsers = parser.add_subparsers(dest="command")

    # 로컬 JSON 저장
    local_parser = subparsers.add_parser("local", help="로컬 JSON으로 벡터 저장")
    local_parser.add_argument("--group_results", required=True, help="GroupPipeResults JSON 경로")
    local_parser.add_argument("--output", required=True, help="출력 JSON 경로")
    local_parser.add_argument("--norm_params", default=None, help="기존 정규화 파라미터 JSON (없으면 자동 산출)")

    # DB 저장 (JSON 소스)
    db_parser = subparsers.add_parser("db", help="JSON → DB 벡터 저장")
    db_parser.add_argument("--group_results", required=True, help="GroupPipeResults JSON 경로")
    db_parser.add_argument("--norm_params", default=None, help="기존 정규화 파라미터 JSON")
    db_parser.add_argument("--dbname", required=True, help="DB명")
    db_parser.add_argument("--user", required=True, help="DB 사용자")
    db_parser.add_argument("--password", required=True, help="DB 비밀번호")
    db_parser.add_argument("--host", default="localhost", help="DB 호스트")
    db_parser.add_argument("--port", default="5432", help="DB 포트")

    # ★ DB → DB (경로 테이블에서 읽어 벡터 테이블에 저장)
    fromdb_parser = subparsers.add_parser("from-db", help="DB 경로 → DB 벡터 저장 (TB_ROUTE_PATH → TB_ROUTE_FEATURE_VECTOR)")
    fromdb_parser.add_argument("--dbname", required=True, help="DB명")
    fromdb_parser.add_argument("--user", required=True, help="DB 사용자")
    fromdb_parser.add_argument("--password", required=True, help="DB 비밀번호")
    fromdb_parser.add_argument("--host", default="localhost", help="DB 호스트")
    fromdb_parser.add_argument("--port", default="5432", help="DB 포트")
    fromdb_parser.add_argument("--norm_params", default=None, help="기존 정규화 파라미터 JSON (없으면 자동 산출)")
    fromdb_parser.add_argument("--save_norm", default=None, help="정규화 파라미터 저장 경로")
    fromdb_parser.add_argument("--save_json", default=None, help="벡터 JSON도 함께 저장할 경로 (선택)")

    # 정규화 파라미터 산출만
    norm_parser = subparsers.add_parser("norm", help="정규화 파라미터만 산출")
    norm_parser.add_argument("--group_results", required=True, help="GroupPipeResults JSON 경로")
    norm_parser.add_argument("--output", required=True, help="출력 JSON 경로")

    args = parser.parse_args()

    if args.command == "local":
        records = load_group_pipe_records(args.group_results)
        if not records:
            print("경로 레코드가 없습니다.")
            return

        if args.norm_params:
            norm_params = NormalizationParams.load(args.norm_params)
        else:
            norm_params = NormalizationParams.from_dataset(records)

        start_time = time.time()
        vectors = build_vectors(records, norm_params)
        elapsed = time.time() - start_time

        save_vectors_json(records, vectors, norm_params, args.output)

        # 정규화 파라미터도 함께 저장
        norm_path = os.path.splitext(args.output)[0] + "_norm_params.json"
        norm_params.save(norm_path)

        print(f"\n=== 벡터 생성 완료 ===")
        print(f"총 경로: {len(records)}개")
        print(f"인코딩 시간: {elapsed:.2f}초 ({elapsed * 1000 / max(len(records), 1):.1f}ms/건)")
        print(f"출력 파일: {args.output}")
        print(f"정규화 파라미터: {norm_path}")

    elif args.command == "db":
        records = load_group_pipe_records(args.group_results)
        if not records:
            print("경로 레코드가 없습니다.")
            return

        if args.norm_params:
            norm_params = NormalizationParams.load(args.norm_params)
        else:
            norm_params = NormalizationParams.from_dataset(records)

        start_time = time.time()
        vectors = build_vectors(records, norm_params)
        elapsed_encode = time.time() - start_time

        db_params = {
            "host": args.host, "database": args.dbname,
            "user": args.user, "password": args.password, "port": args.port,
        }

        start_time = time.time()
        save_vectors_db(records, vectors, db_params)
        elapsed_db = time.time() - start_time

        print(f"\n=== DB 저장 완료 ===")
        print(f"총 경로: {len(records)}개")
        print(f"인코딩 시간: {elapsed_encode:.2f}초")
        print(f"DB 저장 시간: {elapsed_db:.2f}초")

    elif args.command == "from-db":
        db_params = {
            "host": args.host, "database": args.dbname,
            "user": args.user, "password": args.password, "port": args.port,
        }

        # 1. DB에서 경로 로드
        loader = RoutePathLoader(db_params)
        loader.connect()
        try:
            start_time = time.time()
            records = loader.load_all_paths()
            elapsed_load = time.time() - start_time
        finally:
            loader.close()

        if not records:
            print("DB에서 로드된 경로가 없습니다.")
            return

        print(f"\n=== DB 경로 로드 완료 ===")
        print(f"총 경로: {len(records)}개")
        print(f"로드 시간: {elapsed_load:.2f}초")

        # 2. 정규화 파라미터
        if args.norm_params:
            norm_params = NormalizationParams.load(args.norm_params)
        else:
            norm_params = NormalizationParams.from_dataset(records)

        if args.save_norm:
            os.makedirs(os.path.dirname(args.save_norm) or ".", exist_ok=True)
            norm_params.save(args.save_norm)
            print(f"정규화 파라미터 저장: {args.save_norm}")

        # 3. 벡터 인코딩
        start_time = time.time()
        vectors = build_vectors(records, norm_params)
        elapsed_encode = time.time() - start_time

        # 4. 벡터 JSON 저장 (선택)
        if args.save_json:
            save_vectors_json(records, vectors, norm_params, args.save_json)

        # 5. TB_ROUTE_FEATURE_VECTOR에 저장
        start_time = time.time()
        save_vectors_db(records, vectors, db_params)
        elapsed_db = time.time() - start_time

        print(f"\n=== DB → DB 벡터화 완료 ===")
        print(f"총 경로: {len(records)}개")
        print(f"DB 로드: {elapsed_load:.2f}초")
        print(f"인코딩: {elapsed_encode:.2f}초 ({elapsed_encode * 1000 / max(len(records), 1):.1f}ms/건)")
        print(f"DB 저장: {elapsed_db:.2f}초")
        print(f"정규화: bbox_max={norm_params.bbox_max}")
        print(f"         displacement_max={norm_params.displacement_max:.1f}")
        print(f"         total_length_max={norm_params.total_length_max:.1f}")

    elif args.command == "norm":
        records = load_group_pipe_records(args.group_results)
        if not records:
            print("경로 레코드가 없습니다.")
            return

        norm_params = NormalizationParams.from_dataset(records)
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        norm_params.save(args.output)

        print(f"\n=== 정규화 파라미터 산출 완료 ===")
        print(f"총 경로: {len(records)}개")
        print(f"bbox_max: {norm_params.bbox_max}")
        print(f"displacement_max: {norm_params.displacement_max:.1f}")
        print(f"total_length_max: {norm_params.total_length_max:.1f}")
        print(f"bend_count_max: {norm_params.bend_count_max:.1f}")
        print(f"출력 파일: {args.output}")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
