from __future__ import annotations

# =====================================================================================
# [실행 명령어]
#
#   (1) 로컬 평가 (DB 없이 GroupPipeResults JSON만으로 평가)
#   python EvaluateTopK.py local \
#       --group_results ../data/GroupPipeResults/group_pipe_results_20260405194814.json \
#       --output ../data/Evaluation/evaluation_report.json \
#       --n_queries 100 --k 5 --seed 42
#
#   (2) DB 기반 평가 (pgvector 검색 vs 복합유사도 Ground Truth)
#   python EvaluateTopK.py db \
#       --group_results ../data/GroupPipeResults/group_pipe_results_20260405194814.json \
#       --output ../data/Evaluation/evaluation_report.json \
#       --n_queries 100 --k 5 --seed 42 \
#       --dbname AUTOROUTINGV7 --user postgres --password dinno \
#       --host localhost --port 5432 \
#       --norm_params ../data/FeatureVectors/db_norm_params.json
#
# =====================================================================================
"""
EvaluateTopK.py  — Top-K 유사배관 검색 품질 평가 스크립트
========================================================

[프로그램 개요]
  기존 복합유사도(compute_composite_similarity)를 Ground Truth로 삼고,
  30D 벡터 기반 Top-K 검색 결과와 비교하여 품질 지표를 산출합니다.

[평가 지표]
  - Recall@K   : GT Top-K 중 벡터 Top-K에 포함된 비율 (목표 >= 0.80)
  - NDCG@K     : 순위 가중 정밀도 (목표 >= 0.85)
  - Spearman ρ : 코사인 거리 vs 복합유사도 순위 상관계수 (목표 >= 0.80)
  - MRR        : Mean Reciprocal Rank (GT 1위가 벡터 결과에서 몇 위인지)

[처리 흐름]
  [1] GroupPipeResults JSON 로드 → 전체 경로 레코드
  [2] 무작위 N개 쿼리 샘플링
  [3] 각 쿼리에 대해:
      (a) compute_composite_similarity로 전수 비교 → GT Top-K
      (b) 30D 벡터 기반 Top-K 검색 (로컬 또는 DB)
      (c) Recall, NDCG, Spearman rho, MRR 계산
  [4] 전체 평균 및 분포 통계 → evaluation_report.json 출력
"""

import argparse
import json
import logging
import math
import os
import random
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# 동일 디렉토리의 모듈 import
from TopKRoutingSearch import (
    FeatureVectorDB,
    NormalizationParams,
    PathFeatureEncoder,
    TopKSearchEngine,
    load_group_pipe_records,
)

# AnalyzeRoutingAi_V2의 복합유사도 (Ground Truth 생성용)
from AnalyzeRoutingAi_V2 import compute_composite_similarity

LOGGER = logging.getLogger("EvaluateTopK")


# ─────────────────────────────────────────────────────────────
# 1. 평가 지표 계산 함수
# ─────────────────────────────────────────────────────────────


def recall_at_k(gt_ids: List[str], pred_ids: List[str]) -> float:
    """Recall@K: GT Top-K 중 예측 Top-K에 포함된 비율."""
    if not gt_ids:
        return 1.0
    gt_set = set(gt_ids)
    pred_set = set(pred_ids)
    return len(gt_set & pred_set) / len(gt_set)


def ndcg_at_k(gt_ids: List[str], pred_ids: List[str]) -> float:
    """NDCG@K: Normalized Discounted Cumulative Gain.

    GT 순위를 relevance score로 사용합니다.
    GT 1위 = K점, 2위 = K-1점, ... K위 = 1점.
    GT에 없는 항목 = 0점.
    """
    k = len(gt_ids)
    if k == 0:
        return 1.0

    # GT 순위 → relevance score 매핑
    relevance = {}
    for rank, gid in enumerate(gt_ids):
        relevance[gid] = k - rank  # 1위=K점, K위=1점

    # DCG 계산 (pred 순서에서)
    dcg = 0.0
    for i, pid in enumerate(pred_ids):
        rel = relevance.get(pid, 0.0)
        dcg += rel / math.log2(i + 2)  # i+2 because log2(1)=0

    # Ideal DCG (GT 순서 그대로)
    idcg = 0.0
    for i in range(k):
        rel = k - i
        idcg += rel / math.log2(i + 2)

    if idcg == 0:
        return 0.0
    return dcg / idcg


def spearman_rho(ranks_a: List[float], ranks_b: List[float]) -> float:
    """Spearman 순위 상관계수.

    ranks_a, ranks_b는 동일 항목 집합에 대한 순위(1부터 시작).
    """
    n = len(ranks_a)
    if n < 2:
        return 1.0

    d_sq_sum = sum((a - b) ** 2 for a, b in zip(ranks_a, ranks_b))
    rho = 1.0 - (6.0 * d_sq_sum) / (n * (n * n - 1))
    return max(-1.0, min(1.0, rho))


def mean_reciprocal_rank(gt_top1_id: str, pred_ids: List[str]) -> float:
    """MRR: GT 1위 항목이 예측 결과에서 몇 위에 있는지의 역수."""
    for i, pid in enumerate(pred_ids):
        if pid == gt_top1_id:
            return 1.0 / (i + 1)
    return 0.0


# ─────────────────────────────────────────────────────────────
# 2. Ground Truth 생성 (복합유사도 전수비교)
# ─────────────────────────────────────────────────────────────


def _get_record_id(record: Dict[str, Any]) -> str:
    """레코드의 고유 ID를 반환합니다."""
    return record.get("route_path_guid", record.get("poc_id", ""))


def compute_ground_truth(
    query: Dict[str, Any],
    candidates: List[Dict[str, Any]],
    k: int = 5,
) -> List[Tuple[str, float]]:
    """복합유사도 전수비교로 Ground Truth Top-K를 생성합니다.

    Returns
    -------
    list of (record_id, similarity_score) 튜플, 유사도 내림차순 정렬.
    """
    query_id = _get_record_id(query)
    scored = []

    for cand in candidates:
        cand_id = _get_record_id(cand)
        if cand_id == query_id:
            continue  # 자기 자신 제외
        sim = compute_composite_similarity(query, cand)
        scored.append((cand_id, sim))

    scored.sort(key=lambda x: -x[1])
    return scored[:k]


# ─────────────────────────────────────────────────────────────
# 3. 단일 쿼리 평가
# ─────────────────────────────────────────────────────────────


def evaluate_single_query(
    query: Dict[str, Any],
    all_records: List[Dict[str, Any]],
    engine: TopKSearchEngine,
    k: int = 5,
    all_scored: Optional[List[Tuple[str, float]]] = None,
) -> Dict[str, Any]:
    """단일 쿼리에 대해 GT vs 벡터 검색 비교 평가를 수행합니다.

    Parameters
    ----------
    query : 쿼리 경로 레코드
    all_records : 전체 후보 경로 레코드
    engine : TopKSearchEngine 인스턴스
    k : Top-K 개수
    all_scored : 미리 계산된 복합유사도 전수 결과 (최적화용, 없으면 내부 계산)

    Returns
    -------
    dict : 평가 결과 (recall, ndcg, spearman, mrr 등)
    """
    query_id = _get_record_id(query)

    # (a) Ground Truth 생성
    if all_scored is not None:
        gt_top_k = all_scored[:k]
    else:
        gt_top_k = compute_ground_truth(query, all_records, k=k)

    gt_ids = [gid for gid, _ in gt_top_k]

    # (b) 벡터 기반 전수 검색 (전체 순위 + Top-K 추출)
    vec_result = engine.search_local(query, all_records, k=len(all_records))

    # 자기 자신 제외
    vec_results_filtered = [
        sr for sr in vec_result.results
        if sr.route_path_guid != query_id
    ]
    vec_ids = [sr.route_path_guid for sr in vec_results_filtered[:k]]

    # (c) 평가 지표 계산
    r_at_k = recall_at_k(gt_ids, vec_ids)
    n_at_k = ndcg_at_k(gt_ids, vec_ids)

    # Spearman rho: GT 전체 순위 vs 벡터 코사인 거리 전체 순위의 상대적 순서 비교
    if all_scored is not None and len(all_scored) >= 2:
        # 벡터 전체 순위 맵 (코사인 거리 오름차순)
        vec_rank_map = {sr.route_path_guid: rank + 1
                        for rank, sr in enumerate(vec_results_filtered)}

        # GT 상위 N개 항목을 선택하고, 이 항목들의 벡터 순위를 조회
        compare_n = min(len(all_scored), k * 4)
        compare_ids = [gid for gid, _ in all_scored[:compare_n]]

        # 이 항목들의 벡터 순위를 가져와서 상대 순위로 재변환
        vec_abs_ranks = [vec_rank_map.get(cid, len(all_records)) for cid in compare_ids]

        # 상대 순위 변환: 절대 순위를 1~N 상대 순위로 변환
        # GT는 이미 정렬됨 → 상대 순위 = [1, 2, 3, ..., N]
        gt_relative = list(range(1, compare_n + 1))

        # 벡터 절대 순위를 상대 순위로 변환 (작은 값일수록 높은 순위)
        indexed_vec = sorted(range(compare_n), key=lambda i: vec_abs_ranks[i])
        vec_relative = [0] * compare_n
        for relative_rank, orig_idx in enumerate(indexed_vec):
            vec_relative[orig_idx] = relative_rank + 1

        sp_rho = spearman_rho(gt_relative, vec_relative)
    else:
        sp_rho = 0.0

    # MRR: GT 1위가 벡터 결과에서 몇 위인지
    mrr = mean_reciprocal_rank(gt_ids[0], vec_ids) if gt_ids else 0.0

    return {
        "query_id": query_id,
        "equipment_name": query.get("equipment_name", ""),
        "utility": query.get("utility", ""),
        "size": query.get("size", ""),
        "path_arrow": query.get("path_arrow", ""),
        "recall_at_k": round(r_at_k, 4),
        "ndcg_at_k": round(n_at_k, 4),
        "spearman_rho": round(sp_rho, 4),
        "mrr": round(mrr, 4),
        "gt_top_k": [{"id": gid, "similarity": round(sim, 6)} for gid, sim in gt_top_k],
        "vec_top_k": [
            {
                "id": sr.route_path_guid,
                "cosine_distance": round(sr.cosine_distance, 6),
                "similarity": round(sr.similarity_score, 6),
            }
            for sr in vec_results_filtered[:k]
        ],
        "search_time_ms": round(vec_result.search_time_ms, 2),
    }


# ─────────────────────────────────────────────────────────────
# 4. DB 기반 단일 쿼리 평가
# ─────────────────────────────────────────────────────────────


def evaluate_single_query_db(
    query: Dict[str, Any],
    all_records: List[Dict[str, Any]],
    engine: TopKSearchEngine,
    k: int = 5,
) -> Dict[str, Any]:
    """DB 기반 Top-K 검색으로 단일 쿼리를 평가합니다."""
    query_id = _get_record_id(query)

    # (a) Ground Truth 생성 (복합유사도 전수비교)
    gt_all = compute_ground_truth(query, all_records, k=len(all_records))
    gt_top_k = gt_all[:k]
    gt_ids = [gid for gid, _ in gt_top_k]

    # (b) DB pgvector Top-K 검색
    vec_result = engine.search_by_record(query, k=k + 5)  # 여유분 확보

    # 자기 자신 제외
    vec_results_filtered = [
        sr for sr in vec_result.results
        if sr.route_path_guid.strip() != query_id.strip()
    ][:k]
    vec_ids = [sr.route_path_guid.strip() for sr in vec_results_filtered]

    # (c) 평가 지표 계산
    r_at_k = recall_at_k(gt_ids, vec_ids)
    n_at_k = ndcg_at_k(gt_ids, vec_ids)

    # Spearman rho (DB 모드에서는 Top-K 교집합의 상대 순위로 비교)
    common_ids_list = [cid for cid in gt_ids if cid in set(vec_ids)]
    if len(common_ids_list) >= 2:
        gt_order = {gid: i for i, gid in enumerate(gt_ids)}
        vec_order = {vid: i for i, vid in enumerate(vec_ids)}
        gt_ranks = [gt_order[cid] + 1 for cid in common_ids_list]
        vec_ranks = [vec_order[cid] + 1 for cid in common_ids_list]
        sp_rho = spearman_rho(gt_ranks, vec_ranks)
    else:
        sp_rho = 0.0

    mrr = mean_reciprocal_rank(gt_ids[0], vec_ids) if gt_ids else 0.0

    return {
        "query_id": query_id,
        "equipment_name": query.get("equipment_name", ""),
        "utility": query.get("utility", ""),
        "size": query.get("size", ""),
        "path_arrow": query.get("path_arrow", ""),
        "recall_at_k": round(r_at_k, 4),
        "ndcg_at_k": round(n_at_k, 4),
        "spearman_rho": round(sp_rho, 4),
        "mrr": round(mrr, 4),
        "gt_top_k": [{"id": gid, "similarity": round(sim, 6)} for gid, sim in gt_top_k],
        "vec_top_k": [
            {
                "id": sr.route_path_guid.strip(),
                "cosine_distance": round(sr.cosine_distance, 6),
                "similarity": round(sr.similarity_score, 6),
            }
            for sr in vec_results_filtered
        ],
        "search_time_ms": round(vec_result.search_time_ms, 2),
    }


# ─────────────────────────────────────────────────────────────
# 5. 전체 평가 실행 + 리포트 생성
# ─────────────────────────────────────────────────────────────


def _percentile(values: List[float], p: float) -> float:
    """p-th 퍼센타일 (0~1)."""
    if not values:
        return 0.0
    sorted_vals = sorted(values)
    idx = min(int(len(sorted_vals) * p), len(sorted_vals) - 1)
    return sorted_vals[idx]


def _stats(values: List[float]) -> Dict[str, float]:
    """기본 통계량."""
    if not values:
        return {"mean": 0.0, "median": 0.0, "p5": 0.0, "p95": 0.0, "min": 0.0, "max": 0.0}
    return {
        "mean": round(sum(values) / len(values), 4),
        "median": round(_percentile(values, 0.5), 4),
        "p5": round(_percentile(values, 0.05), 4),
        "p95": round(_percentile(values, 0.95), 4),
        "min": round(min(values), 4),
        "max": round(max(values), 4),
    }


def run_evaluation_local(
    records: List[Dict[str, Any]],
    n_queries: int = 100,
    k: int = 5,
    seed: int = 42,
) -> Dict[str, Any]:
    """로컬 모드 전체 평가를 실행합니다."""
    rng = random.Random(seed)

    # 쿼리 샘플링
    n_queries = min(n_queries, len(records))
    query_indices = rng.sample(range(len(records)), n_queries)

    # 정규화 파라미터 및 인코더 초기화
    norm_params = NormalizationParams.from_dataset(records)
    encoder = PathFeatureEncoder(norm_params=norm_params)
    engine = TopKSearchEngine(encoder=encoder)

    LOGGER.info("평가 시작: %d개 쿼리, K=%d, 후보=%d건", n_queries, k, len(records))

    results = []
    total_start = time.time()

    for progress, qi in enumerate(query_indices):
        query = records[qi]
        query_id = _get_record_id(query)

        # GT 전수 비교 (자기 자신 제외)
        gt_all = []
        for ci, cand in enumerate(records):
            if ci == qi:
                continue
            cand_id = _get_record_id(cand)
            sim = compute_composite_similarity(query, cand)
            gt_all.append((cand_id, sim))
        gt_all.sort(key=lambda x: -x[1])

        # 단일 쿼리 평가
        eval_result = evaluate_single_query(
            query=query,
            all_records=records,
            engine=engine,
            k=k,
            all_scored=gt_all,
        )
        results.append(eval_result)

        if (progress + 1) % 10 == 0:
            LOGGER.info("진행: %d / %d", progress + 1, n_queries)

    total_elapsed = time.time() - total_start

    # 집계
    return _build_report(results, k, n_queries, len(records), total_elapsed, "local", seed)


def run_evaluation_db(
    records: List[Dict[str, Any]],
    db_params: Dict[str, str],
    norm_params: NormalizationParams,
    n_queries: int = 100,
    k: int = 5,
    seed: int = 42,
) -> Dict[str, Any]:
    """DB 모드 전체 평가를 실행합니다."""
    rng = random.Random(seed)

    n_queries = min(n_queries, len(records))
    query_indices = rng.sample(range(len(records)), n_queries)

    encoder = PathFeatureEncoder(norm_params=norm_params)
    db = FeatureVectorDB(db_params)
    db.connect()

    try:
        engine = TopKSearchEngine(encoder=encoder, db=db)

        LOGGER.info("DB 평가 시작: %d개 쿼리, K=%d, 후보=%d건", n_queries, k, len(records))

        results = []
        total_start = time.time()

        for progress, qi in enumerate(query_indices):
            query = records[qi]
            eval_result = evaluate_single_query_db(
                query=query,
                all_records=records,
                engine=engine,
                k=k,
            )
            results.append(eval_result)

            if (progress + 1) % 10 == 0:
                LOGGER.info("진행: %d / %d", progress + 1, n_queries)

        total_elapsed = time.time() - total_start
    finally:
        db.close()

    return _build_report(results, k, n_queries, len(records), total_elapsed, "db", seed)


def run_evaluation_hybrid(
    records: List[Dict[str, Any]],
    n_queries: int = 100,
    k: int = 5,
    rerank_n: int = 20,
    seed: int = 42,
) -> Dict[str, Any]:
    """하이브리드 모드 평가: 벡터 Top-N → 복합유사도 재정렬 → Top-K."""
    rng = random.Random(seed)

    n_queries = min(n_queries, len(records))
    query_indices = rng.sample(range(len(records)), n_queries)

    norm_params = NormalizationParams.from_dataset(records)
    encoder = PathFeatureEncoder(norm_params=norm_params)
    engine = TopKSearchEngine(encoder=encoder)

    LOGGER.info("하이브리드 평가 시작: %d개 쿼리, K=%d, rerank_N=%d, 후보=%d건",
                n_queries, k, rerank_n, len(records))

    results = []
    total_start = time.time()

    for progress, qi in enumerate(query_indices):
        query = records[qi]
        query_id = _get_record_id(query)

        # GT 전수 비교
        gt_all = []
        for ci, cand in enumerate(records):
            if ci == qi:
                continue
            cand_id = _get_record_id(cand)
            sim = compute_composite_similarity(query, cand)
            gt_all.append((cand_id, sim))
        gt_all.sort(key=lambda x: -x[1])
        gt_top_k = gt_all[:k]
        gt_ids = [gid for gid, _ in gt_top_k]

        # 하이브리드 검색: 벡터 Top-N → 복합유사도 재정렬 → Top-K
        hybrid_result = engine.search_hybrid_local(
            query_record=query,
            candidate_records=records,
            k=k,
            rerank_n=rerank_n,
            composite_fn=compute_composite_similarity,
        )

        vec_ids = [sr.route_path_guid for sr in hybrid_result.results]

        # 평가 지표
        r_at_k = recall_at_k(gt_ids, vec_ids)
        n_at_k = ndcg_at_k(gt_ids, vec_ids)

        # Spearman rho (하이브리드 Top-K와 GT Top-K의 교집합 순위 비교)
        common_ids_list = [cid for cid in gt_ids if cid in set(vec_ids)]
        if len(common_ids_list) >= 2:
            gt_order = {gid: i for i, gid in enumerate(gt_ids)}
            vec_order = {vid: i for i, vid in enumerate(vec_ids)}
            gt_ranks = [gt_order[cid] + 1 for cid in common_ids_list]
            vec_ranks = [vec_order[cid] + 1 for cid in common_ids_list]
            sp_rho = spearman_rho(gt_ranks, vec_ranks)
        else:
            sp_rho = 0.0

        mrr = mean_reciprocal_rank(gt_ids[0], vec_ids) if gt_ids else 0.0

        results.append({
            "query_id": query_id,
            "equipment_name": query.get("equipment_name", ""),
            "utility": query.get("utility", ""),
            "size": query.get("size", ""),
            "path_arrow": query.get("path_arrow", ""),
            "recall_at_k": round(r_at_k, 4),
            "ndcg_at_k": round(n_at_k, 4),
            "spearman_rho": round(sp_rho, 4),
            "mrr": round(mrr, 4),
            "gt_top_k": [{"id": gid, "similarity": round(sim, 6)} for gid, sim in gt_top_k],
            "vec_top_k": [
                {
                    "id": sr.route_path_guid,
                    "cosine_distance": round(sr.cosine_distance, 6),
                    "composite_similarity": round(sr.similarity_score, 6),
                }
                for sr in hybrid_result.results
            ],
            "search_time_ms": round(hybrid_result.search_time_ms, 2),
        })

        if (progress + 1) % 10 == 0:
            LOGGER.info("진행: %d / %d", progress + 1, n_queries)

    total_elapsed = time.time() - total_start

    return _build_report(
        results, k, n_queries, len(records), total_elapsed,
        f"hybrid(rerank_n={rerank_n})", seed,
    )


def _build_report(
    results: List[Dict[str, Any]],
    k: int,
    n_queries: int,
    total_candidates: int,
    total_elapsed: float,
    mode: str,
    seed: int,
) -> Dict[str, Any]:
    """평가 결과를 종합 리포트로 집계합니다."""
    recalls = [r["recall_at_k"] for r in results]
    ndcgs = [r["ndcg_at_k"] for r in results]
    spearman_rhos = [r["spearman_rho"] for r in results]
    mrrs = [r["mrr"] for r in results]
    search_times = [r["search_time_ms"] for r in results]

    # 목표 달성 여부 판정
    avg_recall = sum(recalls) / len(recalls) if recalls else 0.0
    avg_ndcg = sum(ndcgs) / len(ndcgs) if ndcgs else 0.0
    avg_spearman = sum(spearman_rhos) / len(spearman_rhos) if spearman_rhos else 0.0

    targets = {
        "recall_at_k": {"target": 0.80, "actual": round(avg_recall, 4), "passed": avg_recall >= 0.80},
        "ndcg_at_k": {"target": 0.85, "actual": round(avg_ndcg, 4), "passed": avg_ndcg >= 0.85},
        "spearman_rho": {"target": 0.80, "actual": round(avg_spearman, 4), "passed": avg_spearman >= 0.80},
        "search_time_p95_ms": {
            "target": 100.0,
            "actual": round(_percentile(search_times, 0.95), 2),
            "passed": _percentile(search_times, 0.95) < 100.0,
        },
    }

    all_passed = all(t["passed"] for t in targets.values())

    # Recall 분포 히스토그램 (0.0~1.0, 10구간)
    recall_hist = [0] * 10
    for r in recalls:
        bin_idx = min(int(r * 10), 9)
        recall_hist[bin_idx] += 1

    # 유틸리티별 통계
    by_utility: Dict[str, List[float]] = {}
    for r in results:
        util = r.get("utility", "unknown")
        by_utility.setdefault(util, []).append(r["recall_at_k"])
    utility_stats = {
        util: {
            "count": len(vals),
            "mean_recall": round(sum(vals) / len(vals), 4),
        }
        for util, vals in sorted(by_utility.items())
    }

    report = {
        "evaluation_info": {
            "created_at": datetime.now().isoformat(),
            "mode": mode,
            "k": k,
            "n_queries": n_queries,
            "total_candidates": total_candidates,
            "seed": seed,
            "total_evaluation_time_sec": round(total_elapsed, 2),
            "avg_query_time_sec": round(total_elapsed / max(n_queries, 1), 3),
        },
        "target_assessment": targets,
        "all_targets_passed": all_passed,
        "summary": {
            "recall_at_k": _stats(recalls),
            "ndcg_at_k": _stats(ndcgs),
            "spearman_rho": _stats(spearman_rhos),
            "mrr": _stats(mrrs),
            "search_time_ms": _stats(search_times),
        },
        "recall_histogram": {
            f"{i * 10}-{(i + 1) * 10}%": recall_hist[i] for i in range(10)
        },
        "by_utility": utility_stats,
        "query_results": results,
    }

    return report


def save_report(report: Dict[str, Any], output_path: str) -> None:
    """평가 리포트를 JSON으로 저장합니다."""
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    LOGGER.info("평가 리포트 저장: %s", output_path)


def print_summary(report: Dict[str, Any]) -> None:
    """평가 결과 요약을 콘솔에 출력합니다."""
    info = report["evaluation_info"]
    targets = report["target_assessment"]
    summary = report["summary"]

    print(f"\n{'=' * 65}")
    print(f" Top-K 품질 평가 리포트 ({info['mode']} 모드)")
    print(f"{'=' * 65}")
    print(f"  쿼리 수: {info['n_queries']}개  |  K: {info['k']}  |  후보: {info['total_candidates']}건")
    print(f"  평가 시간: {info['total_evaluation_time_sec']:.1f}초 "
          f"({info['avg_query_time_sec']:.3f}초/쿼리)")
    print(f"  시드: {info['seed']}")

    print(f"\n{'─' * 65}")
    print(f" {'지표':<20} {'평균':>8} {'중앙값':>8} {'P5':>8} {'P95':>8} {'목표':>8} {'결과':>6}")
    print(f"{'─' * 65}")

    for metric_key in ["recall_at_k", "ndcg_at_k", "spearman_rho"]:
        s = summary[metric_key]
        t = targets[metric_key]
        status = "PASS" if t["passed"] else "FAIL"
        print(f" {metric_key:<20} {s['mean']:>8.4f} {s['median']:>8.4f} "
              f"{s['p5']:>8.4f} {s['p95']:>8.4f} {t['target']:>8.2f} {status:>6}")

    s_mrr = summary["mrr"]
    print(f" {'mrr':<20} {s_mrr['mean']:>8.4f} {s_mrr['median']:>8.4f} "
          f"{s_mrr['p5']:>8.4f} {s_mrr['p95']:>8.4f} {'  -':>8} {'  -':>6}")

    s_time = summary["search_time_ms"]
    t_time = targets["search_time_p95_ms"]
    status_time = "PASS" if t_time["passed"] else "FAIL"
    print(f" {'search_time_ms':<20} {s_time['mean']:>8.1f} {s_time['median']:>8.1f} "
          f"{s_time['p5']:>8.1f} {s_time['p95']:>8.1f} {t_time['target']:>8.0f} {status_time:>6}")

    print(f"\n{'─' * 65}")
    print(f" 종합 판정: {'[PASS] 모든 목표 달성' if report['all_targets_passed'] else '[FAIL] 일부 목표 미달'}")

    # Recall 히스토그램
    hist = report["recall_histogram"]
    print(f"\n Recall@{info['k']} 분포:")
    for label, count in hist.items():
        bar = "█" * count
        print(f"  {label:>8}: {bar} ({count})")

    # 유틸리티별
    by_util = report.get("by_utility", {})
    if by_util:
        print(f"\n 유틸리티별 Recall@{info['k']}:")
        for util, us in by_util.items():
            print(f"  {util:<12}: {us['mean_recall']:.4f} ({us['count']}건)")

    print(f"{'=' * 65}\n")


# ─────────────────────────────────────────────────────────────
# 6. CLI
# ─────────────────────────────────────────────────────────────


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(name)s | %(levelname)s | %(message)s",
    )

    parser = argparse.ArgumentParser(description="Top-K 유사배관 검색 품질 평가")
    subparsers = parser.add_subparsers(dest="command")

    # 로컬 평가
    local_parser = subparsers.add_parser("local", help="로컬 평가 (DB 없이)")
    local_parser.add_argument("--group_results", required=True, help="GroupPipeResults JSON 경로")
    local_parser.add_argument("--output", required=True, help="평가 리포트 출력 경로")
    local_parser.add_argument("--n_queries", type=int, default=100, help="쿼리 수 (기본: 100)")
    local_parser.add_argument("--k", type=int, default=5, help="Top-K (기본: 5)")
    local_parser.add_argument("--seed", type=int, default=42, help="랜덤 시드 (기본: 42)")

    # DB 평가
    db_parser = subparsers.add_parser("db", help="DB 기반 평가")
    db_parser.add_argument("--group_results", required=True, help="GroupPipeResults JSON 경로")
    db_parser.add_argument("--output", required=True, help="평가 리포트 출력 경로")
    db_parser.add_argument("--n_queries", type=int, default=100, help="쿼리 수")
    db_parser.add_argument("--k", type=int, default=5, help="Top-K")
    db_parser.add_argument("--seed", type=int, default=42, help="랜덤 시드")
    db_parser.add_argument("--dbname", required=True, help="DB명")
    db_parser.add_argument("--user", required=True, help="DB 사용자")
    db_parser.add_argument("--password", required=True, help="DB 비밀번호")
    db_parser.add_argument("--host", default="localhost", help="DB 호스트")
    db_parser.add_argument("--port", default="5432", help="DB 포트")
    db_parser.add_argument("--norm_params", default=None, help="정규화 파라미터 JSON")

    # 하이브리드 평가
    hybrid_parser = subparsers.add_parser("hybrid", help="하이브리드 평가 (벡터 Top-N + 복합유사도 재정렬)")
    hybrid_parser.add_argument("--group_results", required=True, help="GroupPipeResults JSON 경로")
    hybrid_parser.add_argument("--output", required=True, help="평가 리포트 출력 경로")
    hybrid_parser.add_argument("--n_queries", type=int, default=100, help="쿼리 수")
    hybrid_parser.add_argument("--k", type=int, default=5, help="Top-K")
    hybrid_parser.add_argument("--rerank_n", type=int, default=20, help="벡터 검색 후보 수 (기본: 20)")
    hybrid_parser.add_argument("--seed", type=int, default=42, help="랜덤 시드")

    args = parser.parse_args()

    if args.command == "local":
        records = load_group_pipe_records(args.group_results)
        if not records:
            print("경로 레코드가 없습니다.")
            return

        print(f"\n총 경로 레코드: {len(records)}개")
        report = run_evaluation_local(
            records=records,
            n_queries=args.n_queries,
            k=args.k,
            seed=args.seed,
        )

        save_report(report, args.output)
        print_summary(report)

    elif args.command == "db":
        records = load_group_pipe_records(args.group_results)
        if not records:
            print("경로 레코드가 없습니다.")
            return

        norm_params = NormalizationParams()
        if args.norm_params:
            norm_params = NormalizationParams.load(args.norm_params)
        else:
            norm_params = NormalizationParams.from_dataset(records)

        db_params = {
            "host": args.host, "database": args.dbname,
            "user": args.user, "password": args.password, "port": args.port,
        }

        print(f"\n총 경로 레코드: {len(records)}개")
        report = run_evaluation_db(
            records=records,
            db_params=db_params,
            norm_params=norm_params,
            n_queries=args.n_queries,
            k=args.k,
            seed=args.seed,
        )

        save_report(report, args.output)
        print_summary(report)

    elif args.command == "hybrid":
        records = load_group_pipe_records(args.group_results)
        if not records:
            print("경로 레코드가 없습니다.")
            return

        print(f"\n총 경로 레코드: {len(records)}개")
        report = run_evaluation_hybrid(
            records=records,
            n_queries=args.n_queries,
            k=args.k,
            rerank_n=args.rerank_n,
            seed=args.seed,
        )

        save_report(report, args.output)
        print_summary(report)

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
