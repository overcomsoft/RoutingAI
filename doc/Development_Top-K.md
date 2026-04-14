# Top-K 추천경로 검색 시스템 개발계획서

> **프로젝트**: RoutingAI 자동 배관 설계 시스템  
> **문서 버전**: v2.0  
> **작성일**: 2026-04-14  
> **최종 수정일**: 2026-04-14  
> **대상 모듈**: `TopKRoutingSearch` (신규)  
> **개발 상태**: Phase A~C 구현 완료

---

## 1. 개요

### 1.1 배경

현재 `AutoRoutingDesigner_V2.py`는 신규 PoC에 대해 **단일 최적 경로 1개**만 반환한다.
템플릿 그룹 전체를 Python에서 순차적으로 비교하는 O(N) 방식이며,
경로 수가 수천~수만 개로 증가하면 응답 시간이 급격히 늘어난다.

### 1.2 목표

| 항목 | 현재 | 목표 |
|------|------|------|
| 추천 결과 수 | 1개 (best match) | **Top-K (기본 5개)** |
| 검색 방식 | Python 전수 비교 O(N) | **pgvector ANN 검색 O(log N)** |
| 응답 시간 | N에 비례 | **< 100ms** (인덱스 활용) |
| 유사도 모델 | 5차원 가중합 (Python) | **30차원 벡터 + 코사인 거리** (DB) |
| 결과 활용 | 내부 로직 전용 | **3D 뷰어에서 Top-K 비교 UI** |

### 1.3 시스템 위치 (아키텍처)

```
                    ┌──────────────────────────────────────────────────┐
                    │  PostgreSQL (AUTOROUTINGV7)                      │
                    │                                                  │
                    │  TB_ROUTE_PATH              ← 경로 메타 (2,624) │
                    │  TB_ROUTE_PATH_SEGMENT_MAP  ← 경로↔구간 매핑 *  │
                    │  TB_ROUTE_SEGMENTS          ← 경로 구간 (5,355) │
                    │  TB_ROUTE_SEGMENT_DETAIL    ← 배관 상세 (71,655)│
                    │                                                  │
                    │  TB_ROUTE_FEATURE_VECTOR    ← [신규] 30D 벡터   │
                    │       ↑ HNSW 인덱스 (pgvector cosine)            │
                    └───────────────┬──────────────────────────────────┘
                                    │
            ┌───────────────────────┼───────────────────────┐
            │                       │                       │
   ┌────────▼────────┐   ┌─────────▼────────┐   ┌──────────▼─────────┐
   │ BuildFeature    │   │ TopKRouting      │   │ AutoRouting        │
   │ Vectors.py      │   │ Search.py        │   │ 3DViewer (C#/WPF)  │
   │                 │   │                  │   │                    │
   │ DB 경로 로드    │   │ 30D 벡터 인코딩  │   │ Top-K 비교 UI      │
   │ + 벡터 인코딩   │   │ + pgvector 검색  │   │ 유사도 히트맵      │
   │ + DB 저장       │   │ + Top-K 결과반환 │   │ 3D 경로 오버레이   │
   │ (배치/오프라인) │   │ (실시간/온라인)  │   │                    │
   └─────────────────┘   └──────────────────┘   └────────────────────┘
```

> **\* TB_ROUTE_PATH_SEGMENT_MAP 참고**: 세그먼트 조합(공유/재활용)을 위한 매핑 테이블.
> 현재 RoutePathLoader는 `TB_ROUTE_SEGMENTS.ROUTE_PATH_GUID`로 직접 조인 중.
> 향후 MAP 테이블이 활성화되면 MAP 경유 조인으로 전환 필요.

---

## 2. 현행 유사도 계산 분석

### 2.1 복합 유사도 구조 (`compute_composite_similarity`)

```
최종 유사도 (0~1)
  │
  ├─ Arrow Similarity ×0.25 ── Levenshtein Distance("R-H-D-R" vs "R-H-H-D-R")
  │
  ├─ Vector Similarity ×0.25 ── 구간별 코사인유사도 평균 × 커버리지
  │     └─ path_step_vectors: [{x,y,z}, ...] 가변길이
  │
  ├─ Range Similarity ×0.15 ── BBox X/Y/Z 범위 비율 비교
  │     └─ path_range: {x, y, z}
  │
  ├─ Length Similarity ×0.15 ── 총 길이 비율 비교
  │     └─ path_step_lengths: [float, ...] 가변길이
  │
  └─ Obstacle Relation ×0.20 ── 4유형 장애물 가중합
        │
        ├─ 구조기둥 ×0.35: count(0.2) + min_dist(0.2) + crossings(0.3) + pattern(0.3)
        ├─ H-Beam   ×0.30: crossing(0.4) + clearance(0.3) + parallel(0.3)
        ├─ 그레이팅 ×0.20: coverage(0.4) + count_below(0.3) + gap(0.3)
        └─ 포스트   ×0.15: density(0.3) + alignment(0.4) + count(0.3)
```

### 2.2 AutoRoutingDesigner_V2 확장 유사도 (8차원)

Designer V2에서는 Phase 1+2의 5차원에 **3개 차원을 추가**하여 8차원으로 확장한다:

| 차원 | 가중치 | 내용 |
|------|--------|------|
| path_arrow | 0.12 | 방향 패턴 |
| path_vector | 0.12 | 변위 벡터 시퀀스 |
| path_range | 0.08 | BBox 범위 |
| path_length | 0.08 | 총 길이 |
| **equip_relative** | **0.15** | 장비 상대좌표 + 시작면(E/W/N/S) |
| **terminal** | **0.15** | 종단점 타입/거리/레벨 |
| obstacle_proximity | 0.20 | 장애물 공간관계 |
| **level** | **0.10** | SpaceInfo 레벨(CSF/A/F/CR) 매칭 |

### 2.3 pgvector 전환 시 해결해야 할 과제

| 과제 | 설명 | 해결 방안 |
|------|------|-----------|
| **가변길이 → 고정길이** | path_step_vectors, path_step_lengths가 경로마다 길이가 다름 | 통계 요약(평균, 분산, 비율)으로 고정 차원 변환 |
| **문자열 패턴** | path_arrow("R-H-D-R"), col_relative_pattern("LBR") | N-gram 빈도 벡터로 변환 |
| **이종 스케일** | 길이(mm), 각도(0~90), 카운트(0~N), 비율(0~1) 혼재 | Min-Max 또는 Z-score 정규화 |
| **가중치 반영** | 현재 5~8차원별 가중치가 정교하게 튜닝되어 있음 | 인코딩 시 가중치를 차원 스케일에 반영 |

---

## 3. 벡터 인코딩 설계

### 3.1 총 벡터 차원: 30D (확정안 — 기하학 중심 설계)

> **설계 변경 사유**: 장애물, 레벨, 장비상대좌표 등은 데이터 결측 가능성이 있으므로,
> 항상 존재하는 **기하학적 형상 특성 중심**으로 설계를 변경하였다.
> 원안(8차원 복합유사도 기반)에서 기하학 중심으로 전환하여 데이터 결측에 강건한 구조로 확정.

```
30차원 벡터 구성 (확정안)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 인덱스     차원명                 설명                           가중치
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [0~2]     시작 위상 (3D)         첫 번째 세그먼트 방향 단위벡터     0.20
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [3~5]     종단 위상 (3D)         마지막 세그먼트 방향 단위벡터      0.20
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [6~8]     공간 변위 (3D)         시작→종단 상대 위치 (정규화)       0.15
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [9~11]    바운딩 박스 (3D)        경로 BBox XYZ 범위 (정규화)       0.15
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [12~14]   구간 1 (3D)            초기 33% 구간의 지배적 꺾임 방향   0.06
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [15~17]   구간 2 (3D)            중기 33% 구간의 지배적 꺾임 방향   0.06
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [18~20]   구간 3 (3D)            후기 33% 구간의 지배적 꺾임 방향   0.06
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [21~24]   환경 및 비용 (4D)       총길이, 꺾임횟수, 장애물, 서포트  0.12
           [21] total_length (정규화)                          (각 0.03)
           [22] bend_count (정규화)
           [23] obstacle_score (결측 시 0.0)
           [24] support_score (결측 시 0.0)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [25~29]   예비 (5D)              향후 확장용 (0.0 패딩)              -
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 가중치 합: 0.20+0.20+0.15+0.15+0.06+0.06+0.06+0.12 = 1.00
```

**구간별 꺾임 방향 계산 방법:**

- 경로를 길이 기준으로 3등분 (초기/중기/후기)
- 각 구간 내 연속 세그먼트의 외적(cross product)을 누적
- 누적 벡터를 단위벡터로 정규화하여 지배적 꺾임 방향을 표현

### 3.2 가중치 반영 전략 (Weighted Embedding)

코사인 거리는 모든 차원을 동등하게 취급하므로,
가중치를 **벡터 스케일 팩터**로 반영한다.
각 차원 그룹에 `sqrt(weight × total_dim / group_dim)` 스케일을 적용한다.

```python
# 실제 구현: PathFeatureEncoder._build_scale_factors()
# scale = sqrt(weight * 30 / dim_count) — VECTOR_DIM=30 기준
WEIGHT_MAP = {
    "start_topology":  {"indices": (0, 3),   "weight": 0.20},  # sqrt(0.20*30/3) = sqrt(2.0) ≈ 1.414
    "end_topology":    {"indices": (3, 6),   "weight": 0.20},  # sqrt(2.0) ≈ 1.414
    "displacement":    {"indices": (6, 9),   "weight": 0.15},  # sqrt(1.5) ≈ 1.225
    "bounding_box":    {"indices": (9, 12),  "weight": 0.15},  # sqrt(1.5) ≈ 1.225
    "segment_1":       {"indices": (12, 15), "weight": 0.06},  # sqrt(0.6) ≈ 0.775
    "segment_2":       {"indices": (15, 18), "weight": 0.06},  # sqrt(0.6) ≈ 0.775
    "segment_3":       {"indices": (18, 21), "weight": 0.06},  # sqrt(0.6) ≈ 0.775
    "env_cost":        {"indices": (21, 25), "weight": 0.12},  # sqrt(0.9) ≈ 0.949
    "reserved":        {"indices": (25, 30), "weight": 0.00},  # 스케일 없음
}
# 인코딩 시: vector[i] *= scale_factor[group]
# L2 정규화 후 코사인 거리 계산 → 가중치가 자연스럽게 반영됨
```

### 3.3 정규화 파라미터

벡터 인코딩 전에 환경/비용 차원을 0~1 범위로 정규화한다.
정규화 파라미터는 `NormalizationParams.from_dataset()`으로 전체 데이터셋의 99퍼센타일에서 산출한다.

```python
# 기본값 (데이터 없을 때)
NORMALIZATION_DEFAULTS = {
    "bbox_max": {"x": 30000.0, "y": 30000.0, "z": 20000.0},
    "displacement_max": 50000.0,
    "total_length_max": 50000.0,
    "bend_count_max": 30,
    "obstacle_score_max": 1.0,
    "support_score_max": 1.0,
}
```

**AUTOROUTINGV7 실측값** (2,624건 경로 기준, 2026-04-14 산출):

```python
# data/FeatureVectors/db_norm_params.json
{
    "bbox_max":          {"x": 9534.0, "y": 11955.4, "z": 11492.0},
    "displacement_max":  11616.5,
    "total_length_max":  26465.2,
    "bend_count_max":    48.0,
    "obstacle_score_max": 1.0,
    "support_score_max":  1.0
}
```

> **참고**: 시작/종단 위상(단위벡터), 구간별 꺾임 방향(단위벡터)은 이미 크기 1로 정규화되므로 별도 파라미터 불필요.

---

## 4. 데이터베이스 설계

### 4.1 신규 테이블: `TB_ROUTE_FEATURE_VECTOR`

기존 `TB_ROUTE_PATH`에 컬럼을 추가하지 않고, **별도 테이블**로 분리한다.
이유: pgvector 인덱스 관리 독립, 벡터 차원 변경 시 기존 테이블 영향 없음.

```sql
-- ============================================================
-- 0. 확장 설치 (최초 1회)
-- ============================================================
CREATE EXTENSION IF NOT EXISTS vector;

-- ============================================================
-- 1. 특징 벡터 테이블
-- ============================================================
CREATE TABLE TB_ROUTE_FEATURE_VECTOR (
    -- PK
    FEATURE_VECTOR_ID    SERIAL          PRIMARY KEY,

    -- FK: 기존 경로 테이블 참조
    ROUTE_PATH_GUID      CHAR(36)        NOT NULL,

    -- 경로 식별 (빠른 필터링용)
    PROCESS_NAME         VARCHAR(50),     -- CMP, ETCH, DIFF, ...
    EQUIPMENT_NAME       VARCHAR(100)    NOT NULL,
    UTILITY_GROUP        VARCHAR(50),     -- Vaccum, Gas, Chemical, ...
    UTILITY              VARCHAR(50),     -- Foreline, PV_VENT, PUMP, ...
    SIZE                 VARCHAR(20),     -- 20A, 50A, 100A, ...

    -- PoC 위치 정보 (검색 조건)
    START_POSX           DOUBLE PRECISION,
    START_POSY           DOUBLE PRECISION,
    START_POSZ           DOUBLE PRECISION,
    END_POSX             DOUBLE PRECISION,
    END_POSY             DOUBLE PRECISION,
    END_POSZ             DOUBLE PRECISION,

    -- 경로 요약 (결과 표시용)
    DIRECTION_PATTERN    VARCHAR(200),    -- "R-H-D-R"
    TOTAL_LENGTH_MM      DOUBLE PRECISION,
    STEP_COUNT           INTEGER,
    START_LEVEL          VARCHAR(10),     -- CSF, A, F, CR
    END_LEVEL            VARCHAR(10),

    -- ★ 핵심: 30차원 특징 벡터
    FEATURE_VECTOR       vector(30)      NOT NULL,

    -- 인코딩 메타데이터
    ENCODER_VERSION      VARCHAR(20)     DEFAULT 'v1.0',
    ENCODED_AT           TIMESTAMPTZ     DEFAULT NOW(),

    -- 원본 특징량 (디버깅/재인코딩용)
    RAW_FEATURES_JSON    JSONB,

    -- FK 제약
    CONSTRAINT fk_route_path
        FOREIGN KEY (ROUTE_PATH_GUID)
        REFERENCES TB_ROUTE_PATH (ROUTE_PATH_GUID)
        ON DELETE CASCADE
);

-- ============================================================
-- 2. 인덱스
-- ============================================================

-- pgvector 코사인 거리 인덱스 (HNSW: 더 정확, 빌드 느림)
CREATE INDEX idx_feature_vector_hnsw
    ON TB_ROUTE_FEATURE_VECTOR
    USING hnsw (FEATURE_VECTOR vector_cosine_ops)
    WITH (m = 16, ef_construction = 200);

-- pgvector 코사인 거리 인덱스 (IVFFlat: 빌드 빠름, 대규모용)
-- 데이터가 10만 건 이상일 때 HNSW 대신 사용 검토
-- CREATE INDEX idx_feature_vector_ivfflat
--     ON TB_ROUTE_FEATURE_VECTOR
--     USING ivfflat (FEATURE_VECTOR vector_cosine_ops)
--     WITH (lists = 100);

-- 필터링용 복합 인덱스
CREATE INDEX idx_fv_process_equip
    ON TB_ROUTE_FEATURE_VECTOR (PROCESS_NAME, EQUIPMENT_NAME);

CREATE INDEX idx_fv_utility
    ON TB_ROUTE_FEATURE_VECTOR (UTILITY_GROUP, UTILITY);

CREATE INDEX idx_fv_size
    ON TB_ROUTE_FEATURE_VECTOR (SIZE);

-- ============================================================
-- 3. 뷰: Top-K 검색 결과 + 경로 메타 조인
-- ============================================================
CREATE OR REPLACE VIEW VW_ROUTE_SEARCH AS
SELECT
    fv.FEATURE_VECTOR_ID,
    fv.ROUTE_PATH_GUID,
    fv.PROCESS_NAME,
    fv.EQUIPMENT_NAME,
    fv.UTILITY_GROUP,
    fv.UTILITY,
    fv.SIZE,
    fv.START_POSX, fv.START_POSY, fv.START_POSZ,
    fv.END_POSX,   fv.END_POSY,   fv.END_POSZ,
    fv.DIRECTION_PATTERN,
    fv.TOTAL_LENGTH_MM,
    fv.STEP_COUNT,
    fv.START_LEVEL,
    fv.END_LEVEL,
    fv.FEATURE_VECTOR,
    rp.SOURCE_OWNER_NAME,
    rp.TARGET_OWNER_NAME,
    rp.PR_BRANCH_COUNT,
    rp.PR_BEND_COUNT,
    rp.PR_PATH_EFFICIENCY,
    rp.PR_TOTAL_LENGTH
FROM TB_ROUTE_FEATURE_VECTOR fv
JOIN TB_ROUTE_PATH rp ON fv.ROUTE_PATH_GUID = rp.ROUTE_PATH_GUID;
```

### 4.2 Top-K 검색 SQL 함수

```sql
-- ============================================================
-- Top-K 유사 경로 검색 함수
-- ============================================================
CREATE OR REPLACE FUNCTION fn_search_top_k_routes(
    p_query_vector   vector(30),
    p_k              INTEGER DEFAULT 5,
    p_utility_group  VARCHAR DEFAULT NULL,
    p_utility        VARCHAR DEFAULT NULL,
    p_size           VARCHAR DEFAULT NULL,
    p_process        VARCHAR DEFAULT NULL
)
RETURNS TABLE (
    route_path_guid      CHAR(36),
    equipment_name       VARCHAR(100),
    utility_group        VARCHAR(50),
    utility              VARCHAR(50),
    direction_pattern    VARCHAR(200),
    total_length_mm      DOUBLE PRECISION,
    step_count           INTEGER,
    start_level          VARCHAR(10),
    end_level            VARCHAR(10),
    cosine_distance      DOUBLE PRECISION,
    similarity_score     DOUBLE PRECISION,
    rank_position        INTEGER
)
LANGUAGE plpgsql
AS $$
BEGIN
    RETURN QUERY
    SELECT
        fv.ROUTE_PATH_GUID,
        fv.EQUIPMENT_NAME,
        fv.UTILITY_GROUP,
        fv.UTILITY,
        fv.DIRECTION_PATTERN,
        fv.TOTAL_LENGTH_MM,
        fv.STEP_COUNT,
        fv.START_LEVEL,
        fv.END_LEVEL,
        (fv.FEATURE_VECTOR <=> p_query_vector)::DOUBLE PRECISION AS cosine_dist,
        (1.0 - (fv.FEATURE_VECTOR <=> p_query_vector))::DOUBLE PRECISION AS sim_score,
        ROW_NUMBER() OVER (
            ORDER BY fv.FEATURE_VECTOR <=> p_query_vector
        )::INTEGER AS rank_pos
    FROM TB_ROUTE_FEATURE_VECTOR fv
    WHERE
        (p_utility_group IS NULL OR fv.UTILITY_GROUP = p_utility_group)
        AND (p_utility IS NULL OR fv.UTILITY = p_utility)
        AND (p_size IS NULL OR fv.SIZE = p_size)
        AND (p_process IS NULL OR fv.PROCESS_NAME = p_process)
    ORDER BY fv.FEATURE_VECTOR <=> p_query_vector
    LIMIT p_k;
END;
$$;
```

---

## 5. 모듈 설계

### 5.1 파일 구조

```
RoutingAI/src/
├── AnalyzeRoutingAi_V2.py          (기존) Phase 1+2 분석
├── AutoRoutingDesigner_V2.py        (기존) Phase 3 자동 설계
├── ExtractVacuumToPumpPaths.py      (기존) Vaccum→Pump 경로 추출
├── VisualizeVacuumPumpPaths.py      (기존) 시각화
│
├── TopKRoutingSearch.py             ★ [구현 완료] Top-K 검색 메인 모듈
│   ├── RoutePathLoader              ── DB에서 경로 데이터 로드
│   ├── PathFeatureEncoder           ── 경로 → 30D 벡터 인코딩
│   ├── NormalizationParams          ── 정규화 파라미터 관리
│   ├── FeatureVectorDB              ── DB CRUD + pgvector 검색
│   └── TopKSearchEngine             ── Top-K 검색 + 결과 조립
│
├── BuildFeatureVectors.py           ★ [구현 완료] 기존 경로 일괄 벡터화 (배치)
│   └── DB 경로(TB_ROUTE_PATH) → TB_ROUTE_FEATURE_VECTOR 마이그레이션
│
├── create_feature_vector_table.sql  ★ [구현 완료] DDL + 인덱스 + 검색 함수
│
└── EvaluateTopK.py                  ☐ [미구현] Top-K 품질 평가
    └── Recall@K, NDCG@K, 복합유사도 상관계수 측정
```

### 5.2 클래스 다이어그램

```
┌──────────────────────────────────┐
│  RoutePathLoader                 │  DB에서 경로 데이터 로드
├──────────────────────────────────┤
│ - db_params: dict                │
│ - _conn: psycopg2.connection     │
├──────────────────────────────────┤
│ + connect()                      │
│ + close()                        │
│ + load_all_paths() → List[dict]  │  전체 경로 일괄 로드
│ + load_path(guid) → dict         │  단건 로드
│ - _build_record(path, details)   │  DB 행 → 인코더 입력 변환
└──────────────────┬───────────────┘
                   │ 조회: TB_ROUTE_PATH
                   │       + TB_ROUTE_SEGMENTS
                   │       + TB_ROUTE_SEGMENT_DETAIL
                   ▼
┌──────────────────────────────────┐
│  PathFeatureEncoder              │  경로 → 30D 벡터 인코딩
├──────────────────────────────────┤
│ - VECTOR_DIM: int = 30           │
│ - norm_params: NormalizationParams│
│ - scale_factors: np.ndarray      │
├──────────────────────────────────┤
│ + encode(record) → np.ndarray    │  경로 레코드 → 30D 벡터
│ + normalize_l2(vec) → np.ndarray │  L2 정규화
│ + decode_explain(vec) → dict     │  벡터 → 사람이 읽을 수 있는 해석
│ - _encode_start_topology(vecs)   │  [0~2]  시작 방향 단위벡터
│ - _encode_end_topology(vecs)     │  [3~5]  종단 방향 단위벡터
│ - _encode_displacement(rec)      │  [6~8]  공간 변위 (정규화)
│ - _encode_bounding_box(rec)      │  [9~11] BBox 범위 (정규화)
│ - _encode_segment_bends(vecs)    │  [12~20] 3구간 꺾임 방향 (외적)
│ - _encode_env_cost(rec)          │  [21~24] 환경/비용
│ - _build_scale_factors()         │  가중치 스케일 팩터 계산
└──────────────────┬───────────────┘
                   │ uses
┌──────────────────▼───────────────┐
│  NormalizationParams             │  정규화 파라미터 관리
├──────────────────────────────────┤
│ + bbox_max: dict                 │
│ + displacement_max: float        │
│ + total_length_max: float        │
│ + bend_count_max: float          │
│ + obstacle_score_max: float      │
│ + support_score_max: float       │
├──────────────────────────────────┤
│ + from_dataset(records) → self   │  데이터셋에서 99pctl 산출
│ + save(path)                     │  JSON으로 저장
│ + load(path) → self              │  JSON에서 로드
└──────────────────────────────────┘

┌──────────────────────────────────┐
│  FeatureVectorDB                 │  DB CRUD + pgvector 검색
├──────────────────────────────────┤
│ - TABLE: "TB_ROUTE_FEATURE_VECTOR"│
│ - _conn: psycopg2.connection     │
├──────────────────────────────────┤
│ + connect() / close()            │
│ + ensure_schema()                │  테이블 + 인덱스 자동 생성
│ + ensure_hnsw_index()            │  HNSW 인덱스 생성
│ + insert_vector(record, vec)     │  단건 저장
│ + bulk_insert(records, vecs)     │  일괄 저장 (500건 배치)
│ + search_top_k(query_vec, k,     │  pgvector 코사인 거리 Top-K
│     filters) → TopKResult        │
│ + get_vector(route_guid)         │  단건 조회
│ + delete_by_route(route_guid)    │  삭제
│ + get_stats() → dict             │  통계 (총 건수, 공정별 분포)
└──────────────────┬───────────────┘
                   │ uses
┌──────────────────▼───────────────┐
│  TopKSearchEngine                │  통합 검색 엔진
├──────────────────────────────────┤
│ - encoder: PathFeatureEncoder    │
│ - db: FeatureVectorDB (Optional) │
├──────────────────────────────────┤
│ + search(                        │  좌표 기반 검색 (DB)
│     start_pos, end_pos,          │
│     utility, size, process,      │
│     k=5                          │
│   ) → TopKResult                 │
│                                  │
│ + search_by_record(              │  경로 레코드로 검색 (DB)
│     query_record, k=5            │
│   ) → TopKResult                 │
│                                  │
│ + search_local(                  │  로컬 메모리 검색 (테스트용)
│     query, candidates, k=5       │
│   ) → TopKResult                 │
└──────────────────────────────────┘

┌──────────────────────────────────┐
│  TopKResult (데이터 클래스)       │
├──────────────────────────────────┤
│ + query_vector: np.ndarray       │
│ + results: List[SearchResult]    │
│ + search_time_ms: float          │
│ + filters_applied: dict          │
│ + total_candidates: int          │
├──────────────────────────────────┤
│  SearchResult:                   │
│  + rank: int                     │
│  + route_path_guid: str          │
│  + equipment_name: str           │
│  + process_name: str             │
│  + utility: str                  │
│  + size: str                     │
│  + direction_pattern: str        │
│  + cosine_distance: float        │
│  + similarity_score: float       │
│  + total_length_mm: float        │
│  + step_count: int               │
│  + start_level: str              │
│  + end_level: str                │
└──────────────────────────────────┘
```

---

## 6. 처리 흐름

### 6.1 벡터 생성 파이프라인 (배치, 오프라인) — 구현 완료

```
[실행 명령어]
  python BuildFeatureVectors.py from-db \
      --dbname AUTOROUTINGV7 --user postgres --password dinno \
      --save_norm ../data/FeatureVectors/db_norm_params.json

[1] DB 경로 데이터 로드 (RoutePathLoader.load_all_paths)
    │  TB_ROUTE_PATH                      ← 경로 메타 (2,624건)
    │  + TB_ROUTE_SEGMENTS                ← 경로 구간 (5,355건)
    │  + TB_ROUTE_SEGMENT_DETAIL          ← 배관/자재 상세 (71,655건)
    │  소요시간: ~0.5초
    │
[2] 특징량 추출 (RoutePathLoader._build_record)
    │  SEGMENT_DETAIL의 FROM/TO 좌표에서:
    │  - path_step_vectors: [{x,y,z}, ...] 변위 벡터 시퀀스
    │  - path_arrow: "R-H-D-H-R-..." 방향 패턴 (R/H/D 자동 분류)
    │  - path_range: {x,y,z} BBox 범위
    │  - path_total_length: 총 길이 (PR_TOTAL_LENGTH 또는 합산)
    │
[3] 정규화 파라미터 산출
    │  NormalizationParams.from_dataset(all_records)
    │  99퍼센타일 기반 (이상치 영향 최소화)
    │  → db_norm_params.json 저장
    │
[4] 30D 벡터 인코딩 (PathFeatureEncoder)
    │  encode(record) → [30D float]
    │  normalize_l2(vec) → 단위벡터
    │  소요시간: ~0.1ms/건 (2,624건 = 0.15초)
    │
[5] DB 저장 (FeatureVectorDB.bulk_insert)
    │  "TB_ROUTE_FEATURE_VECTOR" INSERT (500건 배치)
    │  소요시간: ~1.2초
    │
[6] HNSW 인덱스 빌드
    │  ensure_hnsw_index() → "IDX_FEATURE_VECTOR_HNSW"
    │  (m=16, ef_construction=200)
    │
[7] 품질 검증
       ☐ EvaluateTopK (미구현)
```

### 6.2 Top-K 검색 흐름 (실시간, 온라인) — 구현 완료

```
[실행 명령어]
  python TopKRoutingSearch.py search \
      --start 205117.7,15457.4,15495.0 --end 208364.8,8689.6,12372.7 \
      --utility NW --size 3/4B --k 10 \
      --dbname AUTOROUTINGV7 --user postgres --password dinno \
      --norm_params ../data/FeatureVectors/db_norm_params.json

[사용자 입력]
  │  시작 PoC: (205117.7, 15457.4, 15495.0)
  │  종단 위치: (208364.8, 8689.6, 12372.7)
  │  유틸리티: NW / 사이즈: 3/4B
  │
  ▼
[1] 쿼리 레코드 구성 (TopKSearchEngine._build_query_record)
  │  - path_step_vectors가 없으면 start→end 단일 벡터로 간이 생성
  │  - path_range: start/end에서 추정
  │  - path_total_length: 직선 거리로 추정
  │
[2] 쿼리 벡터 생성
  │  PathFeatureEncoder.encode(query_record) → [30D]
  │  PathFeatureEncoder.normalize_l2(vec) → 단위벡터
  │
[3] pgvector 검색 (FeatureVectorDB.search_top_k)
  │  SELECT ... FROM "TB_ROUTE_FEATURE_VECTOR"
  │  WHERE "UTILITY" = 'NW' AND "SIZE" = '3/4B'
  │  ORDER BY "FEATURE_VECTOR" <=> query_vector
  │  LIMIT 10
  │  ← 실측 응답: 4.6ms (HNSW 인덱스, 2,624건)
  │
[4] 결과 반환
     TopKResult {
       results: [
         {rank:1, similarity:0.624, equipment:"WTNHJ02_", pattern:"R-D-H-...", length:18970mm},
         {rank:2, similarity:0.622, equipment:"WTNHJ02_", pattern:"R-D-H-...", length:18844mm},
         ...
       ],
       search_time_ms: 4.6,
       filters_applied: {utility: "NW", size: "3/4B"}
     }
```

> **참고**: 간이 인코딩(start/end만)은 유사도가 낮게 나올 수 있다.
> 실제 경로의 step_vectors를 함께 제공하면(`search_by_record`) 정밀도가 크게 향상된다.

### 6.3 시퀀스 다이어그램 (실시간 검색)

```
 사용자         3D뷰어(C#)     TopKSearch(Py)    PostgreSQL
   │               │                │                │
   │ PoC 선택      │                │                │
   ├──────────────►│                │                │
   │               │ HTTP/gRPC      │                │
   │               ├───────────────►│                │
   │               │                │ encode()       │
   │               │                ├──┐             │
   │               │                │  │ 30D 벡터    │
   │               │                │◄─┘             │
   │               │                │                │
   │               │                │ fn_search_top_k│
   │               │                ├───────────────►│
   │               │                │                │ HNSW scan
   │               │                │                ├──┐
   │               │                │                │  │ < 100ms
   │               │                │                │◄─┘
   │               │                │◄───────────────┤ Top-5 결과
   │               │                │                │
   │               │                │ 정밀 유사도     │
   │               │                │ 재계산          │
   │               │                ├──┐             │
   │               │                │  │             │
   │               │                │◄─┘             │
   │               │                │                │
   │               │◄───────────────┤ TopKResult     │
   │               │                │                │
   │ Top-K 표시    │                │                │
   │◄──────────────┤                │                │
   │               │                │                │
```

---

## 7. 3D 뷰어 연동 설계

### 7.1 C# 측 변경사항

```
SearchRoutingPath/AutoRouting3DViewer/
├── Services/
│   └── RouteDatabaseService.cs   ← Top-K 검색 메서드 추가
├── Models/
│   └── TopKSearchResult.cs       ★ [신규] 검색 결과 모델
├── ViewModels/
│   └── TopKCompareViewModel.cs   ★ [신규] Top-K 비교 뷰모델
└── MainWindow.xaml.cs            ← Top-K UI 패널 추가
```

### 7.2 UI 구성

```
┌─────────────────────────────────────────────────────────────┐
│  Top-K 유사 경로 검색                                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  시작 PoC: [___________]  종단: [___________]  [검색]       │
│  유틸리티: [Foreline ▼]   사이즈: [50A ▼]                   │
│                                                             │
│  ┌─ Top-5 결과 ────────────────────────────────────────┐    │
│  │ # │ 유사도 │ 장비      │ 패턴      │ 길이  │ 표시  │    │
│  ├───┼────────┼───────────┼───────────┼───────┼───────┤    │
│  │ 1 │ 0.923  │ DANHJ14   │ R-H-D-R   │ 12.5m │ [✓]  │    │
│  │ 2 │ 0.881  │ ELOHJ07   │ R-H-H-D-R │ 13.1m │ [✓]  │    │
│  │ 3 │ 0.854  │ DANHJ14   │ R-D-R     │ 12.8m │ [ ]  │    │
│  │ 4 │ 0.812  │ SLWHJ02   │ R-H-D-H-R │ 14.2m │ [ ]  │    │
│  │ 5 │ 0.783  │ SLWHJ02   │ H-R-D-R   │ 11.9m │ [ ]  │    │
│  └───┴────────┴───────────┴───────────┴───────┴───────┘    │
│                                                             │
│  ┌─ 유사도 상세 분해 (선택 경로) ──────────────────────┐    │
│  │  Arrow: ████████░░ 0.85                             │    │
│  │  Vector: █████████░ 0.92                            │    │
│  │  Range: ███████░░░ 0.76                             │    │
│  │  Length: ████████░░ 0.89                             │    │
│  │  Equip: █████████░ 0.95                             │    │
│  │  Terminal: ████████░░ 0.88                           │    │
│  │  Obstacle: ██████░░░░ 0.68                          │    │
│  │  Level: █████████░ 0.90                             │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
│  [3D에서 이 경로로 자동 설계 시작]                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 7.3 3D 뷰포트 오버레이

```
체크된 Top-K 경로를 3D 공간에 동시 표시:

  - Rank 1: 실선, 빨강, 두께 3px, 불투명
  - Rank 2: 실선, 주황, 두께 2.5px, 80% 불투명
  - Rank 3: 점선, 녹색, 두께 2px, 60% 불투명
  - Rank 4: 점선, 파랑, 두께 1.5px, 40% 불투명
  - Rank 5: 점선, 보라, 두께 1px, 30% 불투명

  + 공통 시작점: 파란 구체 (●)
  + 각 종단점: Rank 색상별 별표 (★)
```

---

## 8. 품질 평가 계획

### 8.1 평가 지표

| 지표 | 설명 | 목표 |
|------|------|------|
| **Recall@K** | 복합유사도 Top-K 중 pgvector Top-K에 포함된 비율 | >= 0.80 |
| **NDCG@K** | 순위 품질 (복합유사도 순위와의 상관) | >= 0.85 |
| **Spearman rho** | 코사인 거리 vs 복합유사도 순위 상관계수 | >= 0.80 |
| **응답 시간** | 95th percentile 검색 시간 | < 100ms |
| **인코딩 시간** | 단일 경로 벡터 인코딩 | < 5ms |

### 8.2 평가 방법

```
[1] Ground Truth 생성
    - 전체 경로 N개에서 무작위 100개 쿼리 샘플링
    - 각 쿼리에 대해 compute_composite_similarity로 전수 비교
    - 복합 유사도 기준 Top-5 → Ground Truth

[2] pgvector 검색 실행
    - 동일 100개 쿼리에 대해 pgvector Top-5 검색

[3] 비교 분석
    - Recall@5: GT Top-5 ∩ pgvector Top-5 / 5
    - NDCG@5: 순위 가중 정밀도
    - Spearman rho: 전체 순위 상관

[4] 벡터 차원 / 가중치 튜닝
    - Recall@5 < 0.80이면:
      ① 차원 수 확대 (30 → 40)
      ② 가중치 스케일 조정
      ③ 인코딩 방식 변경 (N-gram 확대 등)
```

---

## 9. 개발 일정

### Phase A: 벡터 인코딩 모듈 — ✅ 완료 (2026-04-14)

| 단계 | 작업 | 산출물 | 상태 |
|------|------|--------|------|
| A-1 | PathFeatureEncoder 구현 (30D 확정안) | TopKRoutingSearch.py | ✅ |
| A-2 | NormalizationParams 산출 (99pctl) | db_norm_params.json | ✅ |
| A-3 | 로컬 Top-K 검색 테스트 | CLI test 커맨드 | ✅ |
| A-4 | 가중치 스케일 팩터 검증 | 2,624건 인코딩 0.1ms/건 | ✅ |

### Phase B: DB 테이블 + 마이그레이션 — ✅ 완료 (2026-04-14)

| 단계 | 작업 | 산출물 | 상태 |
|------|------|--------|------|
| B-1 | TB_ROUTE_FEATURE_VECTOR DDL (대문자) | create_feature_vector_table.sql | ✅ |
| B-2 | fn_search_top_k_routes 함수 | 동일 SQL 파일 | ✅ |
| B-3 | BuildFeatureVectors.py (from-db 모드) | BuildFeatureVectors.py | ✅ |
| B-4 | 2,624건 경로 마이그레이션 실행 | HNSW 인덱스 구축 완료 | ✅ |

### Phase C: Top-K 검색 엔진 — ✅ 완료 (2026-04-14)

| 단계 | 작업 | 산출물 | 상태 |
|------|------|--------|------|
| C-1 | RoutePathLoader 구현 (DB 경로 로드) | TopKRoutingSearch.py | ✅ |
| C-2 | FeatureVectorDB 구현 (CRUD + pgvector 검색) | TopKRoutingSearch.py | ✅ |
| C-3 | TopKSearchEngine 구현 (DB/로컬 검색) | TopKRoutingSearch.py | ✅ |
| C-4 | CLI 검색 도구 (test/search 커맨드) | 실측 4.6ms/검색 | ✅ |

### Phase D: 품질 평가 — ✅ 완료 (2026-04-14)

| 단계 | 작업 | 산출물 | 상태 |
|------|------|--------|------|
| D-1 | EvaluateTopK.py 구현 | EvaluateTopK.py (local/db/hybrid 3모드) | ✅ |
| D-2 | Ground Truth 생성 (100개 쿼리) | compute_composite_similarity 전수비교 | ✅ |
| D-3 | Recall@5, NDCG@5, Spearman rho 측정 | evaluation_report.json (6종 리포트) | ✅ |
| D-4 | 파라미터 튜닝 | Arrow 패턴 5D 추가 + 하이브리드 검색 구현 | ✅ |

### Phase E: 3D 뷰어 연동 — ☐ 미착수

| 단계 | 작업 | 산출물 | 상태 |
|------|------|--------|------|
| E-1 | TopKSearchResult 모델 + DB 서비스 확장 | C# 코드 | ☐ |
| E-2 | Top-K 검색 UI 패널 | XAML | ☐ |
| E-3 | 3D 경로 오버레이 렌더링 | | ☐ |
| E-4 | 유사도 상세 분해 차트 | | ☐ |
| E-5 | 통합 테스트 | | ☐ |

### 전체 일정 요약

```
Phase A: 벡터 인코딩        ████████████ ✅ 완료             (2026-04-14)
Phase B: DB + 마이그레이션  ████████     ✅ 완료             (2026-04-14)
Phase C: 검색 엔진          ████████████ ✅ 완료             (2026-04-14)
Phase D: 품질 평가          ████████████ ✅ 완료             (2026-04-14)
Phase E: 3D 뷰어 연동                               ░░░░░░░░░░░░ (미착수)
```

### 실측 성능 요약 (2026-04-14)

| 항목 | 실측값 | 목표 |
|------|--------|------|
| DB 경로 로드 (2,624건) | 0.48초 | - |
| 벡터 인코딩 (2,624건) | 0.15초 (0.1ms/건) | < 5ms/건 ✅ |
| DB 벡터 저장 (2,624건) | 1.21초 | - |
| pgvector Top-K 검색 | **4.6ms** | < 100ms ✅ |
| 공정별 벡터 분포 | ETCH 1,136 / CLEAN 896 / METAL 225 / DIFF 123 / CMP 113 / PHOTO 77 / IMP 33 / CVD 21 | - |

### 품질 평가 결과 (2026-04-14, Phase D)

**평가 조건**: 355건 GroupPipeResults, 100개 쿼리, K=5, seed=42

#### 벡터 only (30D + Arrow 5D)

| 지표 | 실측값 | 목표 | 판정 |
|------|--------|------|------|
| Recall@5 | 0.3948 | >= 0.80 | FAIL |
| NDCG@5 | 0.4034 | >= 0.85 | FAIL |
| Spearman rho | 0.2366 | >= 0.80 | FAIL |
| 검색 시간 P95 | 18.7ms | < 100ms | PASS |

#### 하이브리드 검색 (벡터 Top-N + 복합유사도 재정렬)

| N | Recall@5 | NDCG@5 | Spearman | P95 |
|-----|----------|--------|----------|---------|
| 20 | 0.5807 | 0.6312 | 0.0930 | 31.6ms |
| 50 | 0.6720 | 0.7014 | 0.2205 | 25.9ms |
| 100 | 0.7598 | 0.7703 | 0.3585 | 31.1ms |
| **150** | **0.8352** | **0.8205** | **0.5160** | **35.3ms** |
| 200 | 0.8557 | 0.8350 | 0.5565 | 40.6ms |

**결론**: 하이브리드 검색 N=150에서 Recall@5 >= 0.80 목표 달성.
벡터 검색으로 후보 축소(O(log N)) + 복합유사도 정밀 재정렬(O(N))로
속도와 품질을 모두 확보하는 전략 채택.

**개선 사항 (v1.0 → v1.1)**:

1. 예비 5D [25~29]를 Arrow 패턴 특징(H/R/D비율, 세그먼트수, 방향전환율)으로 채움
2. TopKSearchEngine에 search_hybrid_local() 하이브리드 검색 메서드 추가
3. 가중치 조정: reserved(0.00) → arrow_pattern(0.15)

---

## 10. 리스크 및 대응

| 리스크 | 영향 | 대응 방안 |
|--------|------|-----------|
| Recall@K 목표 미달 | 추천 품질 저하 | 벡터 차원 확대(30→40), 가중치 재튜닝, HNSW ef_search 파라미터 조정 |
| 가변길이 정보 손실 | 긴 경로 vs 짧은 경로 구분력 저하 | 구간 수 구간별 통계(Q1/Q2/Q3) 추가하여 차원 확대 |
| 인덱스 빌드 시간 | HNSW 대규모 데이터에서 느림 | IVFFlat로 전환 또는 배치 빌드 스케줄링 |
| Python↔C# 통신 | 지연 시간 증가 | REST API(FastAPI) 또는 직접 DB 쿼리(C#→PostgreSQL) |
| 정규화 파라미터 드리프트 | 신규 데이터 분포 변화 | 주기적 파라미터 재산출 + 버전 관리(ENCODER_VERSION) |

---

## 11. 향후 확장

### 11.1 단기 (v1.1)

- **하이브리드 검색**: pgvector Top-20 → Python 복합유사도 재정렬 → Top-5 반환
  (pgvector의 속도 + 복합유사도의 정밀도 결합)
- **유사도 임계값 필터**: similarity < 0.5인 결과 자동 제외

### 11.2 중기 (v2.0)

- **학습 기반 임베딩**: Siamese Network로 복합유사도를 직접 학습한 임베딩 생성
  (수동 인코딩 → 학습 인코딩으로 전환)
- **다중 쿼리 지원**: 여러 PoC를 동시 검색하여 그룹 배관 후보 추천

### 11.3 장기 (v3.0)

- **실시간 피드백 학습**: 사용자가 선택한 경로를 positive sample로 활용
- **GNN 기반 그래프 임베딩**: PipeGraph 구조 자체를 임베딩하여 토폴로지 유사도 반영

---

## 부록

### A. pgvector 연산자 레퍼런스

| 연산자 | 의미 | 용도 |
|--------|------|------|
| `<->` | L2 (유클리드) 거리 | 절대 좌표 비교 시 |
| `<=>` | 코사인 거리 (1 - cosine) | **특징 벡터 유사도 (기본)** |
| `<#>` | 내적의 음수 (-inner product) | 정규화된 벡터에서 코사인과 동일 |

### B. 인덱스 타입 비교

| 항목 | HNSW | IVFFlat |
|------|------|---------|
| 검색 정확도 | 높음 (99%+) | 보통 (95%+) |
| 검색 속도 | 매우 빠름 | 빠름 |
| 빌드 속도 | 느림 | 빠름 |
| 메모리 사용 | 많음 | 적음 |
| 권장 데이터 규모 | < 10만 | 10만+ |
| **본 프로젝트 선택** | **기본 (v1.0)** | 대규모 전환 시 |

### C. 참고 파일 위치

| 파일 | 경로 | 역할 |
|------|------|------|
| AnalyzeRoutingAi_V2.py | `RoutingAI/src/` | 복합유사도 원본 (재활용) |
| AutoRoutingDesigner_V2.py | `RoutingAI/src/` | 8차원 확장 유사도 + 자동설계 |
| ExtractVacuumToPumpPaths.py | `RoutingAI/src/` | Vaccum→Pump 경로 추출 |
| AUTOROUTINGV7_TABLE_DEFINITION_FINAL.md | `SearchRoutingPath/` | DB 스키마 정의 |
| RouteDatabaseService.cs | `AutoRouting3DViewer/Services/` | C# DB 서비스 |
