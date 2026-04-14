-- ============================================================
-- Top-K 유사배관 검색을 위한 특징벡터 테이블 DDL
-- 대상 DB: AUTOROUTINGV7 (PostgreSQL + pgvector)
-- 생성일: 2026-04-14
-- ============================================================

-- 0. pgvector 확장 설치 (최초 1회)
CREATE EXTENSION IF NOT EXISTS vector;

-- ============================================================
-- 1. 특징 벡터 테이블
-- ============================================================
CREATE TABLE IF NOT EXISTS "TB_ROUTE_FEATURE_VECTOR" (
    -- PK
    "FEATURE_VECTOR_ID"    SERIAL          PRIMARY KEY,

    -- FK: 기존 경로 테이블 참조
    "ROUTE_PATH_GUID"      CHAR(36)        NOT NULL,

    -- 경로 식별 (빠른 필터링용)
    "PROCESS_NAME"         VARCHAR(50),     -- CMP, ETCH, DIFF, ...
    "EQUIPMENT_NAME"       VARCHAR(100)    NOT NULL,
    "UTILITY_GROUP"        VARCHAR(50),     -- Vaccum, Gas, Chemical, ...
    "UTILITY"              VARCHAR(50),     -- Foreline, PV_VENT, AKWW, ...
    "SIZE"                 VARCHAR(20),     -- 20A, 50A, 100A, ...

    -- PoC 위치 정보 (검색 조건)
    "START_POSX"           DOUBLE PRECISION,
    "START_POSY"           DOUBLE PRECISION,
    "START_POSZ"           DOUBLE PRECISION,
    "END_POSX"             DOUBLE PRECISION,
    "END_POSY"             DOUBLE PRECISION,
    "END_POSZ"             DOUBLE PRECISION,

    -- 경로 요약 (결과 표시용)
    "DIRECTION_PATTERN"    VARCHAR(200),    -- "R-H-D-R"
    "TOTAL_LENGTH_MM"      DOUBLE PRECISION,
    "STEP_COUNT"           INTEGER,
    "START_LEVEL"          VARCHAR(10),     -- CSF, A, F, CR
    "END_LEVEL"            VARCHAR(10),

    -- 30차원 특징 벡터 (확정안: 기하학 중심)
    -- [0~2]  시작 위상 (출발 방향 벡터)     w=0.20
    -- [3~5]  종단 위상 (진입 방향 벡터)     w=0.20
    -- [6~8]  공간 변위 (시작→종단 상대위치)  w=0.15
    -- [9~11] 바운딩 박스 (3D 공간 비율)     w=0.15
    -- [12~14] 구간 1 (초기 33% 꺾임)       w=0.06
    -- [15~17] 구간 2 (중기 33% 꺾임)       w=0.06
    -- [18~20] 구간 3 (후기 33% 꺾임)       w=0.06
    -- [21~24] 환경/비용 (길이,꺾임,장애물,서포트) w=0.12
    -- [25~29] 예비 (0.0 패딩)
    "FEATURE_VECTOR"       vector(30)      NOT NULL,

    -- 인코딩 메타데이터
    "ENCODER_VERSION"      VARCHAR(20)     DEFAULT 'v1.0',
    "ENCODED_AT"           TIMESTAMPTZ     DEFAULT NOW(),

    -- 원본 특징량 (디버깅/재인코딩용)
    "RAW_FEATURES_JSON"    JSONB
);

-- ============================================================
-- 2. pgvector 코사인 거리 인덱스 (HNSW)
-- ============================================================
CREATE INDEX IF NOT EXISTS "IDX_FEATURE_VECTOR_HNSW"
    ON "TB_ROUTE_FEATURE_VECTOR"
    USING hnsw ("FEATURE_VECTOR" vector_cosine_ops)
    WITH (m = 16, ef_construction = 200);

-- IVFFlat 대안 (10만 건 이상 시 HNSW 대신 검토)
-- CREATE INDEX "IDX_FEATURE_VECTOR_IVFFLAT"
--     ON "TB_ROUTE_FEATURE_VECTOR"
--     USING ivfflat ("FEATURE_VECTOR" vector_cosine_ops)
--     WITH (lists = 100);

-- ============================================================
-- 3. 필터링용 인덱스
-- ============================================================
CREATE INDEX IF NOT EXISTS "IDX_FV_PROCESS_EQUIP"
    ON "TB_ROUTE_FEATURE_VECTOR" ("PROCESS_NAME", "EQUIPMENT_NAME");

CREATE INDEX IF NOT EXISTS "IDX_FV_UTILITY"
    ON "TB_ROUTE_FEATURE_VECTOR" ("UTILITY_GROUP", "UTILITY");

CREATE INDEX IF NOT EXISTS "IDX_FV_SIZE"
    ON "TB_ROUTE_FEATURE_VECTOR" ("SIZE");

-- ============================================================
-- 4. 뷰: Top-K 검색 결과 + 경로 메타 조인
-- ============================================================
CREATE OR REPLACE VIEW "VW_ROUTE_SEARCH" AS
SELECT
    fv."FEATURE_VECTOR_ID",
    fv."ROUTE_PATH_GUID",
    fv."PROCESS_NAME",
    fv."EQUIPMENT_NAME",
    fv."UTILITY_GROUP",
    fv."UTILITY",
    fv."SIZE",
    fv."START_POSX", fv."START_POSY", fv."START_POSZ",
    fv."END_POSX",   fv."END_POSY",   fv."END_POSZ",
    fv."DIRECTION_PATTERN",
    fv."TOTAL_LENGTH_MM",
    fv."STEP_COUNT",
    fv."START_LEVEL",
    fv."END_LEVEL",
    fv."FEATURE_VECTOR",
    fv."ENCODER_VERSION",
    fv."ENCODED_AT"
FROM "TB_ROUTE_FEATURE_VECTOR" fv;

-- ============================================================
-- 5. Top-K 검색 함수
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
        fv."ROUTE_PATH_GUID",
        fv."EQUIPMENT_NAME",
        fv."UTILITY_GROUP",
        fv."UTILITY",
        fv."DIRECTION_PATTERN",
        fv."TOTAL_LENGTH_MM",
        fv."STEP_COUNT",
        fv."START_LEVEL",
        fv."END_LEVEL",
        (fv."FEATURE_VECTOR" <=> p_query_vector)::DOUBLE PRECISION AS cosine_dist,
        (1.0 - (fv."FEATURE_VECTOR" <=> p_query_vector))::DOUBLE PRECISION AS sim_score,
        ROW_NUMBER() OVER (
            ORDER BY fv."FEATURE_VECTOR" <=> p_query_vector
        )::INTEGER AS rank_pos
    FROM "TB_ROUTE_FEATURE_VECTOR" fv
    WHERE
        (p_utility_group IS NULL OR fv."UTILITY_GROUP" = p_utility_group)
        AND (p_utility IS NULL OR fv."UTILITY" = p_utility)
        AND (p_size IS NULL OR fv."SIZE" = p_size)
        AND (p_process IS NULL OR fv."PROCESS_NAME" = p_process)
    ORDER BY fv."FEATURE_VECTOR" <=> p_query_vector
    LIMIT p_k;
END;
$$;
