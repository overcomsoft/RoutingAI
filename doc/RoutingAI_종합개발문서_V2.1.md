# RoutingAI 종합 개발 문서 (V2.1)

> 최종 업데이트: 2026-04-14  
> 프로젝트: AI 기반 3D 배관 자동설계 시스템  
> 위치: `KGraphGen03/Analyzer/RoutingAI/`

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [프로젝트 구조](#2-프로젝트-구조)
3. [3-Phase 파이프라인 전체 흐름도](#3-3-phase-파이프라인-전체-흐름도)
4. [Phase 1: 경로 추출 및 장애물관계 분석](#4-phase-1-경로-추출-및-장애물관계-분석)
5. [Phase 2: 유사도 기반 그룹 클러스터링](#5-phase-2-유사도-기반-그룹-클러스터링)
6. [Phase 3: 장애물 인식 자동 경로 설계](#6-phase-3-장애물-인식-자동-경로-설계)
7. [공간장애물 처리 체계](#7-공간장애물-처리-체계)
8. [유사도 분석 상세](#8-유사도-분석-상세)
9. [보조 분석 모듈](#9-보조-분석-모듈)
10. [데이터베이스 적재 모듈](#10-데이터베이스-적재-모듈)
11. [3D 시각화](#11-3d-시각화)
12. [실행 명령어 모음](#12-실행-명령어-모음)
13. [데이터 디렉토리 구조](#13-데이터-디렉토리-구조)
14. [버전 이력](#14-버전-이력)

---

## 1. 시스템 개요

### 1.1 핵심 철학

> **"비슷한 공간환경에서는 비슷한 배관경로가 최적"**

RoutingAI는 기존 배관 설계 데이터에서 경로를 추출하고, **장애물 유형별 공간관계가 유사한 기존 설계를 찾아 템플릿으로 활용**하여 신규 배관 경로를 자동 설계합니다. 단순한 기하학적 유사도뿐 아니라 **구조기둥·H빔·그레이팅·포스트 등 장애물 유형별 18개 공간관계 속성**을 분석에 반영합니다.

### 1.2 기술 스택

| 항목 | 기술 |
|------|------|
| 언어 | Python 3.10+ |
| 그래프 탐색 | BFS (너비 우선 탐색) |
| 클러스터링 | Union-Find (Disjoint Set) |
| 유사도 | 다차원 복합 유사도 (5차원/8차원) |
| 충돌 감지 | Slab Method (Ray-AABB Intersection) |
| 데이터베이스 | PostgreSQL + PostGIS (3D 공간 쿼리) |
| 시각화 | Plotly (Python) + Three.js (WebGL) |

### 1.3 입력 데이터

| 데이터 | 내용 | 예시 |
|--------|------|------|
| Nodes | 배관 네트워크 노드 (1,183~3,472개/장비) | POC, ELBOW, TEE, VALVE 등 |
| Edges | 배관 연결 정보 (879~2,588개/장비) | 배관 세그먼트 |
| Equipment | 장비 정보 (BBox, POC 목록) | KSCTA01, WTNHJ02 등 7종 |
| Obstacles | 공간 장애물 (456~3,001개/장비) | 기둥, 빔, 그레이팅 등 |
| SpaceInfo | 공간 레벨 (CSF/A/F/CR) | 층별 Z범위 |

---

## 2. 프로젝트 구조

```
RoutingAI/
├── src/                          # 소스 코드 (14개 Python 모듈)
│   │
│   │  ── 핵심 3-Phase 엔진 ──
│   ├── AnalyzeRoutingAi_V2.py         (43KB)  Phase 1+2: 경로 추출 + 그룹 클러스터링
│   ├── AutoRoutingDesigner_V2.py      (94KB)  Phase 3: 장애물 인식 자동 경로 설계
│   │
│   │  ── 보조 분석 모듈 ──
│   ├── AnalyzeRoutingPath.py          (53KB)  BFS 경로 추출 V1 (레거시)
│   ├── AnalyzeBranching.py            (18KB)  분기점(Tee/Cross) 분석
│   ├── analyze_group_pipes.py         (34KB)  공차 기반 배관 그룹화
│   ├── duct_poc_clustering.py         (18KB)  덕트 PoC 클러스터링
│   ├── ExtractVacuumToPumpPaths.py    (32KB)  Vacuum→Pump 경로 추출
│   │
│   │  ── DB 적재 모듈 ──
│   ├── import_obstacles_json.py       (22KB)  BIM 4종 데이터 통합 임포트
│   ├── import_duct_poc_cluster.py     (12KB)  덕트 PoC 클러스터 DB 적재
│   ├── import_pipe_group_analyze.py   (11KB)  배관 그룹 분석 결과 DB 적재
│   ├── import_objects.py              (10KB)  BIM 객체 요약 DB 적재
│   │
│   │  ── 시각화 ──
│   └── VisualizeRouting3D.py          (7KB)   Plotly 3D 시각화
│
├── data/
│   ├── input/                  # 원본 설계 JSON (8개 장비, ~20MB)
│   ├── RoutingResults/         # Phase 1 결과 (30개 경로 JSON, ~16MB)
│   ├── GroupPipeResults/       # Phase 2 결과 (6쌍 JSON+CSV, ~1.2MB)
│   ├── AutoRoutingResults/     # Phase 3 결과 (5개 테스트, ~100KB)
│   └── VacuumPumpPaths/        # Vacuum 경로 (4개 JSON, ~150KB)
│
├── visualization/              # Three.js 3D 웹 뷰어 (2개 HTML)
├── doc/                        # 문서 (8개 파일, 492KB)
├── README.md                   # 프로젝트 가이드
└── requirements.txt            # Python 의존성
```

---

## 3. 3-Phase 파이프라인 전체 흐름도

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                                                                             │
│  원본 설계 JSON (Nodes, Edges, Equipment, Obstacles, SpaceInfo)            │
│  data/input/*.json                                                         │
│                                                                             │
│      │                                                                      │
│      ▼                                                                      │
│  ╔═══════════════════════════════════════════════════════════════╗          │
│  ║  Phase 1: 경로 추출 (AnalyzeRoutingAi_V2.py)                ║          │
│  ║                                                               ║          │
│  ║  1. RoutingGraph.load_from_json()  ─ 노드/엣지/장비 인덱싱  ║          │
│  ║  2. SpatialContext.load_from_json() ─ 장애물 6종 분류       ║          │
│  ║  3. BFS 경로 탐색 (POC→종단)       ─ 분기/사이클 처리       ║          │
│  ║  4. _compute_path_features()        ─ arrow/vector/range     ║          │
│  ║  5. ObstacleRelationExtractor()     ─ 장애물관계 18속성      ║          │
│  ╚═══════════════════════════════════════════════════════════════╝          │
│      │                                                                      │
│      ▼  RoutingResults/*.json                                               │
│                                                                             │
│  ╔═══════════════════════════════════════════════════════════════╗          │
│  ║  Phase 2: 그룹 클러스터링 (AnalyzeRoutingAi_V2.py)          ║          │
│  ║                                                               ║          │
│  ║  1. _load_routing_records()         ─ Phase1 결과 로딩       ║          │
│  ║  2. (장비, 유틸리티, 사이즈)별 버킷  ─ 분류                  ║          │
│  ║  3. compute_composite_similarity()   ─ 5차원 유사도 계산     ║          │
│  ║     (arrow 0.25 + vector 0.25 + range 0.15                   ║          │
│  ║      + length 0.15 + obstacle_relations 0.20)                 ║          │
│  ║  4. Union-Find 클러스터링            ─ threshold ≥ 0.70      ║          │
│  ║  5. detect_zones()                   ─ trunk/fan-in/fan-out  ║          │
│  ╚═══════════════════════════════════════════════════════════════╝          │
│      │                                                                      │
│      ▼  GroupPipeResults/*.json                                             │
│                                                                             │
│  ╔═══════════════════════════════════════════════════════════════╗          │
│  ║  Phase 3: 자동 경로 설계 (AutoRoutingDesigner_V2.py)         ║          │
│  ║                                                               ║          │
│  ║  1. SpatialContext 로드              ─ 장비/장애물/레벨      ║          │
│  ║  2. find_matching_group()            ─ 8차원 유사도 매칭     ║          │
│  ║     (arrow 0.12 + vector 0.12 + range 0.08 + length 0.08    ║          │
│  ║      + equip 0.15 + terminal 0.15 + obstacle 0.20           ║          │
│  ║      + level 0.10)                                           ║          │
│  ║  3. ObstacleAwarePathBuilder()       ─ 3단계 경로 생성       ║          │
│  ║     [FAN-IN] → [TRUNK] → [FAN-OUT]                           ║          │
│  ║  4. _apply_obstacle_avoidance()      ─ 구조기둥 회피(최대5회)║          │
│  ║  5. PathValidatorV2.validate()       ─ 5항목 검증            ║          │
│  ╚═══════════════════════════════════════════════════════════════╝          │
│      │                                                                      │
│      ▼  AutoRoutingResults/*.json                                           │
│                                                                             │
│  ╔═══════════════════════════════════════╗                                  │
│  ║  시각화 (HTML 3D Viewer)             ║                                  │
│  ║  VisualizeAutoRouting3D.html         ║                                  │
│  ╚═══════════════════════════════════════╝                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Phase 1: 경로 추출 및 장애물관계 분석

### 4.1 실행

```bash
python AnalyzeRoutingAi_V2.py --phase routing --input ./data/input
```

### 4.2 핵심 클래스: RoutingGraph

```
RoutingGraph
├── load_from_json(file_path)
│   ├── node_by_guid: Dict[str, Dict]     ← GUID→노드 딕셔너리
│   ├── edge_by_guid: Dict[str, Dict]     ← GUID→엣지 딕셔너리
│   ├── equipment_list: List[Dict]         ← 장비 목록
│   └── poc_owner_map: Dict[str, List]     ← POC→소속장비 매핑
│
├── get_neighbors(node_guid)               ← 인접 노드 조회
│   ├── VIRTUAL 엣지 제외
│   ├── connectionGuidList 검증
│   └── 중복 이웃 제거
│
├── is_branch_node(node_guid)              ← 분기 노드 판정
│   └── TEE/BRANCH/JUNCTION/CROSS/WYE 또는 이웃≥3
│
├── find_routing_paths()                   ← 전체 POC 경로 추출
│   └── _trace_paths_from_poc()            ← BFS 단일 POC 탐색
│       ├── 큐 크기 한도: max_queue_size (100,000)
│       ├── 최대 깊이: max_depth (512)
│       ├── POC별 최대 경로: max_paths_per_poc (5,000)
│       ├── _classify_terminal()           ← 종단 판정
│       │   ├── 다른 장비 PoC 도달
│       │   ├── Duct / TakeOff 도달
│       │   ├── Lateral Pipe 도달
│       │   ├── Nozzle PoC 도달
│       │   ├── 종단 PoC 도달
│       │   ├── 장비 노드 도달
│       │   └── 배관 끝단(막힘)
│       └── _reconstruct_path()            ← 부모 포인터 역추적
│
└── _compute_path_features(steps)          ← 경로 특징량 계산
    ├── path_arrow: "H-R-H-R-H"           ← 방향 코드열
    ├── path_step_vectors: [{x,y,z}, ...]  ← 이동 벡터
    ├── path_step_lengths: [200.0, ...]    ← 구간 길이
    ├── path_range: {x, y, z}              ← BBox 범위
    ├── h_segments: [{mean_z, mid_xy}, ...]← 수평 구간
    └── obstacle_relations: {...18속성}    ← 장애물관계
```

### 4.3 방향 코드 분류 (path_arrow)

| 코드 | 의미 | 판정 기준 |
|:----:|------|----------|
| **R** | Riser (수직) | 수직각도 > 85° |
| **H** | Header (수평) | 수직각도 < 5° |
| **D** | Diagonal (대각) | 5° ≤ 수직각도 ≤ 85° |

예: `"R-H-R-H-H-R"` = 수직→수평→수직→수평→수평→수직

### 4.4 출력 구조 (RoutingResults/*.json)

```json
{
  "source_file": "CMP_KSCTA01_*.json",
  "equipment_name": "kscta01",
  "poc_paths": [
    {
      "start_poc_id": "POC-GUID-...",
      "paths": [
        {
          "terminal_label": "Duct / TakeOff 도달",
          "path_arrow": "R-H-R-H-H-R",
          "path_range": {"x": 2500.0, "y": 3200.0, "z": 4100.0},
          "path_step_vectors": [{"x": 0, "y": 0, "z": -2100}, ...],
          "path_step_lengths": [2100.0, 1500.0, ...],
          "obstacle_relations": {
            "col_count_nearby": 5,
            "col_min_distance": 342.1,
            "col_crossings": 2,
            "col_relative_pattern": "LRB",
            "beam_count_crossing": 3,
            "beam_min_clearance": 450.0,
            "grating_coverage": 0.72,
            ...
          }
        }
      ]
    }
  ]
}
```

---

## 5. Phase 2: 유사도 기반 그룹 클러스터링

### 5.1 실행

```bash
python AnalyzeRoutingAi_V2.py --phase grouping --routing_out ./RoutingResults
```

### 5.2 핵심 클래스: GroupAnalyzer

```
GroupAnalyzer
├── find_groups()
│   ├── 1. 버킷 분류: (장비명, 장비ID, Process, Maker, 유틸리티, 사이즈)
│   ├── 2. 유사도 행렬 계산: compute_composite_similarity()
│   ├── 3. Union-Find 클러스터링 (threshold ≥ 0.70)
│   ├── 4. 검증: XY 근접성 (max 5,000mm) + 공통 Z 레벨
│   └── 5. 정렬: 경로수 내림 → 유사도 내림 → 장비명
│
└── _find_common_z_levels(recs)
    ├── 수평(H) 구간의 mean_z 수집
    ├── Z 허용오차(200mm) 내 클러스터 확장
    └── 경로수 + xy_spread 함께 저장
```

### 5.3 5차원 복합 유사도 (compute_composite_similarity)

```
복합유사도 = 0.25 × arrow_sim
           + 0.25 × vector_sim
           + 0.15 × range_sim
           + 0.15 × length_sim
           + 0.20 × obstacle_relation_sim
```

| 차원 | 가중치 | 계산 방법 |
|------|:------:|----------|
| Arrow | 0.25 | Levenshtein Distance 기반 방향패턴 유사도 |
| Vector | 0.25 | DTW식 벡터 시퀀스 코사인 유사도 (길이 패널티 포함) |
| Range | 0.15 | BBox XYZ 범위의 축별 비율 비교 |
| Length | 0.15 | 총 경로 길이 비율 비교 |
| Obstacle Relations | 0.20 | 유형별 가중 합산 (아래 상세) |

### 5.4 장애물관계 유사도 내부 가중치

```
obstacle_relation_sim = 0.35 × 기둥관계_sim
                      + 0.30 × 빔관계_sim
                      + 0.20 × 그레이팅관계_sim
                      + 0.15 × 포스트관계_sim
```

#### 기둥관계 유사도 (0.35) 내부:
- 근접 수 유사도 × 0.2
- 최소 거리 유사도 × 0.2
- 교차 수 유사도 × 0.3
- LR패턴 유사도 × 0.3

#### 빔관계 유사도 (0.30) 내부:
- 교차 수 유사도 × 0.4
- 최소 clearance 유사도 × 0.3
- 평행 비율 유사도 × 0.3

#### 그레이팅관계 유사도 (0.20) 내부:
- 커버리지 유사도 × 0.4
- 하부 수 유사도 × 0.3
- 개구부 수 유사도 × 0.3

#### 포스트관계 유사도 (0.15) 내부:
- 밀도 유사도 × 0.3
- 그리드 정렬도 유사도 × 0.4
- 근접 수 유사도 × 0.3

### 5.5 Zone 탐지 (detect_zones)

```
        시작점(POC)                              종단점(Duct/Lateral)
          │                                          │
          ▼                                          ▼
    ┌──────────┐    ┌──────────────────┐    ┌──────────┐
    │ FAN-IN   │───▶│     TRUNK        │───▶│ FAN-OUT  │
    │ (수직이동) │    │ (수평 주통로)     │    │ (수직이동) │
    │ Z 변화大  │    │ XY이동, Z일정    │    │ Z 변화大  │
    └──────────┘    └──────────────────┘    └──────────┘
```

- **TRUNK**: 공통 Z 레벨 대역에서 xy_spread가 가장 작은 구간
- **FAN-IN**: TRUNK 이전 구간 (시작→트렁크 진입)
- **FAN-OUT**: TRUNK 이후 구간 (트렁크→목적지)

---

## 6. Phase 3: 장애물 인식 자동 경로 설계

### 6.1 실행

```bash
python AutoRoutingDesigner_V2.py \
    --json_input ./data/input/CMP_KSCTA01_*.json \
    --group_results ./GroupPipeResults/group_pipe_results_20260405194814.json \
    --equipment kscta01 --utility AKWW --size 20A \
    --start 5500,28200,15495 --dest 5300,27000,15495
```

### 6.2 설계 파라미터 (DesignConfigV2)

| 파라미터 | 기본값 | 설명 |
|---------|:------:|------|
| obstacle_clearance | 200mm | 배관-장애물 최소 이격거리 |
| obstacle_detour_margin | 300mm | 우회 경로 추가 여유 |
| max_start_xy_distance | 8,000mm | 템플릿 매칭 최대 XY 검색 범위 |
| fitting_length_threshold | 150mm | 피팅(짧은 수직) 구간 임계값 |
| min_segment_length | 50mm | 최소 세그먼트 길이 |
| trunk_approach_tolerance | 300mm | 트렁크 Z 대역 허용 오차 |

### 6.3 8차원 확장 유사도 (SpatialSimilarity)

```
확장유사도 = 0.12 × arrow_sim          ← 방향 패턴
           + 0.12 × vector_sim         ← 벡터 시퀀스
           + 0.08 × range_sim          ← 공간 범위
           + 0.08 × length_sim         ← 경로 길이
           + 0.15 × equip_relative_sim ← 장비 상대 좌표 (NEW)
           + 0.15 × terminal_sim       ← 종단점 매칭 (NEW)
           + 0.20 × obstacle_sim       ← 장애물관계
           + 0.10 × level_sim          ← SpaceInfo 레벨 (NEW)
```

#### 장비 상대 좌표 유사도 (equip_relative_sim, 0.15):
- 시작점 상대좌표 유사도 × 0.4
- 종점 상대좌표 유사도 × 0.4
- 출발면(N/S/E/W/T/B) 일치 보너스 × 0.2

#### 종단점 유사도 (terminal_sim, 0.15):
- 종단 타입 일치 × 0.4 (BRANCH/DUCT/LATERAL/EQUIPMENT)
- 종단 거리 유사도 × 0.3
- 종단 레벨 일치 × 0.3

#### 레벨 유사도 (level_sim, 0.10):
- 시작 레벨 일치 × 0.25
- 끝 레벨 일치 × 0.25
- 레벨 변경 횟수 유사도 × 0.25
- 경유 레벨 Jaccard × 0.25

### 6.4 장애물 회피 경로 생성 (ObstacleAwarePathBuilder)

```
입력: 시작점, 도착점, 그룹 정보, 템플릿 경로

    1. Zone 분류: 템플릿의 fan-in/trunk/fan-out 세그먼트 식별
    2. 회전각도 계산: 템플릿→새방향 각도차 (atan2)
    3. 트렁크 진입/이탈점 계산

    4. 3단계 벡터 생성:
       ┌───────────────────────────────────────────────────────┐
       │ Stage 1: FAN-IN                                       │
       │   - R(수직) 세그먼트: Z축 스케일링                    │
       │   - H(수평) 세그먼트: XY 회전 + 스케일링 + 직교 스냅  │
       │   - 피팅(<150mm): 원본 유지, Z방향만 보정              │
       ├───────────────────────────────────────────────────────┤
       │ Stage 2: TRUNK                                        │
       │   - 진입→이탈 XY 변위를 H세그먼트에 비례 분배         │
       │   - 직교 스냅 (X 또는 Y 우세 방향 선택)               │
       ├───────────────────────────────────────────────────────┤
       │ Stage 3: FAN-OUT                                      │
       │   - FAN-IN과 동일 로직 (역방향)                       │
       └───────────────────────────────────────────────────────┘

    5. 장애물 회피 (_apply_obstacle_avoidance):
       ┌───────────────────────────────────────────────────────┐
       │ 대상: COLUMN_STRUCTURE(구조기둥)만                     │
       │ 조건: H(수평) 세그먼트만 (R 세그먼트 무시)             │
       │ 제한: 경로당 최대 5회 우회                             │
       │                                                       │
       │ 우회 로직:                                            │
       │   X축 이동 우세 → Y방향 오프셋 후 X이동 후 복귀       │
       │   Y축 이동 우세 → X방향 오프셋 후 Y이동 후 복귀       │
       │   오프셋 크기 = 장애물 크기/2 + detour_margin(300mm)  │
       └───────────────────────────────────────────────────────┘

    6. 끝점 보정: 실제 종점과 목적지 간 갭 보정 벡터 추가
    7. 벡터 정리: 영벡터 제거 + 연속 동일방향 병합
```

### 6.5 경로 검증 (PathValidatorV2)

| 검증 항목 | 가중치 | 내용 |
|----------|:------:|------|
| 좌표 연속성 | 0.20 | 벡터 복원 좌표와 실제 좌표 일치 여부 |
| 길이 합리성 | 0.20 | 생성/템플릿 길이 비율 (0.2~2.5 범위) |
| 방향 패턴 | 0.15 | 생성 경로와 템플릿의 arrow 유사도 |
| 장애물 충돌 | 0.25 | COLUMN_STRUCTURE 충돌 세그먼트 수 |
| Zone 준수 | 0.20 | 생성 경로가 TRUNK 영역 범위 내 유지 |

**판정**: 종합 quality ≥ 0.4 AND 연속성 = 1.0 → **PASS**

---

## 7. 공간장애물 처리 체계

### 7.1 장애물 6종 분류 (ObstacleCategory)

| 카테고리 | BIM ddworksType | 자동설계 시 처리 |
|---------|----------------|----------------|
| `STRUCTURAL_COLUMN` | COLUMN_STRUCTURE | **회피 대상** (경로 우회) |
| `POST` | COLUMN_ARCHITECTURE | 유사도 반영 (회피 안 함) |
| `H_BEAM` | BEAM_ARCHITECTURE | 유사도 반영 (수직간격 확인) |
| `STRUCTURAL_BEAM` | BEAM_STRUCTURE | 유사도 반영 |
| `GRATING` | FLOOR_ARCHITECTURE | 유사도 반영 (하부 커버리지) |
| `CEILING` | CEILING_ARCHITECTURE | 분류만 (분석 미반영) |

### 7.2 장애물관계 18개 특징량 상세

#### 구조기둥 관계 (5개)

| 속성 | 타입 | 의미 | 계산 방법 |
|------|------|------|----------|
| `col_count_nearby` | int | 인접 기둥 수 | 경로 중심 기준 반경 내 기둥 카운트 |
| `col_min_distance` | float | 최근접 기둥 거리(mm) | 기둥 BBox와 경로 중심 간 최소 거리 |
| `col_avg_distance` | float | 평균 기둥 거리(mm) | 반경 내 기둥 거리 평균 |
| `col_crossings` | int | 경로-기둥 교차 수 | Slab Method (margin=200mm) |
| `col_relative_pattern` | str | 좌/우/사이 패턴 | 세그먼트 법선벡터에 기둥 중심 투영 → L/R/B |

**LR 패턴 알고리즘**:
```
각 세그먼트(p1→p2)에 대해:
  1. 진행 방향 벡터: (dx, dy) = (p2-p1)
  2. 좌측 법선 벡터: (nx, ny) = (-dy, dx) / |seg|
  3. 반경 내 장애물 중심(oc) 투영: cross = (oc-seg_center) · (nx, ny)
  4. cross > 0 → Left, cross < 0 → Right, 양쪽 모두 → Between
  5. 패턴 연결: "LRB", "LLR", "BBR" 등
```

#### 포스트 관계 (3개)

| 속성 | 타입 | 의미 | 계산 방법 |
|------|------|------|----------|
| `post_count_nearby` | int | 인접 포스트 수 | 경로 중심 반경 내 카운트 |
| `post_density` | float | 포스트 밀도 (0~1) | 포스트 수 / (BBox XY 면적 / 1e6) |
| `post_grid_alignment` | float | 그리드 정렬도 (0~1) | 50% 그리드 규칙성 + 50% 경로 직교율 |

**그리드 정렬도 알고리즘**:
```
그리드 규칙성 (50%):
  - 포스트 중심 X좌표 10mm 단위 반올림 → 간격 추출
  - 간격의 변동계수(CV) = σ/μ
  - 규칙성 = 1 - CV (0~1, CV 작을수록 규칙적)
  - X축 + Y축 평균

경로 직교율 (50%):
  - 각 세그먼트의 axis_ratio = max(|dx|, |dy|) / √(dx²+dy²)
  - 길이 가중 평균 (직교일수록 1에 가까움)
```

#### H-빔 관계 (3개)

| 속성 | 타입 | 의미 | 계산 방법 |
|------|------|------|----------|
| `beam_count_crossing` | int | 빔 교차 수 | Slab Method (margin=100mm) |
| `beam_min_clearance` | float | 최소 수직 간격(mm) | 빔 하단 Z - 경로 Z |
| `beam_parallel_ratio` | float | 평행 빔 비율 (0~1) | cos(빔방향, 경로방향) > 0.7인 비율 |

#### 그레이팅 관계 (3개)

| 속성 | 타입 | 의미 | 계산 방법 |
|------|------|------|----------|
| `grating_coverage` | float | 하부 커버리지 (0~1) | H세그먼트 중 그레이팅 위를 지나는 비율 |
| `grating_count_below` | int | 하부 그레이팅 수 | 반경 내 그레이팅 카운트 |
| `grating_gap_count` | int | 개구부 전환 수 | 커버→비커버 전환 횟수 |

**커버리지 판정 조건**:
- H(수평) 세그먼트만 대상 (Z변화 < 100mm)
- 세그먼트 중심이 그레이팅 XY 범위 내
- 그레이팅 상단 Z ≤ 경로 Z + 500mm

### 7.3 충돌 감지: Slab Method

```
선분(p1→p2)과 BBox(+margin) 교차 판정:

  d = p2 - p1              ← 선분 방향 벡터
  bmin = obs.min - margin  ← 확장된 BBox 최소점
  bmax = obs.max + margin  ← 확장된 BBox 최대점

  각 축(X, Y, Z)에 대해:
    if |d[i]| ≈ 0:         ← 축에 평행한 선분
      if p1[i] ∉ [bmin[i], bmax[i]]: return False
    else:
      t1 = (bmin[i] - p1[i]) / d[i]
      t2 = (bmax[i] - p1[i]) / d[i]
      t_min = max(t_min, min(t1, t2))
      t_max = min(t_max, max(t1, t2))
      if t_min > t_max: return False

  return True  ← 교차 발생
```

### 7.4 SpatialContext 공간 쿼리 메서드

| 메서드 | 용도 | 반환 |
|--------|------|------|
| `find_obstacles_in_path(p1, p2, clearance)` | 선분 경로 충돌 장애물 검색 | List[ObstacleInfo] |
| `find_nearby_obstacles(point, radius)` | 반경 내 장애물 검색 (거리순) | List[(dist, ObstacleInfo)] |
| `compute_obstacle_density(center, radius)` | 가중 장애물 밀도 (기둥=3배) | float (0~1) |
| `get_level_at_z(z)` | Z좌표의 공간 레벨 조회 | SpaceLevelInfo |

---

## 8. 유사도 분석 상세

### 8.1 Phase 2 vs Phase 3 유사도 비교

| 차원 | Phase 2 (5차원) | Phase 3 (8차원) |
|------|:--------------:|:--------------:|
| Arrow (방향패턴) | **0.25** | 0.12 |
| Vector (벡터시퀀스) | **0.25** | 0.12 |
| Range (공간범위) | 0.15 | 0.08 |
| Length (경로길이) | 0.15 | 0.08 |
| Obstacle Relations (장애물관계) | **0.20** | **0.20** |
| Equipment Relative (장비상대좌표) | - | **0.15** |
| Terminal (종단점) | - | **0.15** |
| Level (SpaceInfo 레벨) | - | **0.10** |

### 8.2 장애물관계 유형별 가중치 (Phase 2 = Phase 3 동일)

```
                구조기둥 0.35 ←────── 최우선: 반드시 회피해야 함
                   │
                H-빔 0.30 ←──────── 수직 제약: clearance 확보 필요
                   │
            그레이팅 0.20 ←──────── 지지 구조: 배관 경로에 영향
                   │
              포스트 0.15 ←──────── 규칙적 배치: 예측 가능, 영향 적음
```

### 8.3 개별 유사도 함수

| 함수 | 입력 | 알고리즘 |
|------|------|---------|
| `pattern_similarity()` | arrow1, arrow2 | Levenshtein Distance → 1 - edit/max(len) |
| `_cosine_similarity_0_1()` | vec1, vec2 | cos(θ) → max(0, cos) (음수 방향 = 0점) |
| `_vector_sequence_similarity()` | vecs1, vecs2 | 접두사 코사인 평균 × 길이 커버리지 |
| `_range_similarity()` | range1, range2 | 축별 1-|v1-v2|/max(v1,v2) 평균 |
| `_length_similarity()` | len1, len2 | 1-|l1-l2|/max(l1,l2) |

---

## 9. 보조 분석 모듈

### 9.1 AnalyzeRoutingPath.py (경로 추출 V1, 레거시)

- Phase 1의 원형. V2(AnalyzeRoutingAi_V2.py)에 통합되었으나 독립 실행 가능
- BFS 기반 POC→종단 경로 추출
- 분기점 메타데이터 (branch_depth, branch_segments) 제공

### 9.2 AnalyzeBranching.py (분기점 분석)

- BranchGraph 클래스: 배관 네트워크 위상(Topology) 분석
- 분기점 판정: `connectionGuidList` 크기 ≥ 3 (Tee=3, Cross=4)
- 터미널 추적: 분기점에서 각 방향으로 끝점(PoC, Takeoff, Nozzle) 탐색
- 출력: JSON(상세) + CSV(요약), 장비/유틸리티별 그룹화

### 9.3 analyze_group_pipes.py (공차 기반 그룹화)

- 3단계 클러스터링: Z축 공차(100mm) → 방향벡터 평행성(3°) → BFS 밀집공간(300mm)
- 수직/수평 배관 분류 후 각각 클러스터링
- 그룹별 통계: 바운딩 박스, 간격(Spacing), BOP(Bottom of Pipe), 고도

### 9.4 duct_poc_clustering.py (덕트 PoC 클러스터링)

- DUCT 타입 장비의 TakeOff 접속점 추출 + 접속면(TOP/LEFT/RIGHT) 판별
- 유틸리티별 PoC 간 유클리디안 거리 기반 클러스터링
- 클러스터 3D 바운딩 박스 + Range 계산
- CSV + Excel 내보내기

### 9.5 ExtractVacuumToPumpPaths.py (Vacuum 경로)

- Vacuum utilityGroup PoC에서 Pump 장비까지 BFS 경로 추출
- 경로별 변위 벡터(dx, dy, dz), 방향 패턴(R/H/D) 분류
- direction_raw(모든 세그먼트) vs direction_pattern(연속 동일방향 압축)

---

## 10. 데이터베이스 적재 모듈

### 10.1 테이블 구성

| 모듈 | 테이블 | 주요 공간 컬럼 | 용도 |
|------|--------|:------------:|------|
| import_obstacles_json.py | TB_BIM_OBSTACLES | box3d | 장애물 BBox |
| import_obstacles_json.py | TB_BIM_SPACE_INFO | box3d | 공간 레벨 BBox |
| import_obstacles_json.py | TB_BIM_EQUIPMENT | box3d + MultiPointZ | 장비 BBox + POC 좌표 |
| import_obstacles_json.py | TB_DUCT_LATERAL | box3d | 덕트·라테랄 BBox |
| import_duct_poc_cluster.py | TB_DUCT_POC_CLUSTER | MultiPointZ | PoC 클러스터 좌표 |
| import_pipe_group_analyze.py | TB_PIPE_GROUP_ANALYZE | PolygonZ | 배관 그룹 BBox |
| import_objects.py | TB_OBJECTS | box3d | BIM 객체 BBox |

### 10.2 공통 적재 흐름

```
1. argparse: CSV 경로 + DB 접속 정보 파싱
2. PostgreSQL 연결 + PostGIS 확장 로드
3. 테이블 스키마 검증 (불일치 시 자동 재생성)
4. CSV 파싱 → WKT 형상(box3d / MULTIPOINT Z / POLYGON Z) 변환
5. 동일 파일명 기존 데이터 DELETE (중복 방지)
6. executemany() 일괄 INSERT
7. 통계 보고
```

---

## 11. 3D 시각화

### 11.1 Python Plotly (VisualizeRouting3D.py)

- Phase 1 경로 JSON → 인터랙티브 HTML
- 시작점(POC): 빨간 다이아몬드 마커
- 경로: 10색 팔레트로 구분된 3D 라인
- 종단점: 검정 사각형 마커
- 실제 비율(aspectmode='data') 유지

### 11.2 Three.js 웹 뷰어 (visualization/)

| 파일 | 용도 |
|------|------|
| VisualizeGroupPipe3D.html | 그룹 배관 + 자동 경로 결합 뷰어 |
| VisualizeAutoRouting3D.html | V2 자동 경로 + 장비BBox + 장애물(기둥) 뷰어 |

---

## 12. 실행 명령어 모음

### Phase 1+2: 경로 추출 및 그룹 분석

```bash
# 전체 실행 (Phase 1 → Phase 2)
python src/AnalyzeRoutingAi_V2.py --phase all --input ./data/input

# Phase 1만 (경로 추출)
python src/AnalyzeRoutingAi_V2.py --phase routing --input ./data/input

# Phase 2만 (그룹 클러스터링)
python src/AnalyzeRoutingAi_V2.py --phase grouping --routing_out ./data/RoutingResults

# 옵션: 유사도 임계값 변경, 최소 그룹 크기 등
python src/AnalyzeRoutingAi_V2.py --phase all --input ./data/input \
    --pattern_similarity_min 0.75 --min_group_size 3
```

### Phase 3: 자동 경로 설계

```bash
# 단일 경로 자동설계
python src/AutoRoutingDesigner_V2.py \
    --json_input ./data/input/CMP_KSCTA01_*.json \
    --group_results ./data/GroupPipeResults/group_pipe_results_20260405194814.json \
    --equipment kscta01 --utility AKWW --size 20A \
    --start 5500,28200,15495 --dest 5300,27000,15495

# 배치 자동설계
python src/AutoRoutingDesigner_V2.py \
    --json_input ./data/input/CMP_KSCTA01_*.json \
    --group_results ./data/GroupPipeResults/group_pipe_results_20260405194814.json \
    --batch_input ./batch_poc_list.json

# 그룹 목록 조회
python src/AutoRoutingDesigner_V2.py \
    --json_input ./data/input/CMP_KSCTA01_*.json \
    --group_results ./data/GroupPipeResults/group_pipe_results_20260405194814.json \
    --list_groups

# 공간 컨텍스트 요약
python src/AutoRoutingDesigner_V2.py \
    --json_input ./data/input/CMP_KSCTA01_*.json \
    --group_results ./data/GroupPipeResults/group_pipe_results_20260405194814.json \
    --spatial_summary
```

### 보조 분석

```bash
# 분기점 분석
python src/AnalyzeBranching.py --input ./data/input --output ./BranchAnalysisResults

# 배관 그룹화 (공차 기반)
python src/analyze_group_pipes.py ./data/input --mode equipment --tol_z 150 --max_spacing 500

# 덕트 PoC 클러스터링
python src/duct_poc_clustering.py ./data/input

# Vacuum→Pump 경로 추출
python src/ExtractVacuumToPumpPaths.py -i ./data/input -o ./data/VacuumPumpPaths
```

### 시각화

```bash
# Python Plotly 3D
python src/VisualizeRouting3D.py ./data/RoutingResults

# Three.js 웹 뷰어 (로컬 서버)
python -m http.server 8080
# → http://localhost:8080/visualization/VisualizeAutoRouting3D.html
```

### DB 적재

```bash
# BIM 데이터 통합 임포트
python src/import_obstacles_json.py ./data/input \
    --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432 --clean

# 덕트 PoC 클러스터 적재
python src/import_duct_poc_cluster.py ./duct_poc_cluster_*.csv \
    --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432 --clean

# 배관 그룹 분석 결과 적재
python src/import_pipe_group_analyze.py ./group_rule_data-*.csv \
    --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432 --clean

# BIM 객체 요약 적재
python src/import_objects.py ./all_objects_summary_en_*.csv \
    --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432
```

---

## 13. 데이터 디렉토리 구조

### 13.1 input/ (원본 설계 JSON — 8개 장비)

| 파일 | 크기 | 장비 | Nodes | Edges | Obstacles |
|------|:----:|------|:-----:|:-----:|:---------:|
| CLEAN_WTNHJ02_*.json | 4.0MB | 세정기 | ~2,500 | ~1,800 | ~1,200 |
| CMP_KSCTA01_*.json | 2.3MB | CMP | ~1,500 | ~1,100 | ~456 |
| CVD_TNMHJ02_*.json | 7.1MB | CVD | ~3,472 | ~2,588 | ~3,001 |
| DIFF_DANHJ14_*.json | 2.2MB | 확산로 | ~1,400 | ~1,000 | ~800 |
| ETCH_ELOHJ07_*.json | 2.5MB | 에칭기 | ~1,600 | ~1,200 | ~900 |
| IMP_IVHHJ01_*.json | 1.6MB | 이온주입 | ~1,183 | ~879 | ~600 |
| METAL_SLWHJ02_*.json | 3.6MB | 금속증착 | ~2,200 | ~1,600 | ~1,500 |
| PHOTO_PSTWA03_*.json | 2.3MB | 포토 | ~1,500 | ~1,100 | ~700 |

### 13.2 RoutingResults/ (Phase 1 결과 — 30개 JSON)

장비별 경로 추출 결과. 경로마다 `path_arrow`, `path_step_vectors`, `path_step_lengths`, `path_range`, `h_segments`, `obstacle_relations` 포함.

### 13.3 GroupPipeResults/ (Phase 2 결과 — 6쌍 JSON+CSV)

그룹 클러스터링 결과. 그룹마다 `group_id`, `equipment_name`, `utility`, `size`, `path_count`, `avg_similarity`, `zones`, `paths[]` 포함.

### 13.4 AutoRoutingResults/ (Phase 3 결과 — 5개 JSON)

자동 설계 결과. 각 결과에 `quality_score`, `match_score`, `validation`, `similarity_detail`, `spatial_context` 포함.

---

## 14. 버전 이력

| 날짜 | 버전 | 주요 변경 사항 |
|------|:----:|--------------|
| 2026-03-23 | V1.0 | BFS 경로 추출 (AnalyzeRoutingPath.py) |
| 2026-03-30 | V1.1 | 분기점 분석 + 배관 그룹화 추가 |
| 2026-04-01 | V1.2 | DB 적재 모듈 4종 + 3D 시각화 |
| 2026-04-02 | V1.5 | AnalyzeRoutingAi_V2.py 개선 (5차원 유사도, Union-Find) |
| 2026-04-05 | V2.0 | AutoRoutingDesigner_V2.py (장비/장애물/종단점 8차원 유사도) |
| 2026-04-10 | V2.1 | 장애물 유형별 공간관계 18속성, 유사도 세분화, Slab Method 충돌감지 |
| 2026-04-13 | V2.1.1 | 전체 소스 파일 한글 주석 정비 (실행명령어, 흐름도, 함수/변수 설명) |

---

> **문서 끝**  
> 본 문서는 RoutingAI 시스템의 전체 아키텍처, 알고리즘, 데이터 흐름을 종합적으로 기술합니다.
