# RoutingAI 개발 문서

**프로젝트**: KGraphGen03 - 3D 배관 자동 라우팅 시스템  
**버전**: V2 (장비형상/장애물/종단점 고려)  
**최종 업데이트**: 2026-04-05  
**총 소스 규모**: Python 6,246줄 + HTML/JS 시각화

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [전체 흐름도](#2-전체-흐름도)
3. [Phase 1: 경로 추출 (AnalyzeRoutingAi_V2.py)](#3-phase-1-경로-추출)
4. [Phase 2: 그룹 클러스터링 (AnalyzeRoutingAi_V2.py)](#4-phase-2-그룹-클러스터링)
5. [Phase 3-V1: 자동 경로 설계 (AutoRoutingDesigner.py)](#5-phase-3-v1-자동-경로-설계)
6. [Phase 3-V2: 확장 자동 설계 (AutoRoutingDesigner_V2.py)](#6-phase-3-v2-확장-자동-설계)
7. [보조 분석 모듈](#7-보조-분석-모듈)
8. [3D 시각화](#8-3d-시각화)
9. [데이터 형식 명세](#9-데이터-형식-명세)
10. [설정 파라미터 레퍼런스](#10-설정-파라미터-레퍼런스)

---

## 1. 시스템 개요

### 1.1 목적

반도체/디스플레이 FAB 내 3D 배관 설계 데이터(JSON)를 분석하여:
1. 기존 배관 경로를 자동으로 추출하고 (Phase 1)
2. 장비·유틸리티별 유사 경로를 그룹화한 뒤 (Phase 2)
3. 새로운 Start PoC → 목적지가 주어지면 자동으로 최적 경로를 생성합니다 (Phase 3)

### 1.2 소스 파일 구성

| 파일 | 줄 수 | 역할 |
|------|------|------|
| `AnalyzeRoutingAi_V2.py` | 1,400 | Phase 1+2: BFS 경로 추출 + 유사도 기반 그룹 클러스터링 |
| `AutoRoutingDesigner.py` | 1,203 | Phase 3 V1: Zone 기반 템플릿 변형 자동 설계 |
| `AutoRoutingDesigner_V2.py` | 1,613 | Phase 3 V2: 장비형상/장애물/종단점 고려 확장 자동 설계 |
| `AnalyzeRoutingPath.py` | 1,153 | 경로 추출 V0 (레거시, V2에 통합) |
| `analyze_group_pipes.py` | 743 | 공차 기반 배관 그룹화 (보조) |
| `AnalyzeBranching.py` | 336 | 분기점(TEE/CROSS) 분석 |
| `VisualizeRouting3D.py` | 134 | Python/Plotly 3D 시각화 |
| `VisualizeGroupPipe3D.html` | - | Three.js 그룹 배관 뷰어 |
| `VisualizeAutoRouting3D.html` | - | Three.js V2 자동 경로 뷰어 |

---

## 2. 전체 흐름도

### 2.1 3-Phase 파이프라인

```
┌─────────────────────────────────────────────────────────────────────┐
│                    원본 설계 JSON (data/input/)                       │
│  Equipment(장비BBox, POC목록, 부대장비)                               │
│  Nodes(1,183~3,472개) + Edges(879~2,588개)                          │
│  Obstacles(456~3,001개: 기둥/빔/바닥)                                │
│  SpaceInfo(CSF/A/F/CR 3개 레벨)                                     │
└────────┬──────────────────────────────┬─────────────────────────────┘
         │                              │
         ▼                              │
┌─────────────────────────┐             │
│  Phase 1: 경로 추출      │             │
│  AnalyzeRoutingAi_V2.py │             │
│  --phase routing         │             │
│                          │             │
│  BFS 탐색 → 종단 판정    │             │
│  → 특징량 계산            │             │
│  → 장비별 JSON 저장       │             │
└────────┬────────────────┘             │
         │ RoutingResults/              │
         │ (30개 장비별 경로 JSON)        │
         ▼                              │
┌─────────────────────────┐             │
│  Phase 2: 그룹 분석      │             │
│  AnalyzeRoutingAi_V2.py │             │
│  --phase grouping        │             │
│                          │             │
│  버킷 분류 → 유사도 행렬  │             │
│  → Union-Find 클러스터링  │             │
│  → Zone 추정 → JSON/CSV  │             │
└────────┬────────────────┘             │
         │ GroupPipeResults/            │
         │ (58개 그룹, JSON+CSV)         │
         ▼                              ▼
┌──────────────────────────────────────────────┐
│  Phase 3: 자동 경로 설계                       │
│  AutoRoutingDesigner_V2.py                    │
│                                               │
│  ┌─────────────┐  ┌──────────────────────┐   │
│  │ 원본 JSON    │  │ GroupPipeResults     │   │
│  │ 장비BBox     │  │ 58개 그룹 템플릿      │   │
│  │ 장애물 456개  │  │ 유사도/Zone 정보      │   │
│  │ SpaceInfo   │  │                      │   │
│  └──────┬──────┘  └──────────┬───────────┘   │
│         │                    │               │
│         ▼                    ▼               │
│  SpatialContext        TemplateSelector      │
│  (공간 컨텍스트)        (그룹+대표경로 매칭)    │
│         │                    │               │
│         ▼                    ▼               │
│  EnhancedFeatureExtractor ──→ SpatialSimilarity │
│  (8차원 확장 특징량)         (8항목 가중 유사도)   │
│         │                    │               │
│         ▼                    ▼               │
│  ObstacleAwarePathBuilder                    │
│  (기둥 회피 + Zone 기반 3단계 경로 구성)        │
│         │                                    │
│         ▼                                    │
│  PathValidatorV2 (5항목 검증)                 │
│         │                                    │
│         ▼                                    │
│  AutoRoutingResults/ (자동 생성 경로 JSON)     │
└──────────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────────┐
│  3D 시각화 (Three.js)                         │
│  VisualizeAutoRouting3D.html                  │
│  장비BBox + 기둥 + 기존그룹 + 자동경로 통합 뷰   │
└──────────────────────────────────────────────┘
```

### 2.2 Phase 1 상세 흐름

```
data/input/*.json
    │
    ▼
RoutingGraph.load_from_json()
    ├─ Nodes → node_by_guid 딕셔너리 (GUID 인덱싱)
    ├─ Edges → edge_by_guid 딕셔너리
    ├─ Equipment → equipment_list
    └─ POC → poc_owner_map (POC GUID → 소유 장비 ID)
    │
    ▼
find_routing_paths()  ── 장비별 순회
    │
    ▼  각 POC에 대해
_trace_paths_from_poc()  ── BFS 탐색
    │
    ├─ 큐: (node_guid, record_idx, visited_edges, depth)
    │
    ├─ 반복:
    │   ├─ 안전검사: 큐 크기 > 100,000 → 중단
    │   ├─ 안전검사: depth > 512 → 건너뜀
    │   ├─ _classify_terminal() → 종단이면 경로 복원 후 저장
    │   ├─ get_neighbors() → 이웃 탐색 (VIRTUAL 엣지 제외)
    │   ├─ 사이클 방지 1: visited_edges 체크
    │   ├─ 사이클 방지 2: _is_in_path() 부모 체인 역추적
    │   └─ is_branch_node() → 분기면 branch_info 갱신
    │
    ▼
_compute_path_features()  ── 경로별 특징량 추출
    ├─ path_arrow: "R-H-H-R-R-H-R" (방향 코드열)
    ├─ path_bbox: BoundingBox (min/max/range/spread)
    ├─ path_step_vectors: [{x,y,z}, ...] (구간 벡터)
    ├─ path_step_lengths: [588.0, 100.0, ...] (구간 길이)
    ├─ path_total_length: 4896.0mm
    └─ h_segments: [{mean_z, mid_xy, node_count}] (수평 구간)
    │
    ▼
JSON 저장 → RoutingResults/{파일}_{장비명}_Path.json
```

### 2.3 Phase 2 상세 흐름

```
RoutingResults/*.json (30개)
    │
    ▼
_load_routing_records()  ── 경로 레코드 생성
    │  (장비/POC/유틸리티/경로특징량 포함)
    ▼
GroupAnalyzer.find_groups()
    │
    ├─ 1단계: 버킷 분류
    │    Key = (equipment_name, equipment_id, process, maker, utility, size)
    │
    ├─ 2단계: 버킷 내 유사도 행렬 계산 (n×n)
    │    compute_composite_similarity()
    │    ├─ Arrow Similarity  ×0.30  ← Levenshtein Distance
    │    ├─ Vector Similarity ×0.30  ← 코사인 유사도 시퀀스
    │    ├─ Range Similarity  ×0.20  ← BBox 범위 비교
    │    └─ Length Similarity ×0.20  ← 총 길이 비교
    │
    ├─ 3단계: Union-Find 클러스터링
    │    유사도 >= 0.70 → union (경로 압축 적용)
    │
    ├─ 4단계: 필터링
    │    ├─ 경로 수 >= 2 (min_group_size)
    │    ├─ 시작 POC XY 거리 <= 5,000mm
    │    └─ (선택) 공통 Z 레벨 존재 여부
    │
    └─ 5단계: 정렬 (-path_count, -avg_similarity)
    │
    ▼
detect_zones()  ── 그룹별 영역 추정
    ├─ trunk: 공통 수평 레벨 중 가장 밀집 영역 (path_count 최대)
    ├─ fan_in: trunk 앞쪽(아래쪽) 구간
    └─ fan_out: trunk 뒤쪽(위쪽) 구간
    │
    ▼
JSON + CSV 저장 → GroupPipeResults/group_pipe_results_*.json/.csv
```

### 2.4 Phase 3-V2 상세 흐름

```
입력: 장비명, 유틸리티, 사이즈, 시작좌표(x,y,z), 목적좌표(x,y,z)
    │
    ▼
SpatialContext.load_from_json()  ── 공간 컨텍스트 로딩
    ├─ Equipment: 장비 BBox, 67개 POC, 14개 부대장비(ends)
    ├─ Obstacles: 456개 (기둥 210, 빔 42, 바닥 203, 천장 1)
    └─ SpaceInfo: CSF(z8000~13700), A/F(z13700~15500), CR(z15500~25000)
    │
    ▼
find_matching_group()  ── 그룹 + 대표 경로 매칭
    │
    ├─ 1차: equipment + utility + size 완전 일치 필터
    │
    ├─ 2차: 각 후보 그룹의 모든 경로와 확장 유사도 계산
    │    EnhancedFeatureExtractor.extract() → 8차원 특징량
    │    SpatialSimilarity.compute() → 8항목 가중 유사도
    │    ├─ arrow     ×0.15 : 방향 패턴
    │    ├─ vector    ×0.15 : 벡터 시퀀스
    │    ├─ range     ×0.10 : BBox 범위
    │    ├─ length    ×0.10 : 총 길이
    │    ├─ equip_rel ×0.15 : 장비 상대 좌표
    │    ├─ terminal  ×0.15 : 종단점 유형/거리
    │    ├─ obstacle  ×0.10 : 장애물 밀도/교차
    │    └─ level     ×0.10 : 레벨 경유
    │
    ├─ 3차: XY 근접성 보너스 적용
    └─ 최고 점수 경로 선정
    │
    ▼
ObstacleAwarePathBuilder.build_path()  ── 3단계 경로 구성
    │
    ├─ zone 분류: 템플릿의 step_vectors → fan_in / trunk / fan_out
    │
    ├─ fan_in 구성: start → trunk 진입점
    │    ├─ R(수직) 세그먼트: Z 높이차에 맞게 스케일링
    │    ├─ H(수평) 세그먼트: XY 스케일링 + 방향 회전 + 직교 스냅
    │    └─ 피팅(≤150mm R): 원본 유지 (물리적 크기 불변)
    │
    ├─ trunk 구성: trunk 레벨 고정, H 세그먼트만 재분배
    │
    ├─ fan_out 구성: trunk 이탈점 → destination (fan_in 역순)
    │
    ├─ 장애물 회피: _apply_obstacle_avoidance()
    │    ├─ 각 H 세그먼트에서 COLUMN_STRUCTURE 충돌 감지 (Slab method)
    │    ├─ 충돌 시 우회 세그먼트 삽입 (최대 5회)
    │    └─ 이동 방향 직교 방향으로 기둥 크기 + margin만큼 우회
    │
    ├─ 끝점 보정: 실제 끝점과 destination 간 갭 보정
    │
    └─ 정리: 영벡터 제거, 짧은 세그먼트 병합
    │
    ▼
PathValidatorV2.validate()  ── 5항목 검증
    ├─ continuity   ×0.20 : 좌표 연속성
    ├─ length_ratio ×0.20 : 길이 합리성 (0.2~2.5배)
    ├─ pattern      ×0.15 : 방향 패턴 유사도
    ├─ obstacle_free×0.25 : 기둥 충돌 0 검증
    └─ zone_comply  ×0.20 : Zone 범위 준수
    │
    ▼
결과 저장 → AutoRoutingResults/auto_routing_v2_*.json
```

---

## 3. Phase 1: 경로 추출

### 3.1 RoutingGraph 클래스 (줄 385~727)

#### 인스턴스 변수

| 변수 | 타입 | 설명 |
|------|------|------|
| `config` | AnalysisConfig | 분석 설정 |
| `node_by_guid` | Dict[str, Dict] | GUID → 노드 데이터 매핑 |
| `edge_by_guid` | Dict[str, Dict] | GUID → 엣지 데이터 매핑 |
| `equipment_list` | List[Dict] | 장비 목록 |
| `poc_owner_map` | Dict[str, List[str]] | POC GUID → 소유 장비 ID 목록 |

#### 메서드 상세

| 메서드 | 줄 | 시그니처 | 반환 | 핵심 로직 |
|--------|-----|----------|------|-----------|
| `load_from_json` | 395 | `(file_path: str) → bool` | 성공/실패 | Nodes/Edges/Equipment 파싱 → GUID 인덱싱, poc_owner_map 구축 |
| `get_neighbors` | 442 | `(node_guid: str) → List[Tuple[Dict, str]]` | (엣지, 다음GUID) | connectionGuidList 순회, VIRTUAL 제외, 중복 제거 |
| `is_branch_node` | 495 | `(node_guid: str) → bool` | 분기 여부 | TEE/BRANCH/JUNCTION/CROSS/WYE 또는 이웃≥3 |
| `find_routing_paths` | 518 | `() → Dict[str, Dict]` | 장비별 경로 | 장비 순회 → POC별 _trace_paths_from_poc 호출 |
| `_classify_terminal` | 557 | `(curr, node, start, eq_id) → Tuple[bool, str]` | (종단여부, 사유) | 8단계 종단 판정 |
| `_trace_paths_from_poc` | 603 | `(start_node, start_guid, eq_id) → List[Dict]` | 경로 목록 | BFS + 이중 사이클 방지 + 분기 추적 |
| `_is_in_path` | 707 | `(records, idx, target) → bool` | 중복 여부 | 부모 포인터 역추적 |
| `_reconstruct_path` | 716 | `(records, end_idx) → List[Dict]` | NODE/EDGE 목록 | 역방향 복원 후 reverse |

#### 종단 판정 우선순위 (_classify_terminal)

| 순서 | 조건 | 사유 라벨 |
|------|------|-----------|
| 1 | curr == start | 종단 아님 (시작점) |
| 2 | POC 소유 장비 ≠ 현재 장비 | "다른 장비 PoC 도달" |
| 3 | 타입 DUCT/TAKEOFF | "Duct / TakeOff 도달" |
| 4 | 타입 LATERAL | "Lateral Pipe 도달" |
| 5 | 이름에 NOZZLE 포함 | "Nozzle PoC 도달" |
| 6 | 타입 POC이고 이름 END | "종단 PoC 도달" |
| 7 | 타입 EQUIPMENT | "장비 노드 도달" |
| 8 | 이웃 없음 | "배관 끝단(막힘)" |

### 3.2 BFS 알고리즘 상세

```
입력: start_guid (POC 노드), current_equipment_id
자료구조:
  node_records[] = [{guid, node, parent_idx, edge, branch_info, depth}]
  queue = deque([(guid, record_idx, visited_edges(frozenset), depth)])

알고리즘:
  1. 초기화: node_records[0] = start, queue = [(start, 0, ∅, 0)]
  2. while queue:
     a. (curr_guid, idx, visited, depth) = queue.popleft()
     b. 안전검사: |queue| > max_queue_size → break
     c. 안전검사: depth > max_depth → continue
     d. 종단검사: _classify_terminal() → 종단이면
        - _reconstruct_path()로 경로 복원
        - paths_found에 추가
        - |paths_found| >= max_paths_per_poc → break
     e. neighbors = get_neighbors(curr_guid)
     f. for (edge, next_guid) in neighbors:
        - edge_sig = edge GUID (사이클 방지 1)
        - if edge_sig ∈ visited → skip
        - if _is_in_path(records, idx, next_guid) → skip (사이클 방지 2)
        - 분기 노드면 branch_info 갱신
        - node_records에 추가
        - queue.append((next_guid, new_idx, visited | {edge_sig}, depth+1))

복잡도: O(V + E) per POC (V=노드수, E=엣지수)
사이클 방지: 엣지 기반(frozenset) + 노드 기반(부모 체인) 이중 체크
```

### 3.3 특징량 계산 함수

| 함수 | 줄 | 시그니처 | 설명 |
|------|-----|----------|------|
| `_compute_segment_code` | 184 | `(p1, p2, tol_deg) → str?` | 두 점 사이 이동을 R/H/D로 분류. θ = arcsin(\|dz\|/dist). θ>85°→R, θ<5°→H, else→D |
| `_compute_path_arrow` | 202 | `(positions, tol_deg) → str` | 전체 경로의 segment 코드열 (예: "R-H-H-R-R-H-R") |
| `_extract_h_segments` | 216 | `(positions, arrow) → List[Dict]` | 연속 H 구간의 mean_z, mid_xy, node_count |
| `_bbox` | 143 | `(positions) → Dict?` | x/y/z min/max/range + xy_spread + node_count |
| `_compute_path_features` | 327 | `(steps, config) → Dict` | 위 함수들을 통합하여 전체 특징량 딕셔너리 반환 |

---

## 4. Phase 2: 그룹 클러스터링

### 4.1 유사도 함수

| 함수 | 줄 | 가중치 | 알고리즘 |
|------|-----|--------|----------|
| `pattern_similarity` | 735 | 0.30 | Levenshtein Distance 기반. 코드열을 토큰 단위로 편집 거리 계산 → 1 - dist/max_len |
| `_cosine_similarity_0_1` | 760 | - | 코사인 유사도 0~1 정규화. dot/(mag_a×mag_b), 반대 방향=0 |
| `_vector_sequence_similarity` | 783 | 0.30 | 코사인 유사도 평균 × 길이 커버리지(min_len/max_len) |
| `_range_similarity` | 805 | 0.20 | x,y,z 각 축의 1 - \|v1-v2\|/max(v1,v2,1) 평균 |
| `_length_similarity` | 816 | 0.20 | 1 - \|len1-len2\|/max(len1,len2,1) |
| `compute_composite_similarity` | 827 | 1.00 | 위 4개의 가중합 |

### 4.2 GroupAnalyzer 클래스 (줄 846~993)

#### Union-Find 클러스터링 알고리즘

```
입력: 동일 버킷(장비+유틸리티+사이즈) 내 경로 레코드 n개

1. 유사도 행렬 계산: sim[i][j] = compute_composite_similarity(records[i], records[j])
   복잡도: O(n²)

2. Union-Find 초기화: parent[i] = i

3. for i in range(n):
     for j in range(i+1, n):
       if sim[i][j] >= pattern_similarity_min (0.70):
         union(i, j)  # 경로 압축 적용

4. 클러스터 추출: {find(i): [records...]}

5. 필터링:
   - |cluster| >= min_group_size (2)
   - max_xy_dist(start_positions) <= start_poc_xy_max (5000mm)
   - (선택) 공통 Z 레벨 존재

6. 정렬: -path_count, -avg_similarity
```

### 4.3 Zone 추정 알고리즘 (detect_zones, 줄 997~1062)

```
입력: 그룹 후보 (common_z_levels, paths)

1. trunk 후보 선택:
   - xy_spread <= trunk_max_xy_spread(1500mm)인 것 중 path_count 최대
   - 없으면 xy_spread 최소 선택 (is_estimated=True)

2. Z 대역 정의: trunk_z ± (tol_z_level × trunk_z_band_factor) = ±400mm

3. 각 경로의 노드를 순서대로 분류:
   - trunk 범위 첫 진입 전 → fan_in
   - trunk 범위 내 → trunk
   - trunk 범위 마지막 이후 → fan_out

4. 각 영역의 BoundingBox 계산
```

---

## 5. Phase 3-V1: 자동 경로 설계

### 5.1 클래스 구조

```
AutoRoutingDesigner (오케스트레이터)
  ├─ TemplateSelector       ← 그룹 매칭 + 대표 경로 선정
  ├─ ZoneAwarePathBuilder   ← Zone 기반 3단계 경로 구성
  ├─ PathAdapter            ← 좌표 보정
  └─ PathValidator          ← 품질 검증
```

### 5.2 DesignConfig 설정값

| 필드 | 기본값 | 설명 |
|------|--------|------|
| `group_results_path` | GroupPipeResults JSON | 입력 |
| `output_dir` | ./AutoRoutingResults | 출력 |
| `max_start_xy_distance` | 8,000mm | 후보 그룹 필터 반경 |
| `min_direction_similarity` | 0.3 | displacement 최소 유사도 |
| `elbow_clearance` | 100mm | 엘보우 최소 직관 길이 |
| `trunk_approach_tolerance` | 300mm | trunk Z 접근 허용 오차 |
| `min_segment_length` | 50mm | 최소 세그먼트 길이 |
| `fitting_length_threshold` | 150mm | 피팅 판별 기준 |
| `max_length_ratio` | 2.5 | 길이 검증 상한 |
| `min_length_ratio` | 0.2 | 길이 검증 하한 |

### 5.3 TemplateSelector (줄 86~229)

**그룹 매칭 알고리즘**:
```
1. equipment_name + utility + size 완전 일치 필터
2. 그룹 내 시작점들과의 최소 XY 거리 계산
3. XY 거리 > max_start_xy_distance → 제외
4. 점수 = avg_similarity × log₂(path_count + 1) × (1 - xy_dist/max_dist)
5. Fallback: equipment_name만 일치 (점수 × 0.5)
```

**대표 경로 선정 알고리즘**:
```
각 경로에 대해:
  cos_sim = cosine_similarity(new_displacement, path_displacement)  ×0.4
  xy_score = 1 - xy_dist / max_xy_in_group                        ×0.3
  len_score = 1 - |len_diff| / max(len1, len2)                    ×0.3
→ 최고 점수 경로 선정
```

### 5.4 ZoneAwarePathBuilder (줄 292~700)

**3단계 경로 구성**:

```
new_start(z=15495)
    │
    │  [fan_in] R하강 + H수평 → trunk 진입점까지
    │  - R 세그먼트: Z 높이차에 맞게 스케일링 (scale_z = dz_needed / tmpl_r_dz)
    │  - H 세그먼트: XY 스케일링 + 방향 회전 + 직교 스냅
    │  - 피팅(≤150mm R): 원본 값 그대로 유지 (물리적 크기 불변)
    ▼
trunk(z=14699)
    │  [trunk] 수평 레벨 고정
    │  - 피팅 XY 소모량 계산 → 잔여량을 H 세그먼트에 비례 배분
    │  - 템플릿의 H 길이 비율로 재분배
    ▼
destination(z=15495)
    ▲  [fan_out] trunk 이탈점에서 → H수평 + R상승
    │  - fan_in과 동일 로직 (역방향)
```

**XY 방향 회전**:
```python
rotation = atan2(new_disp_y, new_disp_x) - atan2(tmpl_disp_y, tmpl_disp_x)
rotated_x = x * cos(rotation) - y * sin(rotation)
rotated_y = x * sin(rotation) + y * cos(rotation)
snapped_x, snapped_y = snap_to_orthogonal(rotated_x, rotated_y)
```

---

## 6. Phase 3-V2: 확장 자동 설계

### 6.1 V1 대비 핵심 변경사항

| 항목 | V1 | V2 |
|------|----|----|
| **입력** | GroupPipeResults만 | + 원본 JSON (장비/장애물/SpaceInfo) |
| **특징량** | 4차원 (arrow, vector, range, length) | **8차원** (+장비상대, 종단점, 장애물, 레벨) |
| **유사도** | 4항목 균등 가중 | **8항목 세분화 가중치** |
| **경로 생성** | Zone 기반 템플릿 변형만 | + **기둥 충돌 감지 + 우회** |
| **검증** | 4항목 | 5항목 (**+기둥 충돌 검사**) |

### 6.2 SpatialContext 클래스 (줄 202~387)

원본 JSON에서 공간 정보를 로딩하고 쿼리를 제공합니다.

#### 인스턴스 변수

| 변수 | 타입 | 설명 |
|------|------|------|
| `equipment` | EquipmentInfo | 장비 정보 (BBox, POC 67개, ends 14개) |
| `obstacles` | List[ObstacleInfo] | 전체 장애물 456개 |
| `columns` | List[ObstacleInfo] | 기둥만 210개 (COLUMN_STRUCTURE + COLUMN_ARCHITECTURE) |
| `beams` | List[ObstacleInfo] | 빔만 42개 |
| `levels` | List[SpaceLevelInfo] | CSF/A/F/CR 3개 레벨 |
| `ends_map` | Dict[str, Dict] | 부대장비 GUID → end 정보 |

#### 공간 쿼리 메서드

| 메서드 | 시그니처 | 설명 |
|--------|----------|------|
| `find_obstacles_in_path` | `(p1, p2, clearance) → List[ObstacleInfo]` | 선분-BBox 교차 판정 (Slab method) |
| `find_nearby_obstacles` | `(point, radius) → List[(float, ObstacleInfo)]` | 반경 내 장애물 (거리순 정렬) |
| `compute_obstacle_density` | `(center, radius) → float` | 장애물 밀도 (기둥 가중치 3, 빔 가중치 1) |
| `get_level_at_z` | `(z) → SpaceLevelInfo?` | Z 좌표의 공간 레벨 |

#### Slab Method 충돌 판정 알고리즘

```
입력: 선분(p1→p2), 장애물 BBox(min,max), margin

각 축(x,y,z)에 대해:
  if 방향 성분 ≈ 0:
    p1이 BBox 범위 밖이면 → 교차 없음
  else:
    t1 = (bmin[i] - p1[i]) / d[i]
    t2 = (bmax[i] - p1[i]) / d[i]
    t_min = max(t_min, min(t1,t2))
    t_max = min(t_max, max(t1,t2))
    if t_min > t_max → 교차 없음

t_min <= t_max → 교차 있음
```

### 6.3 EnhancedPathFeatures (줄 390~421)

| 카테고리 | 필드 | 타입 | 설명 |
|----------|------|------|------|
| 기존 | `path_arrow` | str | "R-H-H-R-R-H-R" |
| 기존 | `path_step_vectors` | List[Dict] | [{x,y,z}, ...] |
| 기존 | `path_step_lengths` | List[float] | [588.0, 100.0, ...] |
| 기존 | `path_range` | Dict | {x: 1203, y: 901, z: 796} |
| 기존 | `path_total_length` | float | 4896.0mm |
| **장비** | `start_relative_pos` | Tuple(3) | 장비 BBox 내 0~1 정규화 좌표 |
| **장비** | `end_relative_pos` | Tuple(3) | 종점의 정규화 좌표 |
| **장비** | `start_face` | str | N/S/E/W/T/B (장비의 어느 면에서 출발) |
| **종단점** | `terminal_type` | str | BRANCH/DUCT/LATERAL/EQUIPMENT/DISCONNECTED |
| **종단점** | `terminal_distance` | float | 시작점~종단점 직선 거리 |
| **종단점** | `terminal_level` | str | 종단점이 속한 레벨 (CSF/A/F/CR) |
| **장애물** | `obstacle_count_nearby` | int | 반경 내 장애물 수 |
| **장애물** | `obstacle_density` | float | 장애물 밀도 (0~1) |
| **장애물** | `obstacle_min_distance` | float | 최근접 장애물 거리 |
| **장애물** | `column_crossings` | int | 기둥 교차 세그먼트 수 |
| **레벨** | `start_level` | str | 시작점 레벨 |
| **레벨** | `end_level` | str | 종점 레벨 |
| **레벨** | `levels_traversed` | List[str] | 경유 레벨 시퀀스 (예: [A/F, CSF]) |
| **레벨** | `level_change_count` | int | 레벨 변경 횟수 |

### 6.4 SpatialSimilarity 8항목 가중 유사도 (줄 535~705)

| 항목 | 가중치 | 계산 방법 |
|------|--------|-----------|
| `arrow` | 0.15 | Levenshtein Distance 기반 (V1과 동일) |
| `vector` | 0.15 | 코사인 유사도 시퀀스 × 길이 커버리지 |
| `range` | 0.10 | BBox x,y,z 범위 비교 평균 |
| `length` | 0.10 | 총 길이 비율 |
| `equip_relative` | 0.15 | 시작점 상대좌표 거리(0.4) + 종점 상대좌표(0.4) + 출발면 일치 보너스(0.2) |
| `terminal` | 0.15 | 종단 타입 일치(0.4) + 거리 유사도(0.3) + 레벨 일치(0.3) |
| `obstacle` | 0.10 | 밀도 차이(0.5) + 기둥 교차수 유사도(0.5) |
| `level` | 0.10 | 시작레벨(0.25) + 종점레벨(0.25) + 변경횟수(0.25) + 경유레벨 Jaccard(0.25) |

### 6.5 ObstacleAwarePathBuilder 장애물 회피 (줄 757~1126)

**우회 알고리즘**:
```
입력: 생성된 경로의 모든 세그먼트

각 H(수평) 세그먼트에 대해 (R/피팅은 제외):
  if COLUMN_STRUCTURE와 충돌 (Slab method, clearance=200mm):
    가장 가까운 기둥 선택
    
    주 이동 방향 판별:
      |dx| >= |dy| → Y방향으로 우회
      |dx| < |dy|  → X방향으로 우회
    
    우회 폭 = 기둥 크기/2 + detour_margin(300mm)
    
    3개 세그먼트로 분해:
      [1] 직교 방향 이동 (우회)
      [2] 원래 방향 이동 (기둥 통과)
      [3] 직교 역방향 이동 (복귀) + 잔여 보정
    
    최대 5회 우회 (경로 과도 확장 방지)
```

### 6.6 PathValidatorV2 5항목 검증 (줄 1127~1223)

| 검증 항목 | 가중치 | 기준 |
|-----------|--------|------|
| `continuity` | 0.20 | start_pos + Σ(vectors) == positions, 오차 < 1.5mm |
| `length_ratio` | 0.20 | 생성길이/템플릿길이 ∈ [0.2, 2.5] |
| `pattern` | 0.15 | pattern_similarity(gen_arrow, tmpl_arrow) |
| `obstacle_free` | 0.25 | H 세그먼트의 COLUMN_STRUCTURE 충돌 수 × 0.3 감점 |
| `zone_compliance` | 0.20 | 경로 BBox가 trunk의 1.5배 범위 이내 |

**합격 조건**: quality_score >= 0.4 AND continuity == 1.0

---

## 7. 보조 분석 모듈

### 7.1 analyze_group_pipes.py (743줄)

**목적**: 공차/방향/근접도 기반 배관 그룹화 (Phase 2의 보조)

**전역 상수**:
```python
TOL_Z_ELEVATION = 100.0mm    # 수평 배관 Z 공차
TOL_ANGLE_DEG = 3.0°         # 방향 평행성 공차
MAX_SPACING = 300.0mm        # 배관 간 최대 간격
MAX_LONGITUDINAL_GAP = 1000mm # 종방향 허용 갭
TOL_ALIGNMENT = 100.0mm      # 축 정렬 오차
```

**핵심 클래스**: `PipeData` - 배관 기하학 캡슐화 (양 끝점, 중점, 길이, 방향 벡터)

**알고리즘**: 수직/수평 분류 → 방향별 그룹화 → BFS 근접 클러스터링 → 통계

### 7.2 AnalyzeBranching.py (336줄)

**목적**: 분기점(TEE/CROSS) 분석 및 종단 추적

**핵심 클래스**: `BranchGraph`
- `analyze_branches()`: 모든 분기 노드 탐색 → 각 분기에서 종단까지 추적
- 출력: 분기별 연결 수, 종단 노드 ID, 공간 레벨

### 7.3 VisualizeRouting3D.py (134줄)

**목적**: Plotly 기반 Python 3D 시각화
- POC 마커 (빨간 다이아몬드)
- 유틸리티별 색상 코딩된 경로 라인
- 인터랙티브 3D 카메라

---

## 8. 3D 시각화

### 8.1 VisualizeAutoRouting3D.html

**기술 스택**: Three.js 0.163.0 + ES Module

**렌더링 레이어**:

| 레이어 | 색상 | 투명도 | 토글 |
|--------|------|--------|------|
| 장비 BBox | 파란색 (#6488ff) | 0.12 | btn-equip |
| 기둥 (COLUMN_STRUCTURE) | 빨간색 (#ff3c3c) | 0.15 | btn-columns |
| 기존 그룹 경로 | 보라색 (#b4b4ff) | 0.20 | btn-groups |
| 자동 생성 경로 | 청록 점선 (#00ffc8) | 0.95 | btn-auto |
| 시작점 구체 | 노란색 | 0.85 | - |
| 끝점 구체 | 빨간색 | 0.75 | - |

**UI 구성**:
- 좌측 패널: 경로 리스트 (PASS/WARN 뱃지, 품질/매칭 점수, 충돌 수)
- 하단 패널: 상세 정보 (8개 유사도 항목별 바 차트)
- 상단 툴바: 레이어 토글 + Fit

**좌표 변환**: 설계 좌표(X,Y,Z) → Three.js(X,Z,-Y) Y-up 변환

### 8.2 VisualizeGroupPipe3D.html

**렌더링 레이어**:
- 58개 그룹별 고유 HSL 색상 경로
- Zone 박스: Trunk(파랑), Fan-In(초록), Fan-Out(주황)
- 자동 생성 경로: 점선 + 밝은 녹색 + AUTO 태그

**UI 구성**:
- 검색/필터 (장비명, 유틸리티)
- 그룹 클릭 시 카메라 이동 + 나머지 반투명
- 눈 아이콘: 개별 그룹 표시/숨김

---

## 9. 데이터 형식 명세

### 9.1 입력: 원본 설계 JSON

```json
{
  "FileInfo": {
    "SpaceInfo": [
      {"levelName": "CSF", "boundary": {"min": {"x":..,"y":..,"z":8000}, "max": {"x":..,"y":..,"z":13700}}}
    ]
  },
  "Equipment": {
    "guid": "...", "name": "kscta01", "process": "CMP",
    "position": [5290.9, 28613.4, 15500.0],
    "boundaryBox": {"min": {"x":5290.9,"y":22801.8,"z":15495.0}, "max": {"x":7787.2,"y":28613.4,"z":17500.0}},
    "pocList": [
      {"id": "...", "utility": "AKWW", "pocPosition": [5459.2, 28503.2, 15495.0],
       "endPocs": [{"endName": "END_00002_BRANCH PIPE", "endType": "disconnected pipe", "endPocPosition": [4963.2, 22467.3, 13182.4]}]}
    ],
    "ends": [
      {"name": "BRANCH PIPE_...", "type": "BRANCH PIPE", "boundaryBox": {...}, "pocList": [...]}
    ]
  },
  "Obstacles": [
    {"obstacleId": "...", "ddworksType": "COLUMN_STRUCTURE", "boundary": {"min": {"x":0,"y":33800,"z":0}, "max": {"x":1300,"y":35200,"z":25000}}}
  ],
  "Nodes": [
    {"guid": "...", "type": "ELBOW", "position": [x,y,z], "connectionGuidList": ["edge_guid_1"]}
  ],
  "Edges": [
    {"guid": "...", "type": "PIPE", "utility": "AKWW", "size": "20A", "connectionGuidList": ["node1","node2"]}
  ]
}
```

### 9.2 Phase 1 출력: RoutingResults

```json
{
  "source_file": "...", "equipment_id": "...", "equipment_name": "kscta01",
  "poc_paths": [{
    "start_poc_id": "...",
    "paths": [{
      "terminal_label": "다른 장비 PoC 도달",
      "path_summary": "PO->EB->TE->FL->PO",
      "path_arrow": "R-H-H-R-R-H-R",
      "path_step_vectors": [{"x":0,"y":0,"z":-588}, {"x":0,"y":100,"z":0}, ...],
      "path_step_lengths": [588.0, 100.0, ...],
      "path_total_length": 4896.0,
      "steps": [{"kind":"NODE","data":{...}}, {"kind":"EDGE","data":{...}}, ...]
    }]
  }]
}
```

### 9.3 Phase 2 출력: GroupPipeResults

```json
[{
  "group_id": 1,
  "equipment_name": "kscta01", "utility": "AKWW", "size": "20A",
  "path_count": 36, "avg_similarity": 0.749,
  "common_z_levels": [{"mean_z": 14699.1, "path_count": 36, "xy_spread": 3098.0}],
  "zones": {
    "trunk": {"x_min":4256.2, "x_max":5459.2, "mean_z":14699.1, ...},
    "fan_in": {...}, "fan_out": {...}, "is_trunk_estimated": true
  },
  "paths": [{
    "poc_id": "...", "start_pos": [5459.2, 28503.2, 15495.0],
    "path_arrow": "R-H-H-R-R-R-R-H-H-R-R-R-R-H-R",
    "path_step_vectors": [...], "path_total_length": 4896.0
  }]
}]
```

### 9.4 Phase 3-V2 출력: AutoRoutingResults

```json
[{
  "group_id": "AUTO_001", "source_group_id": 1,
  "is_auto_generated": true, "version": "V2",
  "equipment_name": "kscta01", "utility": "AKWW", "size": "20A",
  "quality_score": 0.934, "match_score": 0.661,
  "validation": {
    "is_valid": true, "quality_score": 0.934,
    "checks": {"continuity":1.0, "length_ratio":0.95, "pattern":0.667, "obstacle_free":1.0, "zone_compliance":1.0},
    "collision_count": 0, "warnings": []
  },
  "similarity_detail": {
    "arrow":0.667, "vector":0.52, "range":0.85, "length":0.78,
    "equip_relative":0.92, "terminal":0.71, "obstacle":0.88, "level":1.0
  },
  "spatial_context": {
    "equipment_bbox": {"min":[5290.9,22801.8,15495.0], "max":[7787.2,28613.4,17500.0]},
    "obstacles_nearby": 45, "obstacle_density": 0.12,
    "start_level": "A/F", "end_level": "A/F",
    "levels_traversed": ["A/F"], "start_face": "B", "terminal_type": "DISCONNECTED"
  },
  "paths": [{
    "poc_id": "auto_v2_20260405...",
    "terminal_label": "자동 생성 경로 (V2)",
    "start_pos": [5500,28200,15495], "end_pos": [5300,27000,15495],
    "path_arrow": "R-H-R-R-H-H-R-R-H-R-H",
    "path_step_vectors": [...], "path_total_length": 3407.8
  }]
}]
```

---

## 10. 설정 파라미터 레퍼런스

### 10.1 AnalysisConfig (Phase 1+2)

| 파라미터 | 기본값 | CLI 옵션 | 영향 |
|----------|--------|----------|------|
| `input_dir` | ./data-v10 | --input | 원본 JSON 디렉토리 |
| `routing_out` | ./RoutingResults | --routing_out | Phase 1 출력 |
| `group_out` | ./GroupPipeResults | --group_out | Phase 2 출력 |
| `max_branch_count` | 8 | - | 분기 수 경고 임계값 |
| `direction_angle_tolerance` | 5.0° | - | R/H/D 분류 기준 |
| `max_paths_per_poc` | 5,000 | --max_paths_per_poc | POC별 최대 경로 수 |
| `max_queue_size` | 100,000 | --max_queue_size | BFS 큐 한도 |
| `max_depth` | 512 | --max_depth | BFS 최대 깊이 |
| `pattern_similarity_min` | 0.70 | --pattern_similarity_min | 그룹 인정 최소 유사도 (↑엄격, ↓느슨) |
| `start_poc_xy_max` | 5,000mm | --start_poc_xy_max | 시작 POC XY 거리 제한 |
| `tol_z_level` | 200mm | --tol_z_level | 공통 레벨 Z 허용 오차 |
| `trunk_max_xy_spread` | 1,500mm | --trunk_max_xy_spread | Trunk XY 최대 spread |
| `trunk_z_band_factor` | 2.0 | --trunk_z_band_factor | Trunk Z 대역 배수 |
| `min_group_size` | 2 | --min_group_size | 최소 그룹 경로 수 |

### 10.2 DesignConfigV2 (Phase 3-V2)

| 파라미터 | 기본값 | 영향 |
|----------|--------|------|
| `obstacle_clearance` | 200mm | 장애물 충돌 판정 마진 |
| `obstacle_detour_margin` | 300mm | 우회 시 추가 마진 |
| `max_start_xy_distance` | 8,000mm | 템플릿 매칭 반경 |
| `fitting_length_threshold` | 150mm | 피팅 판별 (이하의 R 세그먼트) |
| `w_arrow` | 0.15 | 방향 패턴 유사도 가중치 |
| `w_vector` | 0.15 | 벡터 시퀀스 유사도 가중치 |
| `w_range` | 0.10 | BBox 범위 유사도 가중치 |
| `w_length` | 0.10 | 총 길이 유사도 가중치 |
| `w_equip_relative` | 0.15 | 장비 상대 좌표 유사도 가중치 |
| `w_terminal` | 0.15 | 종단점 유사도 가중치 |
| `w_obstacle_proximity` | 0.10 | 장애물 근접도 유사도 가중치 |
| `w_level` | 0.10 | 레벨 유사도 가중치 |
| `max_length_ratio` | 2.5 | 검증: 길이 상한 비율 |
| `min_length_ratio` | 0.2 | 검증: 길이 하한 비율 |

### 10.3 전역 상수

| 상수 | 파일 | 값 | 설명 |
|------|------|-----|------|
| `TYPE_ABBREV` | V2.py:45 | 31개 매핑 | 노드 타입 → 2글자 약어 |
| `BRANCH_NODE_TYPES` | V2.py:54 | {TEE, BRANCH, JUNCTION, CROSS, WYE} | 분기 판정 타입 |
| `FITTING_LENGTH_THRESHOLD` | Designer:53 | 150mm | 피팅 세그먼트 판별 |
| `OBSTACLE_CLEARANCE` | V2_Designer:55 | 200mm | 배관~장애물 최소 거리 |
| `TERMINAL_TYPES` | V2_Designer:59 | 5개 매핑 | 종단 타입 분류 |

---

**문서 작성일**: 2026-04-05  
**프로젝트 상태**: Active Development  
**Python 의존성**: 표준 라이브러리만 (외부 패키지 불필요)
