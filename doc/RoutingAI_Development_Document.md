# RoutingAI 개발 문서

**프로젝트**: KGraphGen03 - 3D 배관 자동 라우팅 시스템  
**버전**: V2.1 (장애물 유형별 공간관계 특징량 확장)  
**최종 업데이트**: 2026-04-10  
**총 소스 규모**: Python ~7,000줄 + HTML/JS 시각화

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [전체 흐름도](#2-전체-흐름도)
3. [Phase 1: 경로 추출 (AnalyzeRoutingAi_V2.py)](#3-phase-1-경로-추출)
4. [Phase 2: 그룹 클러스터링 (AnalyzeRoutingAi_V2.py)](#4-phase-2-그룹-클러스터링)
5. [Phase 3-V1: 자동 경로 설계 (AutoRoutingDesigner.py)](#5-phase-3-v1-자동-경로-설계)
6. [Phase 3-V2: 확장 자동 설계 (AutoRoutingDesigner_V2.py)](#6-phase-3-v2-확장-자동-설계)
7. [장애물 유형별 공간관계 시스템](#7-장애물-유형별-공간관계-시스템)
8. [보조 분석 모듈](#8-보조-분석-모듈)
9. [3D 시각화](#9-3d-시각화)
10. [데이터 형식 명세](#10-데이터-형식-명세)
11. [설정 파라미터 레퍼런스](#11-설정-파라미터-레퍼런스)

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
| `AnalyzeRoutingAi_V2.py` | ~1,500 | Phase 1+2: BFS 경로 추출 + 5차원 유사도 기반 그룹 클러스터링 |
| `AutoRoutingDesigner.py` | 1,203 | Phase 3 V1: Zone 기반 템플릿 변형 자동 설계 |
| `AutoRoutingDesigner_V2.py` | ~1,800 | Phase 3 V2: 장비형상/장애물유형별공간관계/종단점 고려 확장 자동 설계 |
| `AnalyzeRoutingPath.py` | 1,153 | 경로 추출 V0 (레거시, V2에 통합) |
| `analyze_group_pipes.py` | 743 | 공차 기반 배관 그룹화 (보조) |
| `AnalyzeBranching.py` | 336 | 분기점(TEE/CROSS) 분석 |
| `VisualizeRouting3D.py` | 134 | Python/Plotly 3D 시각화 |
| `VisualizeGroupPipe3D.html` | - | Three.js 그룹 배관 뷰어 |
| `VisualizeAutoRouting3D.html` | - | Three.js V2 자동 경로 뷰어 |

### 1.3 변경 이력

| 날짜 | 버전 | 변경 내용 |
|------|------|----------|
| 2026-04-05 | V2.0 | 장비형상/장애물/종단점 고려 자동 설계, 8차원 특징량 |
| 2026-04-10 | V2.1 | 장애물 유형별 공간관계 특징량 18개 확장, 유사도 세분화 |

---

## 2. 전체 흐름도

### 2.1 3-Phase 파이프라인

```
┌─────────────────────────────────────────────────────────────────────┐
│                    원본 설계 JSON (data/input/)                       │
│  Equipment(장비BBox, POC목록, 부대장비)                               │
│  Nodes(1,183~3,472개) + Edges(879~2,588개)                          │
│  Obstacles(456~3,001개: 기둥/빔/그레이팅/포스트)                      │
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
│  ★ SpatialContext 로드    │             │
│  ★ obstacle_relations 저장│             │
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
│  ★ 5차원 유사도           │             │
│  (arrow+vector+range     │             │
│   +length+obstacle_rel)  │             │
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
│  ★ 유형별 장애물 분류    │                    │
│         │                    │               │
│         ▼                    ▼               │
│  EnhancedFeatureExtractor ──→ SpatialSimilarity │
│  (8차원 확장 특징량)         (8항목 가중 유사도)   │
│  ★ ObstacleRelationExtractor                  │
│  ★ 18개 장애물관계 특징량                        │
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
★ SpatialContext.load_from_json()  ── 장애물 유형별 분류 로딩
    ├─ structural_columns (COLUMN_STRUCTURE)
    ├─ posts (COLUMN_ARCHITECTURE)
    ├─ h_beams (BEAM_ARCHITECTURE)
    ├─ structural_beams (BEAM_STRUCTURE)
    └─ gratings (FLOOR_ARCHITECTURE)
    │
    ▼
find_routing_paths()  ── 장비별 순회
    │
    ▼  각 POC에 대해
_trace_paths_from_poc()  ── BFS 탐색
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
★ _compute_obstacle_relations_for_path()  ── 장애물관계 추출
    ├─ ObstacleRelationExtractor.extract()
    └─ 18개 유형별 특징량 → obstacle_relations 딕셔너리
    │
    ▼
JSON 저장 → RoutingResults/{파일}_{장비명}_Path.json
  (obstacle_relations 필드 포함)
```

### 2.3 Phase 2 상세 흐름

```
RoutingResults/*.json (30개)
    │
    ▼
_load_routing_records()  ── 경로 레코드 생성
    │  (장비/POC/유틸리티/경로특징량 + obstacle_relations 포함)
    ▼
GroupAnalyzer.find_groups()
    │
    ├─ 1단계: 버킷 분류
    │    Key = (equipment_name, equipment_id, process, maker, utility, size)
    │
    ├─ 2단계: 버킷 내 유사도 행렬 계산 (n×n)
    │    compute_composite_similarity()  ★ 5차원
    │    ├─ Arrow Similarity      ×0.25  ← Levenshtein Distance
    │    ├─ Vector Similarity     ×0.25  ← 코사인 유사도 시퀀스
    │    ├─ Range Similarity      ×0.15  ← BBox 범위 비교
    │    ├─ Length Similarity     ×0.15  ← 총 길이 비교
    │    └─ ★ Obstacle Relation  ×0.20  ← 유형별 장애물관계 유사도
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
    ├─ Obstacles: 456개 → ★ 유형별 세분화
    │    structural_columns: 18개 (구조기둥)
    │    posts: 192개 (Access Floor 포스트)
    │    h_beams: 32개 (H-Beam)
    │    structural_beams: 10개 (구조 보)
    │    gratings: 203개 (그레이팅)
    └─ SpaceInfo: CSF(z8000~13700), A/F(z13700~15500), CR(z15500~25000)
    │
    ▼
find_matching_group()  ── 그룹 + 대표 경로 매칭
    │
    ├─ 1차: equipment + utility + size 완전 일치 필터
    │
    ├─ 2차: 각 후보 그룹의 모든 경로와 확장 유사도 계산
    │    EnhancedFeatureExtractor.extract() → 8차원 특징량
    │    ★ ObstacleRelationExtractor.extract() → 18개 장애물관계
    │    SpatialSimilarity.compute() → 8항목 가중 유사도
    │    ├─ arrow     ×0.12 : 방향 패턴
    │    ├─ vector    ×0.12 : 벡터 시퀀스
    │    ├─ range     ×0.08 : BBox 범위
    │    ├─ length    ×0.08 : 총 길이
    │    ├─ equip_rel ×0.15 : 장비 상대 좌표
    │    ├─ terminal  ×0.15 : 종단점 유형/거리
    │    ├─ ★obstacle ×0.20 : 유형별 장애물관계 (기둥0.35+빔0.30+그레이팅0.20+포스트0.15)
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

### 3.1 RoutingGraph 클래스

#### 인스턴스 변수

| 변수 | 타입 | 설명 |
|------|------|------|
| `config` | AnalysisConfig | 분석 설정 |
| `node_by_guid` | Dict[str, Dict] | GUID → 노드 데이터 매핑 |
| `edge_by_guid` | Dict[str, Dict] | GUID → 엣지 데이터 매핑 |
| `equipment_list` | List[Dict] | 장비 목록 |
| `poc_owner_map` | Dict[str, List[str]] | POC GUID → 소유 장비 ID 목록 |

#### 메서드 상세

| 메서드 | 시그니처 | 반환 | 핵심 로직 |
|--------|----------|------|-----------|
| `load_from_json` | `(file_path: str) → bool` | 성공/실패 | Nodes/Edges/Equipment 파싱 → GUID 인덱싱, poc_owner_map 구축 |
| `get_neighbors` | `(node_guid: str) → List[Tuple[Dict, str]]` | (엣지, 다음GUID) | connectionGuidList 순회, VIRTUAL 제외, 중복 제거 |
| `is_branch_node` | `(node_guid: str) → bool` | 분기 여부 | TEE/BRANCH/JUNCTION/CROSS/WYE 또는 이웃≥3 |
| `find_routing_paths` | `() → Dict[str, Dict]` | 장비별 경로 | 장비 순회 → POC별 _trace_paths_from_poc 호출 |
| `_classify_terminal` | `(curr, node, start, eq_id) → Tuple[bool, str]` | (종단여부, 사유) | 8단계 종단 판정 |
| `_trace_paths_from_poc` | `(start_node, start_guid, eq_id) → List[Dict]` | 경로 목록 | BFS + 이중 사이클 방지 + 분기 추적 |

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

| 함수 | 시그니처 | 설명 |
|------|----------|------|
| `_compute_segment_code` | `(p1, p2, tol_deg) → str?` | 두 점 사이 이동을 R/H/D로 분류 |
| `_compute_path_arrow` | `(positions, tol_deg) → str` | 전체 경로의 segment 코드열 |
| `_extract_h_segments` | `(positions, arrow) → List[Dict]` | 연속 H 구간의 mean_z, mid_xy |
| `_bbox` | `(positions) → Dict?` | x/y/z min/max/range + xy_spread |
| `_compute_path_features` | `(steps, config) → Dict` | 전체 특징량 딕셔너리 반환 |
| ★ `_compute_obstacle_relations_for_path` | `(positions, spatial_ctx) → Dict?` | 장애물관계 18항목 딕셔너리 |

---

## 4. Phase 2: 그룹 클러스터링

### 4.1 유사도 함수 (★ V2.1 변경)

| 함수 | 가중치 | 알고리즘 |
|------|--------|----------|
| `pattern_similarity` | 0.25 | Levenshtein Distance 기반. 코드열 편집 거리 → 1 - dist/max_len |
| `_vector_sequence_similarity` | 0.25 | 코사인 유사도 평균 × 길이 커버리지(min_len/max_len) |
| `_range_similarity` | 0.15 | x,y,z 각 축의 1 - |v1-v2|/max(v1,v2,1) 평균 |
| `_length_similarity` | 0.15 | 1 - |len1-len2|/max(len1,len2,1) |
| ★ `_obstacle_relation_similarity` | 0.20 | 유형별 장애물관계 유사도 (기둥0.35+빔0.30+그레이팅0.20+포스트0.15) |
| `compute_composite_similarity` | 1.00 | 위 5개의 가중합 |

**V2.0 대비 변경**: 4차원(arrow 0.30 + vector 0.30 + range 0.20 + length 0.20) → 5차원(장애물관계 0.20 추가, 기존 비중 축소)

### 4.2 장애물관계 유사도 상세 (_obstacle_relation_similarity)

장애물관계 데이터가 없는 기존 결과 파일은 자동으로 1.0(중립) 반환 → **하위 호환성 유지**.

```
_obstacle_relation_similarity(r1, r2):
  ├─ 기둥(0.35): 근접수(0.2) + 최소거리(0.2) + 교차수(0.3) + LR패턴(0.3)
  ├─ 빔(0.30):   교차수(0.4) + clearance(0.3) + 평행비(0.3)
  ├─ 그레이팅(0.20): 커버리지(0.4) + 하부수(0.3) + 개구부(0.3)
  └─ 포스트(0.15):   밀도(0.3) + 그리드정렬(0.4) + 근접수(0.3)
```

### 4.3 GroupAnalyzer 클래스

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

### 4.4 Zone 추정 알고리즘 (detect_zones)

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
| `max_start_xy_distance` | 8,000mm | 후보 그룹 필터 반경 |
| `elbow_clearance` | 100mm | 엘보우 최소 직관 길이 |
| `trunk_approach_tolerance` | 300mm | trunk Z 접근 허용 오차 |
| `min_segment_length` | 50mm | 최소 세그먼트 길이 |
| `fitting_length_threshold` | 150mm | 피팅 판별 기준 |
| `max_length_ratio` | 2.5 | 길이 검증 상한 |
| `min_length_ratio` | 0.2 | 길이 검증 하한 |

---

## 6. Phase 3-V2: 확장 자동 설계

### 6.1 V1 대비 핵심 변경사항

| 항목 | V1 | V2 (V2.1) |
|------|----|----|
| **입력** | GroupPipeResults만 | + 원본 JSON (장비/장애물/SpaceInfo) |
| **특징량** | 4차원 (arrow, vector, range, length) | **8차원** (+장비상대, 종단점, 장애물관계, 레벨) |
| **장애물** | 미고려 | ★ **유형별 분류** (기둥/포스트/H-Beam/그레이팅) |
| **유사도** | 4항목 균등 가중 | **8항목 세분화 가중치** (장애물 0.20) |
| **경로 생성** | Zone 기반 템플릿 변형만 | + **기둥 충돌 감지 + 우회** |
| **검증** | 4항목 | 5항목 (**+기둥 충돌 검사**) |

### 6.2 SpatialContext 클래스 (★ V2.1 확장)

원본 JSON에서 공간 정보를 로딩하고 쿼리를 제공합니다.

#### 인스턴스 변수

| 변수 | 타입 | 설명 |
|------|------|------|
| `equipment` | EquipmentInfo | 장비 정보 (BBox, POC 67개, ends 14개) |
| `obstacles` | List[ObstacleInfo] | 전체 장애물 456개 |
| `columns` | List[ObstacleInfo] | 기둥 전체 (하위 호환) |
| `beams` | List[ObstacleInfo] | 빔 전체 (하위 호환) |
| ★ `structural_columns` | List[ObstacleInfo] | 구조 기둥 18개 (COLUMN_STRUCTURE) |
| ★ `posts` | List[ObstacleInfo] | 포스트 192개 (COLUMN_ARCHITECTURE) |
| ★ `h_beams` | List[ObstacleInfo] | H-Beam 32개 (BEAM_ARCHITECTURE) |
| ★ `structural_beams` | List[ObstacleInfo] | 구조 보 10개 (BEAM_STRUCTURE) |
| ★ `gratings` | List[ObstacleInfo] | 그레이팅 203개 (FLOOR_ARCHITECTURE) |
| `levels` | List[SpaceLevelInfo] | CSF/A/F/CR 3개 레벨 |
| `ends_map` | Dict[str, Dict] | 부대장비 GUID → end 정보 |

#### 공간 쿼리 메서드

| 메서드 | 시그니처 | 설명 |
|--------|----------|------|
| `find_obstacles_in_path` | `(p1, p2, clearance) → List[ObstacleInfo]` | 선분-BBox 교차 판정 (Slab method) |
| `find_nearby_obstacles` | `(point, radius) → List[(float, ObstacleInfo)]` | 반경 내 장애물 (거리순 정렬) |
| `compute_obstacle_density` | `(center, radius) → float` | 장애물 밀도 (기둥 가중치 3, 빔 가중치 1) |
| `get_level_at_z` | `(z) → SpaceLevelInfo?` | Z 좌표의 공간 레벨 |

### 6.3 EnhancedPathFeatures (★ V2.1 확장)

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
| **장애물** | `obstacle_count_nearby` | int | 반경 내 장애물 수 (하위호환) |
| **장애물** | `obstacle_density` | float | 장애물 밀도 0~1 (하위호환) |
| **장애물** | `obstacle_min_distance` | float | 최근접 장애물 거리 (하위호환) |
| **장애물** | `column_crossings` | int | 기둥 교차 수 (하위호환) |
| ★ **장애물관계** | `obstacle_relations` | ObstacleRelationFeatures | **유형별 18개 공간관계** (섹션 7 참조) |
| **레벨** | `start_level` | str | 시작점 레벨 |
| **레벨** | `end_level` | str | 종점 레벨 |
| **레벨** | `levels_traversed` | List[str] | 경유 레벨 시퀀스 |
| **레벨** | `level_change_count` | int | 레벨 변경 횟수 |

### 6.4 SpatialSimilarity 8항목 가중 유사도 (★ V2.1 변경)

| 항목 | V2.0 가중치 | ★ V2.1 가중치 | 계산 방법 |
|------|------------|--------------|-----------|
| `arrow` | 0.15 | **0.12** | Levenshtein Distance 기반 |
| `vector` | 0.15 | **0.12** | 코사인 유사도 시퀀스 × 길이 커버리지 |
| `range` | 0.10 | **0.08** | BBox x,y,z 범위 비교 평균 |
| `length` | 0.10 | **0.08** | 총 길이 비율 |
| `equip_relative` | 0.15 | 0.15 | 시작점(0.4)+종점(0.4)+출발면 보너스(0.2) |
| `terminal` | 0.15 | 0.15 | 타입(0.4)+거리(0.3)+레벨(0.3) |
| `obstacle` | 0.10 | **0.20** | ★ **유형별 세분화** (섹션 7.4 참조) |
| `level` | 0.10 | 0.10 | 시작(0.25)+종점(0.25)+변경수(0.25)+Jaccard(0.25) |

### 6.5 ObstacleAwarePathBuilder 장애물 회피

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

### 6.6 PathValidatorV2 5항목 검증

| 검증 항목 | 가중치 | 기준 |
|-----------|--------|------|
| `continuity` | 0.20 | start_pos + Σ(vectors) == positions, 오차 < 1.5mm |
| `length_ratio` | 0.20 | 생성길이/템플릿길이 ∈ [0.2, 2.5] |
| `pattern` | 0.15 | pattern_similarity(gen_arrow, tmpl_arrow) |
| `obstacle_free` | 0.25 | H 세그먼트의 COLUMN_STRUCTURE 충돌 수 × 0.3 감점 |
| `zone_compliance` | 0.20 | 경로 BBox가 trunk의 1.5배 범위 이내 |

**합격 조건**: quality_score >= 0.4 AND continuity == 1.0

---

## 7. 장애물 유형별 공간관계 시스템 (★ V2.1 신규)

### 7.1 ObstacleCategory 분류 체계

| ObstacleCategory | ddworksType | ostType | 실제 객체 | 수량 (KSCTA01) |
|---|---|---|---|---|
| STRUCTURAL_COLUMN | COLUMN_STRUCTURE | OST_StructuralColumns | 구조 기둥 (1300x1400mm) | 18 |
| POST | COLUMN_ARCHITECTURE | OST_Columns | Access Floor 포스트 | 192 |
| H_BEAM | BEAM_ARCHITECTURE | OST_BeamStartSegment | H-Beam (CSF/FSF) | 32 |
| STRUCTURAL_BEAM | BEAM_STRUCTURE | OST_StructuralFraming | 구조 보 (3600x3600) | 10 |
| GRATING | FLOOR_ARCHITECTURE | OST_Floors | Grating (그레이팅) | 203 |
| CEILING | CEILING_ARCHITECTURE | OST_Ceilings | 천장 | 1 |

### 7.2 ObstacleRelationFeatures (18개 특징량)

#### 7.2.1 구조 기둥 (STRUCTURAL_COLUMN) - 5개

| 필드 | 타입 | 설명 | 알고리즘 |
|------|------|------|----------|
| `col_count_nearby` | int | 경로 근방 기둥 수 | 경로 중심에서 radius 내 기둥 카운트 |
| `col_min_distance` | float | 최근접 기둥 거리 (mm) | BBox 점-거리 최소값 |
| `col_avg_distance` | float | 평균 거리 (mm) | 근방 기둥 거리 평균 |
| `col_crossings` | int | 경로-기둥 교차 수 | 세그먼트별 Slab method (margin 200mm) |
| `col_relative_pattern` | str | 좌/우 배치 패턴 | 경로 진행방향 법선벡터 기준 L/R/B 판별 |

**LR 패턴 알고리즘**: 각 세그먼트의 법선벡터(좌측 방향)를 계산하고, 기둥 중심의 내적 부호로 좌(L)/우(R)/양쪽(B) 판별.

#### 7.2.2 포스트 (POST) - 3개

| 필드 | 타입 | 설명 | 알고리즘 |
|------|------|------|----------|
| `post_count_nearby` | int | 근방 포스트 수 | 경로 중심에서 radius 내 카운트 |
| `post_density` | float | 포스트 밀도 (0~1) | 포스트 수 / BBox XY면적 (정규화) |
| `post_grid_alignment` | float | 그리드 정렬도 (0~1) | 그리드 규칙성(50%) + 경로 직교정렬(50%) |

**그리드 정렬도 알고리즘**: 포스트 간 X/Y 간격의 변동계수(CV)로 규칙성 계산 → 경로 세그먼트의 축 정렬도와 가중 평균.

#### 7.2.3 H-Beam (H_BEAM + STRUCTURAL_BEAM) - 3개

| 필드 | 타입 | 설명 | 알고리즘 |
|------|------|------|----------|
| `beam_count_crossing` | int | 빔 교차 수 | Slab method (margin 100mm) |
| `beam_min_clearance` | float | 최소 수직 간격 (mm) | 빔 하단 Z - 경로 Z 차이 |
| `beam_parallel_ratio` | float | 평행 비율 (0~1) | 코사인 > 0.7인 빔 비율 |

#### 7.2.4 그레이팅 (GRATING) - 3개

| 필드 | 타입 | 설명 | 알고리즘 |
|------|------|------|----------|
| `grating_coverage` | float | 경로 하부 커버리지 (0~1) | 수평 세그먼트 중심점 XY범위 + Z조건 판정 |
| `grating_count_below` | int | 경로 아래 그레이팅 수 | radius 내 그레이팅 카운트 |
| `grating_gap_count` | int | 그레이팅 빈틈 수 | 커버 → 미커버 전환 횟수 |

### 7.3 ObstacleRelationExtractor 클래스

```python
class ObstacleRelationExtractor:
    def __init__(self, spatial: SpatialContext)
    
    def extract(self, positions, radius) → ObstacleRelationFeatures
        ├─ _compute_column_relations()   # 구조기둥 5개 특징량
        ├─ _compute_post_relations()     # 포스트 3개 특징량
        ├─ _compute_beam_relations()     # H-Beam 3개 특징량
        └─ _compute_grating_relations()  # 그레이팅 3개 특징량
    
    # 보조 함수
    _compute_lr_pattern()      # 기둥 L/R/B 패턴
    _compute_grid_alignment()  # 포스트 그리드 정렬도
    _path_center()             # 경로 중심점 계산
```

### 7.4 유형별 유사도 계산

SpatialSimilarity._obstacle_sim() 내부 세분화:

| 유형 | 유사도 가중치 | 내부 항목별 가중치 | 근거 |
|------|------------|-------------------|------|
| 구조기둥 | **0.35** | 교차수(0.3) + LR패턴(0.3) + 근접수(0.2) + 거리(0.2) | 배관 경로에 가장 큰 영향 |
| H-Beam | **0.30** | 교차수(0.4) + clearance(0.3) + 평행비(0.3) | 수직 공간 제약 핵심 |
| 그레이팅 | **0.20** | 커버리지(0.4) + 하부수(0.3) + 개구부(0.3) | 하부 구조적 지지 |
| 포스트 | **0.15** | 그리드정렬(0.4) + 밀도(0.3) + 근접수(0.3) | 간접적 영향, 예측 가능 |

### 7.5 검증 결과

| 항목 | 결과 |
|------|------|
| 456개 장애물 유형 분류 | 5개 유형 정확 분류 (18+192+32+10+203+1) |
| 동일 경로 자기 유사도 | **1.0000** |
| 다른 영역 경로 교차 유사도 | **0.5246** |
| 세부: 포스트 | 0.8615 (높은 환경 유사성) |
| 세부: H-Beam | 0.6000 (중간 차이) |
| 세부: 그레이팅 | 0.4467 (커버리지 차이) |
| 세부: 구조기둥 | 0.3600 (위치/패턴 차이) |

---

## 8. 보조 분석 모듈

### 8.1 analyze_group_pipes.py (743줄)

**목적**: 공차/방향/근접도 기반 배관 그룹화 (Phase 2의 보조)

**전역 상수**:
```python
TOL_Z_ELEVATION = 100.0mm    # 수평 배관 Z 공차
TOL_ANGLE_DEG = 3.0°         # 방향 평행성 공차
MAX_SPACING = 300.0mm        # 배관 간 최대 간격
MAX_LONGITUDINAL_GAP = 1000mm # 종방향 허용 갭
TOL_ALIGNMENT = 100.0mm      # 축 정렬 오차
```

### 8.2 AnalyzeBranching.py (336줄)

**목적**: 분기점(TEE/CROSS) 분석 및 종단 추적

### 8.3 VisualizeRouting3D.py (134줄)

**목적**: Plotly 기반 Python 3D 시각화

---

## 9. 3D 시각화

### 9.1 VisualizeAutoRouting3D.html

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

**좌표 변환**: 설계 좌표(X,Y,Z) → Three.js(X,Z,-Y) Y-up 변환

### 9.2 VisualizeGroupPipe3D.html

**렌더링 레이어**:
- 58개 그룹별 고유 HSL 색상 경로
- Zone 박스: Trunk(파랑), Fan-In(초록), Fan-Out(주황)
- 자동 생성 경로: 점선 + 밝은 녹색 + AUTO 태그

---

## 10. 데이터 형식 명세

### 10.1 입력: 원본 설계 JSON

```json
{
  "FileInfo": {
    "SpaceInfo": [
      {"levelName": "CSF", "boundary": {"min": {"z":8000}, "max": {"z":13700}}}
    ]
  },
  "Equipment": {
    "guid": "...", "name": "kscta01", "process": "CMP",
    "boundaryBox": {"min": {...}, "max": {...}},
    "pocList": [...], "ends": [...]
  },
  "Obstacles": [
    {
      "obstacleId": "...",
      "ddworksType": "COLUMN_STRUCTURE",
      "ostType": "OST_StructuralColumns",
      "name": "기둥명",
      "boundary": {"min": {"x":0,"y":33800,"z":0}, "max": {"x":1300,"y":35200,"z":25000}}
    }
  ],
  "Nodes": [...],
  "Edges": [...]
}
```

### 10.2 Phase 1 출력: RoutingResults (★ V2.1 확장)

```json
{
  "source_file": "...", "equipment_name": "kscta01",
  "poc_paths": [{
    "start_poc_id": "...",
    "paths": [{
      "path_arrow": "R-H-H-R-R-H-R",
      "path_step_vectors": [{"x":0,"y":0,"z":-588}, ...],
      "path_step_lengths": [588.0, 100.0, ...],
      "path_total_length": 4896.0,
      "obstacle_relations": {
        "col_count_nearby": 1, "col_min_distance": 1795.0,
        "col_avg_distance": 1795.0, "col_crossings": 0,
        "col_relative_pattern": "LLR",
        "post_count_nearby": 21, "post_density": 1.0,
        "post_grid_alignment": 1.0,
        "beam_count_crossing": 0, "beam_min_clearance": 138.0,
        "beam_parallel_ratio": 1.0,
        "grating_coverage": 0.0, "grating_count_below": 22,
        "grating_gap_count": 0
      },
      "steps": [...]
    }]
  }]
}
```

### 10.3 Phase 2 출력: GroupPipeResults

```json
[{
  "group_id": 1,
  "equipment_name": "kscta01", "utility": "AKWW", "size": "20A",
  "path_count": 36, "avg_similarity": 0.749,
  "common_z_levels": [{"mean_z": 14699.1, "path_count": 36}],
  "zones": {"trunk": {...}, "fan_in": {...}, "fan_out": {...}},
  "paths": [{
    "poc_id": "...", "start_pos": [5459.2, 28503.2, 15495.0],
    "path_arrow": "R-H-H-R-R-R-R-H-H-R-R-R-R-H-R",
    "path_step_vectors": [...], "path_total_length": 4896.0,
    "obstacle_relations": {...}
  }]
}]
```

### 10.4 Phase 3-V2 출력: AutoRoutingResults

```json
[{
  "group_id": "AUTO_001", "is_auto_generated": true, "version": "V2",
  "quality_score": 0.934, "match_score": 0.661,
  "validation": {
    "is_valid": true, "quality_score": 0.934,
    "checks": {"continuity":1.0, "length_ratio":0.95, "pattern":0.667,
               "obstacle_free":1.0, "zone_compliance":1.0},
    "collision_count": 0
  },
  "similarity_detail": {
    "arrow":0.667, "vector":0.52, "range":0.85, "length":0.78,
    "equip_relative":0.92, "terminal":0.71,
    "obstacle":0.88,
    "obstacle_column":0.36, "obstacle_post":0.86,
    "obstacle_beam":0.60, "obstacle_grating":0.45,
    "level":1.0
  },
  "paths": [{
    "start_pos": [5500,28200,15495], "end_pos": [5300,27000,15495],
    "path_arrow": "R-H-R-R-H-H-R-R-H-R-H",
    "path_step_vectors": [...], "path_total_length": 3407.8
  }]
}]
```

---

## 11. 설정 파라미터 레퍼런스

### 11.1 AnalysisConfig (Phase 1+2)

| 파라미터 | 기본값 | CLI 옵션 | 영향 |
|----------|--------|----------|------|
| `input_dir` | ./data-v10 | --input | 원본 JSON 디렉토리 |
| `routing_out` | ./RoutingResults | --routing_out | Phase 1 출력 |
| `group_out` | ./GroupPipeResults | --group_out | Phase 2 출력 |
| `direction_angle_tolerance` | 5.0° | - | R/H/D 분류 기준 |
| `max_paths_per_poc` | 5,000 | --max_paths_per_poc | POC별 최대 경로 수 |
| `max_queue_size` | 100,000 | --max_queue_size | BFS 큐 한도 |
| `max_depth` | 512 | --max_depth | BFS 최대 깊이 |
| `pattern_similarity_min` | 0.70 | --pattern_similarity_min | 그룹 인정 최소 유사도 |
| `start_poc_xy_max` | 5,000mm | --start_poc_xy_max | 시작 POC XY 거리 제한 |
| `tol_z_level` | 200mm | --tol_z_level | 공통 레벨 Z 허용 오차 |
| `min_group_size` | 2 | --min_group_size | 최소 그룹 경로 수 |

### 11.2 DesignConfigV2 (Phase 3-V2) (★ V2.1 변경)

| 파라미터 | 기본값 | 영향 |
|----------|--------|------|
| `obstacle_clearance` | 200mm | 장애물 충돌 판정 마진 |
| `obstacle_detour_margin` | 300mm | 우회 시 추가 마진 |
| `max_start_xy_distance` | 8,000mm | 템플릿 매칭 반경 |
| `fitting_length_threshold` | 150mm | 피팅 판별 (이하의 R 세그먼트) |
| `w_arrow` | **0.12** | 방향 패턴 유사도 가중치 |
| `w_vector` | **0.12** | 벡터 시퀀스 유사도 가중치 |
| `w_range` | **0.08** | BBox 범위 유사도 가중치 |
| `w_length` | **0.08** | 총 길이 유사도 가중치 |
| `w_equip_relative` | 0.15 | 장비 상대 좌표 유사도 가중치 |
| `w_terminal` | 0.15 | 종단점 유사도 가중치 |
| `w_obstacle_proximity` | **0.20** | ★ 장애물 유형별 공간관계 유사도 가중치 |
| `w_level` | 0.10 | 레벨 유사도 가중치 |
| `max_length_ratio` | 2.5 | 검증: 길이 상한 비율 |
| `min_length_ratio` | 0.2 | 검증: 길이 하한 비율 |

### 11.3 전역 상수

| 상수 | 파일 | 값 | 설명 |
|------|------|-----|------|
| `TYPE_ABBREV` | AnalyzeRoutingAi_V2.py | 31개 매핑 | 노드 타입 → 2글자 약어 |
| `BRANCH_NODE_TYPES` | AnalyzeRoutingAi_V2.py | {TEE, BRANCH, JUNCTION, CROSS, WYE} | 분기 판정 타입 |
| `FITTING_LENGTH_THRESHOLD` | AutoRoutingDesigner_V2.py | 150mm | 피팅 세그먼트 판별 |
| `OBSTACLE_CLEARANCE` | AutoRoutingDesigner_V2.py | 200mm | 배관~장애물 최소 거리 |
| `TERMINAL_TYPES` | AutoRoutingDesigner_V2.py | 5개 매핑 | 종단 타입 분류 |
| ★ `_DDWORKS_TO_CATEGORY` | AutoRoutingDesigner_V2.py | 6개 매핑 | ddworksType → ObstacleCategory |

---

**문서 작성일**: 2026-04-10  
**프로젝트 상태**: Active Development  
**Python 의존성**: 표준 라이브러리만 (외부 패키지 불필요)  
**GitHub**: https://github.com/overcomsoft/RoutingAI
