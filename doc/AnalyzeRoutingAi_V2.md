# AnalyzeRoutingAi_V2.py 개발 문서

## 1. 개요

**파일**: `AnalyzeRoutingAi_V2.py` (1,400줄)  
**목적**: 3D 배관 설계 데이터(JSON)에서 개별 배관 경로를 BFS로 추출하고, 추출된 경로의 형상·방향·길이·공간 정보를 기반으로 **그룹 배관(Trunk/Bundle)** 후보를 분석하는 개선 버전

**V1 대비 개선 포인트**:
1. 예외를 묵살하지 않고 파일명과 원인을 로깅
2. 경로 특징량을 문자열이 아닌 구조화된 숫자 리스트로도 함께 저장
3. 시작-끝 절대 변위 대신 전체 노드의 BBox Range 사용
4. 같은 장비 내부 POC와 다른 장비 POC를 구분하여 종단 판정 개선
5. 코사인 유사도를 0~1 범위로 정규화
6. Grouping 시 size까지 버킷 키에 포함하여 오검출 감소
7. Phase 1(Routing) / Phase 2(Grouping) 분리 실행 가능
8. 경로 수, 깊이, 큐 길이에 대한 안전 장치 추가

---

## 2. 전체 흐름도

```
┌──────────────────────────────────────────────────────────────┐
│                         CLI (main)                           │
│   --phase all | routing | grouping                           │
└─────────────┬────────────────────────────┬───────────────────┘
              │                            │
              ▼                            ▼
┌─────────────────────────┐   ┌──────────────────────────────┐
│   Phase 1: Routing      │   │   Phase 2: Grouping          │
│   run_phase_routing()   │   │   run_phase_grouping()       │
└─────────┬───────────────┘   └──────────┬───────────────────┘
          │                              │
          ▼                              ▼
┌─────────────────────────┐   ┌──────────────────────────────┐
│ 1. data-v10/*.json 로드 │   │ 1. RoutingResults/*.json 로드│
│ 2. RoutingGraph 생성    │   │    _load_routing_records()   │
│ 3. BFS 경로 탐색        │   │ 2. GroupAnalyzer 생성        │
│    find_routing_paths() │   │ 3. 버킷별 유사도 클러스터링  │
│ 4. 특징량 계산           │   │    find_groups()             │
│    _compute_path_features│   │ 4. 영역 추정                │
│ 5. JSON 저장             │   │    detect_zones()            │
│    RoutingResults/       │   │ 5. JSON + CSV 저장           │
│    *_Path.json           │   │    GroupPipeResults/          │
└─────────────────────────┘   └──────────────────────────────┘
```

### 상세 Phase 1 흐름

```
data-v10/*.json
    │
    ▼
RoutingGraph.load_from_json()
    ├─ Nodes → node_by_guid (Dict)
    ├─ Edges → edge_by_guid (Dict)
    ├─ Equipment → equipment_list (List)
    └─ POC → poc_owner_map (Dict)
    │
    ▼
find_routing_paths()
    │  장비 목록 순회
    ▼
    ┌─ 장비별 pocList에서 POC 노드 추출
    └─ POC마다 _trace_paths_from_poc() 호출
           │
           ▼  BFS 탐색
           ├─ 큐에서 (node_guid, record_idx, visited_edges, depth) 꺼냄
           ├─ _classify_terminal() → 종단이면 경로 저장
           ├─ get_neighbors() → 이웃 노드 탐색
           ├─ is_branch_node() → 분기 판정
           ├─ _is_in_path() → 사이클 방지
           └─ 큐에 다음 노드 추가
           │
           ▼
    경로별 _compute_path_features() 계산
    ├─ path_arrow (R/H/D 코드열)
    ├─ path_bbox (BoundingBox)
    ├─ path_step_vectors (방향 벡터 시퀀스)
    ├─ path_step_lengths (구간 길이 시퀀스)
    └─ h_segments (수평 구간 정보)
           │
           ▼
    JSON 파일 저장 → RoutingResults/{파일}_{장비명}_Path.json
```

### 상세 Phase 2 흐름

```
RoutingResults/*.json
    │
    ▼
_load_routing_records()
    │  경로별 레코드 생성 (장비/POC/유틸리티/경로특징량 포함)
    ▼
GroupAnalyzer.find_groups()
    │
    ├─ 1단계: 버킷 분류
    │    Key = (equipment_name, equipment_id, process, maker, utility, size)
    │
    ├─ 2단계: 버킷 내 유사도 행렬 계산
    │    compute_composite_similarity()
    │    ├─ Arrow Similarity (0.30)    ← Levenshtein 기반
    │    ├─ Vector Similarity (0.30)   ← 코사인 유사도 시퀀스
    │    ├─ Range Similarity (0.20)    ← BBox 범위 비교
    │    └─ Length Similarity (0.20)   ← 총 길이 비교
    │
    ├─ 3단계: Union-Find 클러스터링
    │    유사도 >= pattern_similarity_min → 같은 그룹
    │
    ├─ 4단계: 필터링
    │    ├─ min_group_size 이상
    │    ├─ 시작 POC XY 거리 <= start_poc_xy_max
    │    └─ (선택) 공통 Z 레벨 존재 여부
    │
    └─ 5단계: 결과 정렬 (-path_count, -avg_similarity)
    │
    ▼
detect_zones() - 그룹별 영역 추정
    ├─ trunk: 공통 수평 레벨 중 가장 밀집된 영역
    ├─ fan_in: trunk 앞쪽(아래쪽) 구간
    └─ fan_out: trunk 뒤쪽(위쪽) 구간
    │
    ▼
JSON + CSV 저장 → GroupPipeResults/group_pipe_results_{timestamp}.*
```

---

## 3. 모듈 구조

| 섹션 | 줄 번호 | 내용 |
|------|---------|------|
| 0. 설정 값 | 42~80 | 상수, `AnalysisConfig` 데이터클래스 |
| 1. 공통 유틸리티 | 84~378 | 텍스트 정규화, 좌표 계산, 경로 특징량 함수 |
| 2. BFS 경로 탐색기 | 380~728 | `RoutingGraph` 클래스 |
| 3. Grouping 유사도 | 730~1062 | 유사도 함수, `GroupAnalyzer` 클래스, `detect_zones` |
| 4. Phase 1 입출력 | 1066~1216 | `run_phase_routing`, `_load_routing_records` |
| 5. Phase 2 결과 저장 | 1220~1332 | `run_phase_grouping` |
| 6. CLI | 1336~1400 | `build_arg_parser`, `main` |

---

## 4. 클래스 상세

### 4.1 AnalysisConfig (데이터클래스, 줄 58~78)

설정 값을 하나의 객체로 관리하는 데이터클래스.

| 필드명 | 타입 | 기본값 | 설명 |
|--------|------|--------|------|
| `input_dir` | str | `"./data-v10"` | 원본 JSON 디렉토리 경로 |
| `routing_out` | str | `"./RoutingResults"` | Phase 1 결과 저장 경로 |
| `group_out` | str | `"./GroupPipeResults"` | Phase 2 결과 저장 경로 |
| `max_branch_count` | int | 8 | 과도한 분기 수 경고 임계값 |
| `direction_angle_tolerance` | float | 5.0 | R/H/D 코드 판정 각도 공차 (°) |
| `max_paths_per_poc` | int | 5000 | POC별 최대 경로 수 제한 |
| `max_queue_size` | int | 100000 | BFS 큐 최대 크기 |
| `max_depth` | int | 512 | BFS 최대 탐색 깊이 |
| `pattern_similarity_min` | float | 0.70 | 그룹 인정 최소 유사도 |
| `start_poc_xy_max` | float | 5000.0 | 시작 POC 간 최대 XY 거리 (mm) |
| `tol_z_level` | float | 200.0 | 공통 수평 레벨 Z 허용 오차 (mm) |
| `trunk_max_xy_spread` | float | 1500.0 | Trunk 최대 XY spread (mm) |
| `trunk_z_band_factor` | float | 2.0 | Trunk Z 대역 배수 |
| `min_group_size` | int | 2 | 최소 그룹 경로 수 |
| `require_common_z_levels` | bool | False | 공통 Z 레벨 필수 여부 |

---

### 4.2 RoutingGraph (클래스, 줄 385~727)

JSON 설계 데이터를 그래프 구조로 로드하고, BFS 알고리즘으로 모든 배관 경로를 탐색하는 핵심 클래스.

#### 인스턴스 변수

| 변수명 | 타입 | 설명 |
|--------|------|------|
| `config` | AnalysisConfig | 분석 설정 객체 |
| `node_by_guid` | Dict[str, Dict] | GUID → 노드 데이터 매핑 |
| `edge_by_guid` | Dict[str, Dict] | GUID → 엣지 데이터 매핑 |
| `equipment_list` | List[Dict] | 장비 목록 |
| `poc_owner_map` | Dict[str, List[str]] | POC GUID → 소유 장비 ID 목록 |

#### 메서드

| 메서드 | 줄 | 반환 | 설명 |
|--------|-----|------|------|
| `load_from_json(file_path)` | 395 | bool | JSON 파일을 읽어 노드/엣지/장비 인덱스를 생성. 실패 시 False |
| `get_neighbors(node_guid)` | 442 | List[Tuple[Dict, str]] | 현재 노드에서 이동 가능한 (엣지, 다음노드GUID) 목록. VIRTUAL 엣지 제외, 중복 제거 |
| `is_branch_node(node_guid)` | 495 | bool | 분기 노드 판정 (TEE/BRANCH/JUNCTION/CROSS/WYE 또는 이웃 3개 이상) |
| `find_routing_paths()` | 518 | Dict[str, Dict] | 모든 장비의 POC를 시작점으로 BFS 경로 탐색 실행 |
| `_classify_terminal(curr_guid, curr_node, start_guid, current_equipment_id)` | 557 | Tuple[bool, str] | 종단 노드 판정 및 사유 반환 |
| `_trace_paths_from_poc(start_node, start_guid, current_equipment_id)` | 603 | List[Dict] | 단일 POC에서 BFS로 모든 종단 경로 추출 |
| `_is_in_path(node_records, curr_idx, target_guid)` | 707 | bool | 부모 포인터 역추적으로 사이클 검사 |
| `_reconstruct_path(node_records, end_idx)` | 716 | List[Dict] | 부모 포인터로 start→end 방향 NODE/EDGE 목록 복원 |

---

### 4.3 GroupAnalyzer (클래스, 줄 846~993)

Phase 1에서 추출된 경로 레코드들을 그룹 배관 후보로 클러스터링하는 클래스.

#### 인스턴스 변수

| 변수명 | 타입 | 설명 |
|--------|------|------|
| `records` | List[Dict] | Phase 1에서 로딩된 경로 레코드 목록 |
| `config` | AnalysisConfig | 분석 설정 객체 |

#### 메서드

| 메서드 | 줄 | 반환 | 설명 |
|--------|-----|------|------|
| `find_groups()` | 853 | List[Dict] | 버킷 분류 → 유사도 행렬 → Union-Find 클러스터링 → 필터링 → 후보 반환 |
| `_find_common_z_levels(recs)` | 947 | List[Dict] | 수평(H) 구간의 mean_z를 이용해 공통 레벨 탐색 |

---

## 5. 함수 상세

### 5.1 공통 유틸리티 함수 (섹션 1)

| 함수 | 줄 | 인자 | 반환 | 설명 |
|------|-----|------|------|------|
| `_normalize_text(value)` | 89 | Any | str | 공백 제거 + 대문자 정규화. None → 빈 문자열 |
| `_get_guid(obj)` | 97 | Optional[Dict] | Optional[str] | guid/Guid/id/Id 키 중 사용 가능한 식별자 반환 |
| `_safe_filename(value)` | 105 | str | str | 파일명 안전 문자열 생성 (알파벳, 숫자, `-`, `_`만 허용) |
| `_get_position(node_data)` | 113 | Dict | Tuple[float,float,float] | 노드에서 (x,y,z) 좌표 추출. `position` 배열 또는 개별 x/y/z 키 지원 |
| `_dist_xy(p1, p2)` | 126 | Sequence, Sequence | float | 두 점 간 XY 평면 거리 |
| `_dist_xyz(p1, p2)` | 131 | Sequence, Sequence | float | 두 점 간 3D 거리 |
| `_xy_spread(positions)` | 136 | Sequence | float | 점 집합의 최대 XY 거리 (모든 조합 중 최대값) |
| `_bbox(positions)` | 143 | Sequence | Optional[Dict] | 3D 좌표 집합의 BoundingBox 계산 (min/max/range + xy_spread + node_count) |
| `_build_path_summary(steps)` | 166 | Sequence[Dict] | str | NODE 타입 약어를 `->` 로 연결한 요약 문자열 (예: `PO->EB->FL->TE`) |
| `_compute_segment_code(p1, p2, tolerance_deg)` | 184 | Sequence, Sequence, float | Optional[str] | 두 점 사이 이동을 R(수직)/H(수평)/D(대각) 코드로 변환 |
| `_compute_path_arrow(positions, tolerance_deg)` | 202 | Sequence, float | str | 경로 전체의 segment 코드열 (예: `H-H-R-H-D`) |
| `_extract_h_segments(positions, arrow)` | 216 | Sequence, str | List[Dict] | 연속 H 구간의 mean_z, mid_xy, node_count 계산 |
| `_serialize_vectors(vectors)` | 252 | Sequence[Dict] | str | 벡터 딕셔너리 → 문자열 (하위 호환용) |
| `_serialize_lengths(lengths)` | 257 | Sequence[float] | str | 길이 리스트 → 문자열 (하위 호환용) |
| `_parse_legacy_vectors(value)` | 262 | Any | List[Dict] | 문자열/리스트/딕셔너리 → 표준 벡터 리스트 변환 |
| `_parse_legacy_lengths(value)` | 301 | Any | List[float] | 문자열/리스트 → 표준 길이 리스트 변환 |
| `_clamp01(value)` | 317 | float | float | 값을 0.0~1.0 범위로 클램핑 |
| `_ensure_list_of_positions(steps)` | 322 | Sequence[Dict] | List[Tuple] | steps에서 NODE 타입만 필터하여 좌표 리스트 추출 |
| `_compute_path_features(steps, config)` | 327 | Sequence[Dict], Config | Dict | **핵심 함수** - 경로의 모든 특징량(arrow, bbox, vectors, lengths, h_segments) 일괄 계산 |

### 5.2 유사도 계산 함수 (섹션 3)

| 함수 | 줄 | 인자 | 반환 | 설명 |
|------|-----|------|------|------|
| `pattern_similarity(arrow1, arrow2)` | 735 | str, str | float | Levenshtein Distance 기반 방향 패턴 유사도 (0~1) |
| `_cosine_similarity_0_1(v1, v2)` | 760 | Dict, Dict | float | 코사인 유사도를 0~1 범위로 정규화 (반대 방향=0) |
| `_vector_sequence_similarity(v1, v2)` | 783 | List[Dict], List[Dict] | float | 벡터 시퀀스 유사도 (코사인 유사도 평균 × 길이 커버리지) |
| `_range_similarity(r1, r2)` | 805 | Dict, Dict | float | BBox 범위(x,y,z) 유사도 |
| `_length_similarity(l1, l2)` | 816 | List[float], List[float] | float | 총 길이 유사도 |
| `compute_composite_similarity(r1, r2)` | 827 | Dict, Dict | float | **복합 유사도** = Arrow(0.30) + Vector(0.30) + Range(0.20) + Length(0.20) |

### 5.3 영역 추정 함수

| 함수 | 줄 | 인자 | 반환 | 설명 |
|------|-----|------|------|------|
| `detect_zones(candidate, config)` | 997 | Dict, Config | Dict | 그룹 후보의 trunk / fan_in / fan_out 영역 BBox 추정 |

### 5.4 Phase 실행 함수 (섹션 4~5)

| 함수 | 줄 | 인자 | 반환 | 설명 |
|------|-----|------|------|------|
| `run_phase_routing(config)` | 1070 | Config | List[str] | Phase 1 - 원본 JSON → BFS 경로 추출 → JSON 저장 |
| `_load_routing_records(config)` | 1137 | Config | List[Dict] | Phase 1 결과 JSON → Grouping용 레코드 리스트 변환 |
| `run_phase_grouping(config)` | 1224 | Config | Tuple[str,str,List] | Phase 2 - 레코드 로딩 → 그룹 분석 → JSON+CSV 저장 |

### 5.5 CLI 함수 (섹션 6)

| 함수 | 줄 | 인자 | 반환 | 설명 |
|------|-----|------|------|------|
| `build_arg_parser()` | 1340 | - | ArgumentParser | CLI 인자 파서 구성 |
| `main()` | 1365 | - | None | 엔트리포인트: 인자 파싱 → Config 생성 → Phase 실행 |

---

## 6. 주요 상수 및 전역 변수

### 6.1 TYPE_ABBREV (줄 45~52)

배관 노드 타입을 2글자 약어로 매핑하는 딕셔너리. 경로 요약(path_summary) 생성에 사용.

| 타입 | 약어 | 타입 | 약어 | 타입 | 약어 |
|------|------|------|------|------|------|
| ELBOW | EB | TEE | TE | REDUCER | RD |
| UNION | UN | FLANGE | FL | ENDCAP | EC |
| CONNECTOR | CN | SOCKET | SK | BENDING | BD |
| CLAMP | CL | GLAND | GL | GASKET | GK |
| BELLOWS | BL | VALVE | VL | FILTER | FI |
| REGULATOR | RG | DAMPER | DA | DAMPER_DUCT | DD |
| POC | PO | TAKEOFF | TK | LATERAL PIPE | LP |
| LATERAL | LA | DUCT | DT | EQUIPMENT | EQ |
| SUB_EQUIPMENT | SE | BRANCH | BR | JUNCTION | JN |
| CROSS | CR | WYE | WY | ETC | ET |
| DIRECT_NODE | DN | | | | |

### 6.2 BRANCH_NODE_TYPES (줄 54)

분기 노드로 판정되는 타입 집합:
```python
{"TEE", "BRANCH", "JUNCTION", "CROSS", "WYE"}
```

### 6.3 LOGGER (줄 81)

```python
LOGGER = logging.getLogger("AnalyzeRoutingAi")
```
모듈 전체에서 사용하는 로거 인스턴스.

---

## 7. 핵심 알고리즘 상세

### 7.1 BFS 경로 탐색 (`_trace_paths_from_poc`)

```
입력: 시작 POC 노드, 소속 장비 ID
출력: 종단까지의 모든 경로 리스트

알고리즘:
1. 큐 초기화: (start_guid, record_idx=0, visited_edges=∅, depth=0)
2. 반복:
   a. 큐에서 하나 꺼냄
   b. 안전 검사: 큐 크기 > max_queue_size → 중단
   c. 안전 검사: depth > max_depth → 해당 노드 건너뜀
   d. _classify_terminal() → 종단이면 경로 복원 후 저장
   e. 안전 검사: paths_found >= max_paths_per_poc → 중단
   f. get_neighbors() → 이웃 탐색
   g. 각 이웃에 대해:
      - 사이클 방지 1: visited_edges에 있으면 건너뜀
      - 사이클 방지 2: _is_in_path()로 노드 중복 검사
      - 분기 노드면 branch_info 갱신
      - node_records에 추가, 큐에 추가
```

**사이클 방지 전략**: 엣지 기반 + 노드 기반 이중 체크
- `visited_edges` (frozenset): 이미 사용한 엣지 재사용 금지
- `_is_in_path()`: 부모 체인을 역추적하여 동일 노드 재방문 금지

### 7.2 종단 판정 (`_classify_terminal`)

종단 조건 (우선순위 순):

| 순서 | 조건 | 사유 라벨 |
|------|------|-----------|
| 1 | curr_guid == start_guid | 종단 아님 (시작점) |
| 2 | POC 소유 장비가 현재 장비가 아님 | "다른 장비 PoC 도달" |
| 3 | 타입이 DUCT/TAKEOFF이거나 이름에 포함 | "Duct / TakeOff 도달" |
| 4 | 타입이 LATERAL이거나 이름에 포함 | "Lateral Pipe 도달" |
| 5 | 이름에 NOZZLE 포함 | "Nozzle PoC 도달" |
| 6 | 타입 POC이고 이름이 END | "종단 PoC 도달" |
| 7 | 타입이 EQUIPMENT | "장비 노드 도달" |
| 8 | 이웃 노드가 없음 | "배관 끝단(막힘)" |

### 7.3 복합 유사도 계산

```
composite = Arrow(0.30) + Vector(0.30) + Range(0.20) + Length(0.20)
```

| 유사도 | 가중치 | 계산 방법 |
|--------|--------|-----------|
| Arrow Similarity | 0.30 | Levenshtein Distance 기반. R-H-D 코드열을 토큰 단위로 편집 거리 계산 후 1에서 차감 |
| Vector Similarity | 0.30 | 각 구간 벡터의 코사인 유사도를 0~1 정규화한 평균 × 길이 커버리지(min_len/max_len) |
| Range Similarity | 0.20 | BBox의 x,y,z range를 각각 비교한 평균 |
| Length Similarity | 0.20 | 총 길이 차이 비율로 계산 |

### 7.4 Union-Find 클러스터링

```
1. 동일 버킷(장비+유틸리티+사이즈) 내 모든 경로 쌍의 유사도 계산
2. 유사도 >= pattern_similarity_min(기본 0.70) → union 연산
3. 경로 압축(path compression) 적용으로 효율적 탐색
4. 클러스터별 필터링:
   - 경로 수 >= min_group_size
   - 시작 POC 간 XY 거리 <= start_poc_xy_max
   - (선택) 공통 Z 레벨 존재
```

### 7.5 영역 추정 (`detect_zones`)

```
1. 공통 Z 레벨 목록에서 trunk 후보 선택:
   - xy_spread <= trunk_max_xy_spread인 것 중 path_count 최대
   - 없으면 xy_spread 최소인 것 선택 (is_estimated=True)
2. trunk_z ± (tol_z_level × trunk_z_band_factor) 범위 정의
3. 각 경로의 노드를 순서대로 분류:
   - trunk 범위 첫 진입 전 → fan_in
   - trunk 범위 내 → trunk
   - trunk 범위 마지막 이후 → fan_out
4. 각 영역의 BoundingBox 계산
```

---

## 8. 입출력 데이터 형식

### 8.1 입력: 설계 JSON (data-v10/*.json)

```json
{
  "Nodes": [
    {"guid": "...", "type": "ELBOW", "name": "...", "x": 0.0, "y": 0.0, "z": 0.0,
     "connectionGuidList": ["edge_guid_1", "edge_guid_2"]}
  ],
  "Edges": [
    {"guid": "...", "type": "PIPE", "connectionGuidList": ["node_guid_1", "node_guid_2"]}
  ],
  "Equipment": [
    {"guid": "...", "name": "EQ-001", "process": "CLEAN", "maker": "...",
     "pocList": [{"guid": "poc_guid_1", "utility": "PCW", "size": "25A"}]}
  ]
}
```

### 8.2 Phase 1 출력: 경로 JSON (RoutingResults/*_Path.json)

```json
{
  "source_file": "data-v10/CLEAN_WTNHJ02_.json",
  "equipment_id": "eq-guid",
  "equipment_name": "WTNHJ02",
  "equipment_process": "CLEAN",
  "equipment_maker": "...",
  "created_at": "2026-04-05T...",
  "poc_paths": [
    {
      "start_poc_id": "poc-guid",
      "start_poc_info": { ... },
      "paths": [
        {
          "terminal_label": "다른 장비 PoC 도달",
          "branch_depth": 2,
          "branch_segments": [...],
          "path_summary": "PO->EB->TE->FL->PO",
          "end_node_id": "...",
          "end_node_type": "POC",
          "path_arrow": "H-H-R-H-H",
          "path_bbox": {"x_min":..., "x_max":..., ...},
          "path_range": {"x": 500.0, "y": 200.0, "z": 3000.0},
          "path_step_vectors": [{"x": 100, "y": 0, "z": 0}, ...],
          "path_step_lengths": [100.0, 250.5, ...],
          "path_total_length": 5230.5,
          "h_segments": [{"mean_z": 3500.0, "mid_xy": [100, 200], "node_count": 5}],
          "steps": [{"kind": "NODE", "data": {...}}, {"kind": "EDGE", "data": {...}}, ...]
        }
      ]
    }
  ]
}
```

### 8.3 Phase 2 출력: 그룹 분석 JSON (GroupPipeResults/group_pipe_results_*.json)

```json
[
  {
    "group_id": 1,
    "equipment_process": "CLEAN",
    "equipment_maker": "...",
    "equipment_name": "WTNHJ02",
    "equipment_id": "...",
    "utility": "PCW",
    "size": "25A",
    "path_count": 5,
    "unique_poc_count": 3,
    "avg_similarity": 0.8523,
    "max_start_xy_dist": 1200.5,
    "common_z_levels": [{"mean_z": 3500.0, "path_count": 4, "xy_spread": 800.0}],
    "zones": {
      "trunk": {"x_min":..., "mean_z": 3500.0, "path_count": 4, ...},
      "fan_in": {"z_min":..., "z_max":..., ...},
      "fan_out": {"z_min":..., "z_max":..., ...},
      "is_trunk_estimated": false
    },
    "paths": [
      {
        "poc_id": "...",
        "terminal_label": "...",
        "start_pos": [100, 200, 0],
        "end_pos": [500, 600, 3500],
        "displacement_vector": {"x": 400, "y": 400, "z": 3500},
        "path_summary": "PO->EB->TE->FL->PO",
        "path_arrow": "H-H-R-H-H",
        "path_range": {"x": 500, "y": 200, "z": 3000},
        "path_total_length": 5230.5,
        "path_step_lengths": [...],
        "path_step_vectors": [...]
      }
    ]
  }
]
```

### 8.4 Phase 2 출력: CSV 헤더

```
group_id, equipment_process, equipment_maker, equipment_name, equipment_id,
utility, size, path_count, unique_poc_count, avg_similarity, max_start_xy_dist,
trunk_z, trunk_xy_spread, trunk_path_count, is_estimated,
fan_in_z_min, fan_in_z_max, fan_out_z_min, fan_out_z_max, poc_ids
```

---

## 9. CLI 사용법

```bash
# Phase 1 + Phase 2 전체 실행
python AnalyzeRoutingAi_V2.py --phase all

# Phase 1만 실행 (경로 추출)
python AnalyzeRoutingAi_V2.py --phase routing --input ./data-v10

# Phase 2만 실행 (그룹 분석, Phase 1 결과 필요)
python AnalyzeRoutingAi_V2.py --phase grouping --routing_out ./RoutingResults

# 상세 옵션 예시
python AnalyzeRoutingAi_V2.py --phase all \
    --input ./data-v10 \
    --routing_out ./RoutingResults \
    --group_out ./GroupPipeResults \
    --log_level DEBUG \
    --max_paths_per_poc 10000 \
    --max_queue_size 200000 \
    --max_depth 1024 \
    --pattern_similarity_min 0.75 \
    --start_poc_xy_max 3000 \
    --tol_z_level 150 \
    --trunk_max_xy_spread 1000 \
    --min_group_size 3 \
    --require_common_z_levels
```

---

## 10. 의존성

| 패키지 | 용도 |
|--------|------|
| `argparse` | CLI 인자 파싱 |
| `csv` | CSV 파일 출력 |
| `glob` | JSON 파일 패턴 매칭 |
| `itertools` | 조합(combinations) 계산 |
| `json` | JSON 읽기/쓰기 |
| `logging` | 로그 출력 |
| `math` | 삼각함수, 제곱근 등 수학 연산 |
| `os` | 파일 경로 처리 |
| `re` | 정규식 (레거시 벡터 파싱) |
| `collections` | defaultdict, deque |
| `dataclasses` | AnalysisConfig 데이터클래스 |
| `datetime` | 타임스탬프 생성 |
| `typing` | 타입 힌트 |

> 모든 의존성이 Python 표준 라이브러리이므로 별도 설치가 필요 없습니다.
