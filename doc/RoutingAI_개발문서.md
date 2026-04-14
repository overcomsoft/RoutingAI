# RoutingAI 개발 문서

## 1. 프로젝트 개요

**RoutingAI**는 반도체 FAB 설비 배관의 3D 자동 라우팅 시스템입니다. BIM(Building Information Modeling) 설계 데이터(JSON)를 입력받아, 배관 경로를 추출·그룹핑하고, 장애물을 회피하는 신규 경로를 자동 설계합니다.

### 핵심 목적

| 단계 | 목적 | 핵심 모듈 |
|------|------|-----------|
| Phase 1 | BFS 기반 배관 경로 추출 및 특징량 산출 | `AnalyzeRoutingAi_V2.py` |
| Phase 2 | 유사도 기반 경로 그룹핑 (Union-Find) | `AnalyzeRoutingAi_V2.py` |
| Phase 3 | 템플릿 매칭 + 장애물 회피 자동 설계 | `AutoRoutingDesigner_V2.py` |

### 대상 설비 (8종, 7개 공정)

- CLEAN (세정), CMP (화학적기계적연마), CVD (화학기상증착)
- DIFF (확산), ETCH (식각), IMP (이온주입)
- METAL (금속증착), PHOTO (포토리소그래피)

---

## 2. 디렉토리 구조

```
RoutingAI/
├── src/                              # 소스 코드 (13개 Python 모듈)
│   ├── AnalyzeRoutingAi_V2.py        # Phase 1+2 메인 엔진 (1,556 LOC)
│   ├── AutoRoutingDesigner_V2.py     # Phase 3 자동 설계 엔진 (2,235 LOC)
│   ├── AnalyzeRoutingAi.py           # V1 레거시 분석기
│   ├── AutoRoutingDesigner.py        # V1 레거시 설계기
│   ├── AnalyzeRoutingPath.py         # BFS 경로 추출기
│   ├── AnalyzeBranching.py           # 분기점 분석
│   ├── analyze_group_pipes.py        # 파이프 그룹핑/클러스터링
│   ├── VisualizeRouting3D.py         # 3D 시각화 (Plotly)
│   ├── duct_poc_clustering.py        # 덕트 POC 분석
│   ├── import_obstacles_json.py      # BIM → PostgreSQL 임포트
│   ├── import_objects.py             # 객체 임포트
│   ├── import_duct_poc_cluster.py    # 덕트 클러스터링 임포트
│   └── import_pipe_group_analyze.py  # 파이프 그룹 임포트
│
├── data/
│   ├── input/                        # 입력 JSON (8개 설비 파일)
│   ├── RoutingResults/               # Phase 1 결과 (30개 파일)
│   ├── GroupPipeResults/             # Phase 2 결과 (7개 파일)
│   └── AutoRoutingResults/           # Phase 3 결과 (5개 파일)
│
├── visualization/
│   ├── VisualizeGroupPipe3D.html     # Three.js 그룹 시각화
│   └── VisualizeAutoRouting3D.html   # 장애물 포함 3D 뷰어
│
├── doc/                              # 문서
├── requirements.txt                  # Python 의존성
└── README.md                         # 프로젝트 개요
```

---

## 3. Phase 1 — BFS 경로 추출

### 3.1 처리 흐름

```
JSON 파일 로드
    ↓
Node/Edge/Equipment 인덱스 구축
    ↓
SpatialContext 로드 (장애물, 공간레벨)
    ↓
각 Equipment POC에서 BFS 탐색
    ↓
터미널(Duct/Lateral/타장비) 도달 경로 수집
    ↓
5차원 특징량 산출 (arrow, vector, range, length, obstacle_relations)
    ↓
RoutingResults/*.json 저장
```

### 3.2 핵심 클래스: `RoutingGraph`

```python
class RoutingGraph:
    """BFS 기반 그래프 탐색 엔진"""
    
    def load_from_json(path):      # JSON → 노드/엣지 인덱스 구축
    def find_routing_paths():       # 전체 POC에서 경로 추출
    def _trace_paths_from_poc():    # 단일 POC → 모든 터미널 BFS
    def get_neighbors(node_guid):   # 인접 노드 조회 (VIRTUAL 엣지 제외)
    def is_branch_node(node):       # 분기점 판별 (TEE/BRANCH/CROSS 등)
    def _classify_terminal(node):   # 터미널 분류 (7가지 기준)
```

### 3.3 BFS 안전 장치

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `max_depth` | 512 | 최대 탐색 깊이 |
| `max_queue_size` | 100,000 | 큐 크기 제한 |
| `max_paths_per_poc` | 5,000 | POC당 최대 경로 수 |
| `max_branch_count` | 8 | 최대 분기 수 |

### 3.4 터미널 분류 기준 (7가지)

1. 다른 장비 POC 도달
2. DUCT 노드 도달
3. LATERAL 노드 도달
4. NOZZLE 노드 도달
5. Equipment 타입 노드 도달
6. Dead-end (연결 없음)
7. 최대 깊이 초과

### 3.5 Arrow 코드 (방향 패턴)

배관 경로의 방향 변화를 문자열로 인코딩합니다.

| 코드 | 의미 | 각도 범위 |
|------|------|----------|
| `R` (Riser) | 수직 구간 | > 85° |
| `H` (Header) | 수평 구간 | < 5° |
| `D` (Diagonal) | 대각선 구간 | 5° ~ 85° |

**예시**: `H-H-R-H-D` = 수평→수평→수직→수평→대각선

### 3.6 5차원 특징량

| 차원 | 특징 | 설명 |
|------|------|------|
| 1 | `path_arrow` | 방향 패턴 문자열 (H-R-H 등) |
| 2 | `path_step_vectors` | 각 세그먼트 방향 벡터 [{x,y,z}, ...] |
| 3 | `path_bbox` | 경로 Bounding Box 범위 |
| 4 | `path_step_lengths` | 각 세그먼트 길이 배열 |
| 5 | `obstacle_relations` | 장애물 유형별 공간관계 특징 (18개) |

---

## 4. Phase 2 — 유사도 기반 그룹핑

### 4.1 처리 흐름

```
RoutingResults/*.json 로드
    ↓
(Equipment, Utility, Size) 기준 경로 분류
    ↓
5차원 복합 유사도 계산
    ↓
Union-Find 클러스터링 (임계값 ≥ 0.70)
    ↓
Zone 탐지 (Trunk / Fan-in / Fan-out)
    ↓
GroupPipeResults/*.json + *.csv 저장
```

### 4.2 핵심 클래스: `GroupAnalyzer`

```python
class GroupAnalyzer:
    """유사도 기반 Union-Find 클러스터링"""
    
    def find_groups():           # 그룹 탐색 (Union-Find)
    def _find_common_z_levels(): # 공통 수평 레벨 탐지
```

### 4.3 5차원 복합 유사도 공식

```
Similarity = 0.25 × arrow_sim
           + 0.25 × vector_sim
           + 0.15 × range_sim
           + 0.15 × length_sim
           + 0.20 × obstacle_relation_sim
```

| 유사도 | 산출 방법 |
|--------|----------|
| `arrow_sim` | Levenshtein 거리 정규화 |
| `vector_sim` | 방향 벡터 평균 코사인 유사도 |
| `range_sim` | BBox 범위의 L2 거리 |
| `length_sim` | 누적 경로 길이 유사도 |
| `obstacle_relation_sim` | 4개 장애물 유형 가중 평균 |

### 4.4 장애물 관계 유사도 세부 가중치

```
obstacle_relation_sim = 0.35 × column_sim    # 구조기둥
                      + 0.30 × beam_sim      # H-빔
                      + 0.20 × grating_sim   # 그레이팅
                      + 0.15 × post_sim      # 포스트
```

### 4.5 Zone 탐지

그룹핑된 경로에서 3개 영역을 자동 탐지합니다.

| Zone | 설명 | 특징 |
|------|------|------|
| **Fan-in** | 장비 POC 근처, 개별 경로가 모이는 구간 | 높은 Z 분산 |
| **Trunk** | 메인 배관 구간, 공통 수평 경로 | 낮은 XY 분산 (≤1500mm) |
| **Fan-out** | 터미널(덕트 등) 근처, 경로가 분기되는 구간 | 높은 Z 분산 |

---

## 5. Phase 3 — 장애물 회피 자동 설계

### 5.1 처리 흐름

```
JSON + GroupPipeResults 로드
    ↓
SpatialContext 구축 (장애물, 장비, 공간레벨)
    ↓
신규 경로 요청 (start, dest, equipment)
    ↓
8차원 특징량 추정
    ↓
기존 그룹 템플릿과 유사도 매칭
    ↓
3단계 경로 생성 (Fan-in → Trunk → Fan-out)
    ↓
장애물 회피 적용 (Slab Method)
    ↓
경로 품질 검증
    ↓
AutoRoutingResults/*.json 저장
```

### 5.2 장애물 유형 분류 (6종)

```python
class ObstacleCategory(Enum):
    STRUCTURAL_COLUMN = "COLUMN_STRUCTURE"   # 구조기둥
    POST              = "POST"               # 포스트
    H_BEAM            = "H_BEAM"             # H-빔
    STRUCTURAL_BEAM   = "STRUCTURAL_BEAM"    # 구조보
    GRATING           = "GRATING"            # 그레이팅
    CEILING           = "CEILING"            # 천장
```

### 5.3 장애물 공간관계 특징량 (18개)

#### 구조기둥 (Column) — 5개 특징

| 특징 | 설명 |
|------|------|
| `col_count_nearby` | 경로 근처 기둥 수 |
| `col_min_distance` | 최소 이격 거리 (mm) |
| `col_avg_distance` | 평균 이격 거리 (mm) |
| `col_crossings` | 기둥 횡단 횟수 |
| `col_relative_pattern` | 기둥 대비 상대 위치 패턴 (L/R/B) |

#### 포스트 (Post) — 3개 특징

| 특징 | 설명 |
|------|------|
| `post_count_nearby` | 근처 포스트 수 |
| `post_density` | 포스트 밀도 (개/m) |
| `post_grid_alignment` | 포스트 그리드 정렬도 (0~1) |

#### H-빔/구조보 (Beam) — 3개 특징

| 특징 | 설명 |
|------|------|
| `beam_count_crossing` | 빔 횡단 횟수 |
| `beam_min_clearance` | 빔과 최소 여유 간격 (mm) |
| `beam_parallel_ratio` | 빔과 평행 구간 비율 (0~1) |

#### 그레이팅 (Grating) — 3개 특징

| 특징 | 설명 |
|------|------|
| `grating_coverage` | 경로 하부 그레이팅 커버 비율 (0~1) |
| `grating_count_below` | 경로 아래 그레이팅 수 |
| `grating_gap_count` | 그레이팅 간격(갭) 수 |

### 5.4 8차원 강화 특징량

| 차원 | 특징 | 유사도 가중치 |
|------|------|-------------|
| 1 | Arrow (방향 패턴) | 0.12 |
| 2 | Vector (방향 벡터 시퀀스) | 0.12 |
| 3 | Range (BBox 범위) | 0.08 |
| 4 | Length (구간 길이) | 0.08 |
| 5 | Equipment-relative (장비 상대 위치) | 0.15 |
| 6 | Terminal (터미널 유형/거리/레벨) | 0.15 |
| 7 | **Obstacle (장애물 관계)** | **0.20** |
| 8 | Level (공간 레벨) | 0.10 |

### 5.5 8차원 유사도 공식

```
Enhanced_Sim = 0.12 × arrow_sim
             + 0.12 × vector_sim
             + 0.08 × range_sim
             + 0.08 × length_sim
             + 0.15 × equip_relative_sim
             + 0.15 × terminal_sim
             + 0.20 × obstacle_sim
             + 0.10 × level_sim
```

장애물 유사도 내부 가중치:
```
obstacle_sim = 0.35 × column_sim
             + 0.30 × beam_sim
             + 0.20 × grating_sim
             + 0.15 × post_sim
```

### 5.6 경로 생성 알고리즘

`ObstacleAwarePathBuilder` 클래스가 3단계로 경로를 생성합니다.

```
[Start POC]
    │
    ├─── Fan-in 구간 ──→ 수직/대각선 이동 (장비 → Trunk 레벨)
    │
    ├─── Trunk 구간 ──→ 수평 이동 (메인 배관 따라)
    │
    └─── Fan-out 구간 ──→ 수직/대각선 이동 (Trunk → 터미널)
    │
[Destination Terminal]
```

### 5.7 장애물 회피 (Slab Method)

**충돌 감지**: 3D AABB(Axis-Aligned Bounding Box) 선분-박스 교차 검사

```python
def _segment_intersects_box(p1, p2, box_min, box_max):
    """Slab Method - 선분이 AABB와 교차하는지 판정"""
    # 각 축(X,Y,Z)에 대해 진입/탈출 파라미터 계산
    # tmin, tmax 교차 범위로 충돌 판정
```

**회피 전략**:
- 구조기둥(STRUCTURAL_COLUMN)에 대해서만 적용
- 최대 5회 반복 우회
- 수직 방향 우회 세그먼트 삽입 (200mm 여유 간격)

```
충돌 감지
    ↓
기둥 중심 기준 좌/우 판별
    ↓
수직 방향 우회점 계산 (기둥 경계 + 200mm)
    ↓
우회 세그먼트 삽입 (원래 경로 대체)
    ↓
재검증 (최대 5회)
```

### 5.8 경로 품질 검증

`PathValidatorV2`가 5개 항목을 검증합니다.

| 검증 항목 | 설명 | 합격 기준 |
|----------|------|----------|
| `continuity` | 경로 연속성 (끊김 없음) | 1.0 |
| `length_ratio` | 직선 대비 경로 길이 비율 | 합리적 범위 |
| `direction_pattern` | 방향 패턴 일관성 | 패턴 매칭 |
| `obstacle_collision` | 장애물 충돌 없음 | 충돌 0 |
| `zone_compliance` | Zone 구조 준수 | 준수 |

**종합 품질 점수 임계값**: ≥ 0.40

---

## 6. 핵심 데이터 구조

### 6.1 입력 JSON 포맷

```json
{
  "FileInfo": {
    "Project": "ELEC_DB2014_dinno3",
    "SpaceInfo": [
      {
        "levelName": "CSF",          // CSF, A, F, CR
        "boundary": {
          "min": {"x": 0, "y": 0, "z": 0},
          "max": {"x": 100, "y": 100, "z": 5000}
        }
      }
    ],
    "GroupBoundaryBox": {...}
  },
  "Equipment": {
    "guid": "equip-guid-001",
    "name": "kscta01",
    "boundaryBox": {
      "min": {"x": 5000, "y": 27000, "z": 15000},
      "max": {"x": 6000, "y": 29000, "z": 16000}
    },
    "pocList": [
      {
        "guid": "poc-guid-001",
        "pocPosition": [5500, 28200, 15495],
        "utility": "AKWW",
        "endPocs": [
          {
            "endType": "DUCT",
            "endPocPosition": [5300, 27000, 15495]
          }
        ]
      }
    ]
  },
  "Nodes": [
    {
      "guid": "node-guid-001",
      "id": "POC0001",
      "type": "POC",               // POC, ELBOW, TEE, DUCT, LATERAL ...
      "position": [5500, 28200, 15495],
      "connectionGuidList": ["edge-guid-001", "edge-guid-002"]
    }
  ],
  "Edges": [
    {
      "guid": "edge-guid-001",
      "type": "PIPE",
      "connectionGuidList": ["node-guid-001", "node-guid-002"],
      "utility": "AKWW",
      "size": "20A"
    }
  ],
  "Obstacles": [
    {
      "obstacleId": "COL001",
      "ddworksType": "COLUMN_STRUCTURE",
      "name": "Column-K01",
      "boundary": {
        "min": {"x": 5200, "y": 27500, "z": 14000},
        "max": {"x": 5600, "y": 27900, "z": 17000}
      }
    }
  ]
}
```

### 6.2 Phase 1 출력 (RoutingResults)

```json
{
  "equipment_name": "kscta01",
  "equipment_info": {
    "guid": "...",
    "bbox": {...},
    "poc_count": 12
  },
  "poc_paths": [
    {
      "start_poc_id": "POC0001",
      "paths": [
        {
          "end_node": {"id": "DUCT001", "type": "DUCT", "position": [...]},
          "label": "Duct 도달",
          "path": [
            {"kind": "NODE", "data": {"guid": "...", "type": "POC", "position": [...]}},
            {"kind": "EDGE", "data": {"guid": "...", "type": "PIPE", "size": "20A"}},
            {"kind": "NODE", "data": {...}}
          ],
          "path_arrow": "H-H-R-D",
          "path_bbox": {
            "x_min": 5200, "x_max": 6100,
            "y_min": 27000, "y_max": 28200,
            "z_min": 14700, "z_max": 15495
          },
          "path_step_vectors": [
            {"x": 0.0, "y": -1.0, "z": 0.0},
            {"x": 1.0, "y": 0.0, "z": 0.0}
          ],
          "path_step_lengths": [100.5, 250.2, 800.0],
          "obstacle_relations": {
            "col_count_nearby": 1,
            "col_min_distance": 1795.0,
            "col_avg_distance": 1795.0,
            "col_crossings": 0,
            "col_relative_pattern": "L",
            "post_count_nearby": 3,
            "post_density": 0.42,
            "post_grid_alignment": 0.85,
            "beam_count_crossing": 1,
            "beam_min_clearance": 320.0,
            "beam_parallel_ratio": 0.6,
            "grating_coverage": 0.75,
            "grating_count_below": 2,
            "grating_gap_count": 1
          }
        }
      ]
    }
  ]
}
```

### 6.3 Phase 2 출력 (GroupPipeResults)

```json
{
  "group_id": 1,
  "equipment_process": "CMP",
  "utility": "AKWW",
  "size": "20A",
  "path_count": 36,
  "avg_similarity": 0.749,
  "trunk_z": 14699.1,
  "trunk_xy_spread": 4287.2,
  "fan_in_z_min": 15495.0,
  "fan_in_z_max": 15495.0,
  "fan_out_z_min": 15495.0,
  "fan_out_z_max": 15495.0,
  "paths": [...],
  "zones": {
    "trunk": {"z_level": 14699.1, "xy_spread": 4287.2},
    "fan_in": {"z_min": 15495.0, "z_max": 15495.0},
    "fan_out": {"z_min": 15495.0, "z_max": 15495.0}
  }
}
```

### 6.4 Phase 3 출력 (AutoRoutingResults)

```json
{
  "group_id": "AUTO_001",
  "equipment": "kscta01",
  "utility": "AKWW",
  "start": [5500, 28200, 15495],
  "dest": [5300, 27000, 15495],
  "generated_path": {
    "path_vectors": [{"x": 0, "y": -1, "z": 0}, ...],
    "quality_score": 0.85,
    "validation": {
      "continuity": 1.0,
      "length_ratio": 0.92,
      "direction_pattern": 0.8,
      "obstacle_collision": 0.9,
      "zone_compliance": 1.0
    }
  }
}
```

---

## 7. 보조 모듈 상세

### 7.1 AnalyzeBranching.py — 분기점 분석

BFS로 배관 네트워크의 분기 구조를 분석합니다.

```python
class BranchGraph:
    def load_from_json():     # 그래프 구축
    def is_terminal_node():   # 터미널 판별
    def trace_to_terminal():  # 터미널까지 추적
    def analyze_branches():   # 분기점별 메타데이터 수집
```

**출력**: 분기 유형 요약, 터미널 매핑 (JSON/CSV)

### 7.2 analyze_group_pipes.py — 파이프 그룹핑

3단계 클러스터링으로 유사한 파이프를 그룹화합니다.

```
1단계: 허용오차 기반 초기 그룹핑 (Z축)
    ↓
2단계: 방향 벡터 검증 (각도 < 3°, 코사인 유사도)
    ↓
3단계: BFS 근접 클러스터링 (최대 간격 300mm)
```

**핵심 함수**:
- `group_by_tolerance()`: 1D 허용오차 그룹핑
- `group_by_direction()`: 방향 유사도 검증
- `cluster_vertical_pipes()`: 수직 파이프 클러스터링
- `cluster_horizontal_pipes()`: 수평 파이프 클러스터링

### 7.3 duct_poc_clustering.py — 덕트 POC 분석

덕트 연결점(POC) 간 거리 및 간격 통계를 산출합니다.

### 7.4 VisualizeRouting3D.py — 3D 시각화

Plotly 기반 인터랙티브 3D 시각화를 생성합니다.

- **시작 POC**: 빨간 다이아몬드
- **경로**: 컬러 라인 (10색 팔레트)
- **터미널**: 검정 사각형
- **출력**: 설비별 HTML 파일

### 7.5 visualization/ — Three.js 3D 뷰어

| 파일 | 기능 |
|------|------|
| `VisualizeGroupPipe3D.html` | 그룹별 파이프 3D 시각화 |
| `VisualizeAutoRouting3D.html` | 장애물 포함 자동 라우팅 결과 3D 뷰어 |

---

## 8. 데이터베이스 연동 (PostgreSQL + PostGIS)

### 8.1 테이블 스키마

| 테이블 | 설명 | 기하 타입 |
|--------|------|----------|
| `TB_BIM_OBSTACLES` | 장애물 정보 | box3d |
| `TB_BIM_SPACE_INFO` | 공간 레벨 (CSF/A/F/CR) | box3d |
| `TB_BIM_EQUIPMENT` | 장비 + POC | box3d + MultiPointZ |
| `TB_DUCT_LATERAL` | 덕트/래터럴 | box3d |

### 8.2 임포트 모듈

| 모듈 | 기능 |
|------|------|
| `import_obstacles_json.py` | BIM JSON → 장애물/공간/장비 임포트 |
| `import_objects.py` | BIM 객체 임포트 |
| `import_duct_poc_cluster.py` | 덕트 클러스터 임포트 |
| `import_pipe_group_analyze.py` | 파이프 그룹 분석 결과 임포트 |

---

## 9. 설정 및 상수

### 9.1 Phase 1+2 설정 (AnalysisConfig)

```python
@dataclass
class AnalysisConfig:
    max_branch_count: int = 8            # 최대 분기 수
    direction_angle_tolerance: float = 5.0  # 방향 판별 각도 (°)
    max_paths_per_poc: int = 5000        # POC당 최대 경로
    pattern_similarity_min: float = 0.70  # 그룹핑 유사도 임계값
    start_poc_xy_max: float = 5000       # POC 탐색 범위 (mm)
    trunk_max_xy_spread: float = 1500    # Trunk XY 분산 한계 (mm)
    min_group_size: int = 2              # 최소 그룹 크기
```

### 9.2 Phase 3 설정 (DesignConfigV2)

```python
obstacle_clearance = 200       # 장애물 여유 간격 (mm)
direction_angle_tolerance = 5.0  # 방향 판별 각도 (°)
fitting_length_threshold = 150   # 피팅 길이 임계값 (mm)
quality_threshold = 0.40         # 품질 합격 점수
max_detour_iterations = 5        # 최대 우회 반복 횟수
```

### 9.3 공간 레벨

| 레벨 | 의미 |
|------|------|
| CSF | Clean Sub-Fab (클린 서브팹) |
| A | A레벨 |
| F | F레벨 |
| CR | Clean Room (클린룸) |

---

## 10. 실행 방법

### Phase 1+2: 전체 파이프라인

```bash
python src/AnalyzeRoutingAi_V2.py --phase all --input data/input
```

### Phase 3: 단일 경로 자동 설계

```bash
python src/AutoRoutingDesigner_V2.py \
  --json_input data/input/CMP_KSCTA01_*.json \
  --group_results data/GroupPipeResults/group_pipe_results_*.json \
  --equipment kscta01 \
  --utility AKWW \
  --size 20A \
  --start 5500,28200,15495 \
  --dest 5300,27000,15495
```

### 3D 시각화

```bash
python src/VisualizeRouting3D.py ./RoutingResults
```

### 데이터베이스 임포트

```bash
python src/import_obstacles_json.py ./data/input \
  --dbname AUTOROUTINGV7 \
  --user postgres \
  --password dinno \
  --host localhost \
  -p 5432
```

---

## 11. 테스트 데이터 현황

### 입력 데이터 (8개 설비)

| 파일 | 공정 | 설비명 |
|------|------|--------|
| `CLEAN_WTNHJ02_*.json` | 세정 | WTNHJ02 |
| `CMP_KSCTA01_*.json` | CMP | KSCTA01 |
| `CVD_TNMHJ02_*.json` | CVD | TNMHJ02 |
| `DIFF_DANHJ14_*.json` | 확산 | DANHJ14 |
| `ETCH_ELOHJ07_*.json` | 식각 | ELOHJ07 |
| `IMP_IVHHJ01_*.json` | 이온주입 | IVHHJ01 |
| `METAL_SLWHJ02_*.json` | 금속증착 | SLWHJ02 |
| `PHOTO_PSTWA03_*.json` | 포토 | PSTWA03 |

### 장애물 통계 (CMP_KSCTA01 기준)

| 유형 | 수량 |
|------|------|
| 구조기둥 (COLUMN_STRUCTURE) | 18 |
| 포스트 (POST) | 192 |
| H-빔 (H_BEAM) | 32 |
| 그레이팅 (GRATING) | 203 |
| **합계** | **445** |

### 결과 데이터

| Phase | 파일 수 | 비고 |
|-------|---------|------|
| Phase 1 (RoutingResults) | 30개 | 설비당 평균 3.75개 |
| Phase 2 (GroupPipeResults) | 7개 | 평균 유사도 0.749 |
| Phase 3 (AutoRoutingResults) | 5개 | 자동 설계 결과 |

---

## 12. 의존성 (requirements.txt)

| 패키지 | 용도 |
|--------|------|
| pandas | 데이터 처리 |
| geopandas, shapely | 공간 연산 |
| psycopg2 | PostgreSQL 연결 |
| openpyxl | Excel 입출력 |
| python-docx | Word 문서 생성 |
| fpdf | PDF 생성 |
| pyinstaller | EXE 패키징 |
| plotly | 3D 시각화 |

---

## 13. 핵심 기술 혁신

1. **장애물 인식 라우팅**: 유형별 18개 공간관계 특징량 추출 (구조기둥 5개, 포스트 3개, H-빔 3개, 그레이팅 3개 + 공통 4개)
2. **3-Phase 파이프라인**: 추출 → 그룹핑 → 생성의 분리된 처리 단계
3. **8차원 유사도 매칭**: 기하학적(arrow, vector, range, length) + 공간적(equipment, terminal, level) + 관계적(obstacles) 통합
4. **BFS 안전 설계**: 5중 보호 메커니즘 (큐 크기, 깊이, 경로 수, 분기 수, visited 집합)
5. **Slab Method 충돌 감지**: 3D AABB 선분-박스 교차 검사 기반 효율적 충돌 판정
6. **자동 우회 경로**: 구조기둥 대상 최대 5회 반복 우회 생성
7. **다층 공간 지원**: CSF/A/F/CR 레벨 인식 및 레벨간 이동 고려
8. **Union-Find 클러스터링**: O(α(n)) 복잡도의 효율적 경로 그룹핑

---

## 14. V1 → V2 주요 변경사항

| 항목 | V1 | V2 |
|------|----|----|
| 특징량 차원 | 4D (arrow, vector, range, length) | 5D (+obstacle_relations) / 8D (Phase 3) |
| 장애물 고려 | 없음 | 유형별 18개 특징량 |
| 유사도 공식 | 4차원 균등 가중 | 5/8차원 차등 가중 |
| 경로 생성 | 단순 보간 | 3단계 (Fan-in→Trunk→Fan-out) + 장애물 회피 |
| 검증 | 기본 연속성 | 5항목 종합 검증 |
| 공간 레벨 | 미지원 | CSF/A/F/CR 인식 |
