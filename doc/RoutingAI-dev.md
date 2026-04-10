# RoutingAI 개발보고서 - 장애물 유형별 공간관계 특징량 확장

> 작성일: 2026-04-10
> 프로젝트: KGraphGen03 / Analyzer / RoutingAI 자동 배관 경로 설계 시스템

---

## 1. 개요

### 1.1 배경

기존 RoutingAI V2 시스템은 배관 경로의 유사도 분석 시 장애물 관련 특징량으로 **2가지만** 사용하고 있었다:

- `obstacle_density` : 경로 주변 장애물의 스칼라 밀도값 (0~1)
- `column_crossings` : 경로가 기둥을 교차하는 횟수

이 단순한 특징으로는 배관 경로가 다양한 장애물 유형(기둥, H-Beam, 그레이팅, 포스트)과 맺는 **공간적 관계의 패턴**을 포착할 수 없으며, 유사한 장애물 환경을 가진 경로들을 정확히 클러스터링하는 데 한계가 있었다.

### 1.2 목표

장애물 유형별(구조기둥/포스트/H-Beam/그레이팅) 공간관계를 **18개 세분화된 특징량**으로 추출하여 유사도 클러스터링의 정밀도를 향상시킨다.

### 1.3 실제 장애물 데이터 분포

테스트 데이터(CMP_KSCTA01) 기준 **456개** 장애물:

| ddworksType | ostType | 수량 | 실제 객체 | 배관 영향도 |
|---|---|---|---|---|
| COLUMN_STRUCTURE | OST_StructuralColumns | 18 | 구조 기둥 (1300x1400mm) | **최고** - 절대 회피 |
| COLUMN_ARCHITECTURE | OST_Columns | 192 | Access Floor 포스트 | **중간** - 우회 권장 |
| BEAM_ARCHITECTURE | OST_BeamStartSegment | 32 | H-Beam (CSF/FSF) | **높음** - 수직 간섭 |
| BEAM_STRUCTURE | OST_StructuralFraming | 10 | 구조 보 (3600x3600) | **높음** - 수직 간섭 |
| FLOOR_ARCHITECTURE | OST_Floors | 203 | Grating (그레이팅) | **중간** - 통과 제약 |
| CEILING_ARCHITECTURE | OST_Ceilings | 1 | 천장 | **낮음** |

---

## 2. 시스템 아키텍처

### 2.1 전체 파이프라인

```
┌─────────────────────────────────────────────────────────────────┐
│  원본 설계 JSON                                                  │
│  (Nodes, Edges, Equipment, Obstacles, SpaceInfo)                │
└──────────┬──────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────┐     ┌──────────────────────────────────┐
│  Phase 1: BFS 경로추출 │     │  SpatialContext                   │
│  (AnalyzeRoutingAi_V2)│ ──▶ │  장애물 유형별 분류 로딩             │
└──────────┬───────────┘     │  structural_columns / posts /     │
           │                  │  h_beams / structural_beams /     │
           │                  │  gratings                         │
           │                  └──────────┬───────────────────────┘
           │                             │
           ▼                             ▼
┌──────────────────────┐     ┌──────────────────────────────────┐
│  경로별 기본 특징량     │     │  ObstacleRelationExtractor        │
│  arrow / vector /     │     │  유형별 공간관계 추출                │
│  range / length       │     │  _compute_column_relations()      │
└──────────┬───────────┘     │  _compute_post_relations()        │
           │                  │  _compute_beam_relations()        │
           │                  │  _compute_grating_relations()     │
           │                  └──────────┬───────────────────────┘
           │                             │
           ▼                             ▼
┌──────────────────────────────────────────────────────────────────┐
│  Phase 2: 유사도 클러스터링                                        │
│  compute_composite_similarity() / SpatialSimilarity.compute()    │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  기존 특징량 (arrow + vector + range + length)  ─ 기하학적   │  │
│  │  장비상대좌표 + 종단점 + 레벨               ─ 공간 컨텍스트    │  │
│  │  ★ 장애물 유형별 공간관계                   ─ 장애물 관계     │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────┬───────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────┐
│  Phase 3: 자동 경로설계 │
│  (AutoRoutingDesigner  │
│   _V2)                 │
└────────────────────────┘
```

### 2.2 수정 파일 목록

| 파일 | 주요 변경 | 코드 규모 |
|------|----------|----------|
| `AutoRoutingDesigner_V2.py` | ObstacleCategory, ObstacleRelationFeatures, ObstacleRelationExtractor, SpatialSimilarity 확장 | +약 350줄 |
| `AnalyzeRoutingAi_V2.py` | SpatialContext 연동, obstacle_relations 저장, 5차원 유사도 | +약 80줄 |

---

## 3. 구현 상세

### 3.1 장애물 유형 분류 체계

`ObstacleCategory` enum과 `_classify_obstacle()` 매핑 함수를 도입하여 BIM 데이터의 `ddworksType`을 의미 기반 카테고리로 변환한다.

```python
class ObstacleCategory(str, Enum):
    STRUCTURAL_COLUMN = "STRUCTURAL_COLUMN"   # COLUMN_STRUCTURE
    POST              = "POST"                # COLUMN_ARCHITECTURE
    H_BEAM            = "H_BEAM"              # BEAM_ARCHITECTURE
    STRUCTURAL_BEAM   = "STRUCTURAL_BEAM"     # BEAM_STRUCTURE
    GRATING           = "GRATING"             # FLOOR_ARCHITECTURE
    CEILING           = "CEILING"             # CEILING_ARCHITECTURE
    OTHER             = "OTHER"
```

`ObstacleInfo` 데이터클래스에 `category` 필드가 추가되며, `SpatialContext`에 유형별 세분화 리스트가 추가된다:

```python
# SpatialContext 내부
self.structural_columns: List[ObstacleInfo]  # 구조 기둥
self.posts:              List[ObstacleInfo]  # Access Floor 포스트
self.h_beams:            List[ObstacleInfo]  # H-Beam
self.structural_beams:   List[ObstacleInfo]  # 구조 보
self.gratings:           List[ObstacleInfo]  # 그레이팅
```

기존 `self.columns` / `self.beams` 리스트는 하위 호환을 위해 유지된다.

---

### 3.2 장애물 유형별 공간관계 특징량 (18개 항목)

`ObstacleRelationFeatures` 데이터클래스로 정의:

#### 3.2.1 구조 기둥 (STRUCTURAL_COLUMN) - 5개 항목

| 필드명 | 타입 | 설명 |
|--------|------|------|
| `col_count_nearby` | int | 경로 근방 구조 기둥 수 |
| `col_min_distance` | float | 최근접 기둥까지 거리 (mm) |
| `col_avg_distance` | float | 근방 기둥 평균 거리 (mm) |
| `col_crossings` | int | 경로-기둥 교차 수 (Slab method) |
| `col_relative_pattern` | str | 기둥의 좌/우 배치 패턴 (L/R/B 문자열) |

**LR 패턴 알고리즘**: 각 세그먼트의 진행 방향에 대한 법선 벡터를 계산하고, 기둥 중심이 법선 방향의 좌측(L), 우측(R), 또는 양쪽(B=Between)에 위치하는지를 판별한다.

```
경로:  ──→──→──→
기둥:    ●        ●   ●
패턴:    L        R   B    → "LRB"
```

#### 3.2.2 포스트 (POST) - 3개 항목

| 필드명 | 타입 | 설명 |
|--------|------|------|
| `post_count_nearby` | int | 경로 근방 포스트 수 |
| `post_density` | float | 포스트 밀도 (포스트 수 / BBox XY 면적, 0~1) |
| `post_grid_alignment` | float | 포스트 그리드와 경로 정렬도 (0~1) |

**그리드 정렬도 알고리즘**: 두 가지 지표의 가중 평균으로 계산한다.
1. **그리드 규칙성** (50%): 포스트 간 X/Y 간격의 변동계수(CV)로 규칙적 배치 판별
2. **경로 정렬도** (50%): 경로 세그먼트가 X축/Y축에 얼마나 정렬되는지 (직교 비율)

#### 3.2.3 H-Beam (H_BEAM + STRUCTURAL_BEAM) - 3개 항목

| 필드명 | 타입 | 설명 |
|--------|------|------|
| `beam_count_crossing` | int | 빔 교차 수 (Slab method, margin 100mm) |
| `beam_min_clearance` | float | 최소 수직 간격 - 빔 하단과 경로 Z의 차이 (mm) |
| `beam_parallel_ratio` | float | 경로와 평행한 빔 비율 (cos > 0.7이면 평행) |

**수직 간격(clearance)**: 배관이 H-Beam 아래를 지나갈 때의 여유 공간으로, 배관 설계 시 높이 결정에 핵심적인 제약 조건이다.

#### 3.2.4 그레이팅 (GRATING) - 3개 항목

| 필드명 | 타입 | 설명 |
|--------|------|------|
| `grating_coverage` | float | 경로 하부 그레이팅 커버리지 (0~1) |
| `grating_count_below` | int | 경로 아래 그레이팅 수 |
| `grating_gap_count` | int | 그레이팅 빈틈(개구부) 전환 수 |

**커버리지 알고리즘**: 수평 세그먼트(Z변화 < 100mm)에 대해서만 계산하며, 세그먼트 중심점이 그레이팅의 XY 범위 내에 있고 그레이팅 상단이 경로 아래(+500mm 여유)인 경우를 "커버"로 판정한다.

---

### 3.3 ObstacleRelationExtractor 클래스

경로 좌표 리스트를 입력받아 4개 유형의 공간관계를 한번에 추출하는 전용 클래스이다.

```python
class ObstacleRelationExtractor:
    def __init__(self, spatial: SpatialContext):
        self.spatial = spatial

    def extract(self, positions, radius) -> ObstacleRelationFeatures:
        feat = ObstacleRelationFeatures()
        self._compute_column_relations(positions, radius, feat)
        self._compute_post_relations(positions, radius, feat)
        self._compute_beam_relations(positions, radius, feat)
        self._compute_grating_relations(positions, radius, feat)
        return feat
```

**호출 위치**:
- `AutoRoutingDesigner_V2.py` → `EnhancedFeatureExtractor.extract()` 내부에서 호출
- `AnalyzeRoutingAi_V2.py` → `_compute_obstacle_relations_for_path()` 헬퍼를 통해 Phase 1 결과에 포함

---

### 3.4 유사도 계산 확장

#### 3.4.1 AutoRoutingDesigner_V2 (8차원 유사도)

`SpatialSimilarity._obstacle_sim()` 함수를 유형별 4개 세부 유사도로 세분화:

```
_obstacle_sim()
  ├── _column_relation_sim()   × 0.35  ← 구조기둥 (교차수 0.3 + LR패턴 0.3 + 근접수 0.2 + 거리 0.2)
  ├── _beam_relation_sim()     × 0.30  ← H-Beam   (교차수 0.4 + clearance 0.3 + 평행비 0.3)
  ├── _grating_relation_sim()  × 0.20  ← 그레이팅  (커버리지 0.4 + 하부수 0.3 + 개구부 0.3)
  └── _post_relation_sim()     × 0.15  ← 포스트   (그리드정렬 0.4 + 밀도 0.3 + 근접수 0.3)
```

전체 유사도 가중치 변경:

```
변경 전:                          변경 후:
───────────────────────         ───────────────────────
arrow:     0.15                 arrow:     0.12
vector:    0.15                 vector:    0.12
range:     0.10                 range:     0.08
length:    0.10                 length:    0.08
equip:     0.15                 equip:     0.15
terminal:  0.15                 terminal:  0.15
obstacle:  0.10  ← 단순 밀도    obstacle:  0.20  ← 유형별 세분화
level:     0.10                 level:     0.10
───────────────────────         ───────────────────────
합계:      1.00                 합계:      1.00
```

#### 3.4.2 AnalyzeRoutingAi_V2 (5차원 유사도)

`compute_composite_similarity()` 함수에 `_obstacle_relation_similarity()` 추가:

```
변경 전:                                  변경 후:
────────────────────────────           ────────────────────────────
arrow:     0.30                        arrow:     0.25
vector:    0.30                        vector:    0.25
range:     0.20                        range:     0.15
length:    0.20                        length:    0.15
                                       obstacle:  0.20  ← 신규
────────────────────────────           ────────────────────────────
합계:      1.00                        합계:      1.00
```

Phase 1에서 SpatialContext를 함께 로드하여 경로 결과 JSON에 `obstacle_relations` 필드를 포함시키고, Phase 2의 그룹핑에서 이를 활용한다. 장애물 데이터가 없는 기존 결과 파일은 자동으로 중립(1.0) 처리되어 **하위 호환성이 유지**된다.

---

## 4. 검증 결과

### 4.1 유형별 분류 정확성

456개 장애물이 5개 유형으로 정확하게 분류됨을 확인:

```
Total obstacles: 456
  structural_columns: 18   (COLUMN_STRUCTURE → STRUCTURAL_COLUMN)
  posts:             192   (COLUMN_ARCHITECTURE → POST)
  h_beams:            32   (BEAM_ARCHITECTURE → H_BEAM)
  structural_beams:   10   (BEAM_STRUCTURE → STRUCTURAL_BEAM)
  gratings:          203   (FLOOR_ARCHITECTURE → GRATING)
  ceiling:             1   (CEILING_ARCHITECTURE → CEILING)
```

### 4.2 특징량 추출 검증

테스트 경로 `(5500,28200,15495) → (5500,27600,15495) → (5500,27000,15495) → (5300,27000,15495)`:

```
Column:   nearby=1, min_dist=1795mm, crossings=0, pattern=LLR
Post:     nearby=21, density=1.0, grid_align=1.0
Beam:     crossings=0, min_clearance=138mm, parallel=1.0
Grating:  coverage=0.0, below=22, gaps=0
```

### 4.3 유사도 계산 검증

| 비교 | 점수 | 판정 |
|------|------|------|
| 동일 경로 자기 유사도 | **1.0000** | 정확 (기대값 1.0) |
| 다른 영역 경로 교차 유사도 | **0.5246** | 적절한 변별력 |

유형별 세부 유사도 (교차 비교):

| 유형 | 유사도 | 해석 |
|------|--------|------|
| 포스트 | 0.8615 | 두 경로 모두 포스트 밀집 영역 → 높은 유사도 |
| H-Beam | 0.6000 | 빔 배치 차이 → 중간 유사도 |
| 그레이팅 | 0.4467 | 커버리지 차이 큼 → 낮은 유사도 |
| 구조기둥 | 0.3600 | 기둥 위치/패턴 차이 → 낮은 유사도 |

---

## 5. 클래스/함수 참조표

### 5.1 AutoRoutingDesigner_V2.py

| 클래스/함수 | 위치 | 역할 |
|---|---|---|
| `ObstacleCategory` | line 74 | 장애물 유형 enum (7종) |
| `_classify_obstacle()` | line 95 | ddworksType → ObstacleCategory 변환 |
| `ObstacleInfo.category` | line 148 | 장애물 카테고리 필드 |
| `SpatialContext.structural_columns` | line 241 | 구조기둥 리스트 |
| `SpatialContext.posts` | line 242 | 포스트 리스트 |
| `SpatialContext.h_beams` | line 243 | H-Beam 리스트 |
| `SpatialContext.structural_beams` | line 244 | 구조보 리스트 |
| `SpatialContext.gratings` | line 245 | 그레이팅 리스트 |
| `ObstacleRelationFeatures` | line 447 | 유형별 공간관계 데이터클래스 (18필드) |
| `EnhancedPathFeatures.obstacle_relations` | line 500 | 경로 특징량 내 장애물관계 필드 |
| `ObstacleRelationExtractor` | line 625 | 장애물 공간관계 추출 클래스 |
| `ObstacleRelationExtractor._compute_column_relations()` | line 649 | 구조기둥 관계 계산 |
| `ObstacleRelationExtractor._compute_lr_pattern()` | line 686 | 좌/우 배치 패턴 계산 |
| `ObstacleRelationExtractor._compute_post_relations()` | line 737 | 포스트 관계 계산 |
| `ObstacleRelationExtractor._compute_grid_alignment()` | line 769 | 그리드 정렬도 계산 |
| `ObstacleRelationExtractor._compute_beam_relations()` | line 817 | H-Beam 관계 계산 |
| `ObstacleRelationExtractor._compute_grating_relations()` | line 884 | 그레이팅 관계 계산 |
| `SpatialSimilarity._obstacle_sim()` | line 1096 | 장애물관계 통합 유사도 |
| `SpatialSimilarity._column_relation_sim()` | line 1109 | 기둥 관계 유사도 |
| `SpatialSimilarity._post_relation_sim()` | line 1134 | 포스트 관계 유사도 |
| `SpatialSimilarity._beam_relation_sim()` | line 1149 | 빔 관계 유사도 |
| `SpatialSimilarity._grating_relation_sim()` | line 1170 | 그레이팅 관계 유사도 |

### 5.2 AnalyzeRoutingAi_V2.py

| 함수 | 역할 |
|---|---|
| `_compute_obstacle_relations_for_path()` | 경로에 대한 장애물관계를 딕셔너리로 반환 |
| `_obstacle_relation_similarity()` | 장애물관계 유사도 계산 (딕셔너리 기반) |
| `compute_composite_similarity()` | 5차원 복합 유사도 (장애물관계 0.20 가중치 추가) |
| `run_phase_routing()` | Phase 1에서 SpatialContext 로드 + obstacle_relations 저장 |
| `_load_routing_records()` | Phase 2에서 obstacle_relations 필드 로드 |

---

## 6. 가중치 설계 근거

### 6.1 장애물 유형별 유사도 내부 가중치

| 유형 | 가중치 | 근거 |
|------|--------|------|
| 구조 기둥 | 0.35 | 배관 경로에 가장 큰 영향. 반드시 회피해야 하며, 회피 패턴이 경로 형상을 결정 |
| H-Beam | 0.30 | 수직 공간 제약. 배관 높이를 결정하는 핵심 요소 |
| 그레이팅 | 0.20 | 하부 지지 구조. 배관 하중 지지 및 통과 제약 관련 |
| 포스트 | 0.15 | 간접적 영향. 규칙적 그리드 배치이므로 상대적으로 예측 가능 |

### 6.2 세부 항목별 가중치

**구조 기둥** (합계 1.0):
- 교차수 0.3 + LR패턴 0.3: 실제 회피 행동을 반영하는 가장 중요한 지표
- 근접수 0.2 + 거리 0.2: 환경 유사성 보조 지표

**H-Beam** (합계 1.0):
- 교차수 0.4: 경로가 빔을 지나가는 빈도
- clearance 0.3: 높이 여유 공간의 유사성
- 평행비 0.3: 빔과 경로의 방향 관계

**그레이팅** (합계 1.0):
- 커버리지 0.4: 경로 하부의 구조적 지지 비율
- 하부수 0.3 + 개구부수 0.3: 구조적 연속성

**포스트** (합계 1.0):
- 그리드정렬 0.4: 포스트 배치와 경로의 정렬 패턴
- 밀도 0.3 + 근접수 0.3: 포스트 환경 밀도

---

## 7. 하위 호환성

| 항목 | 호환 방식 |
|------|----------|
| 기존 `self.columns` / `self.beams` 리스트 | 유지 (새 리스트와 병렬) |
| 기존 `obstacle_density` / `column_crossings` 필드 | EnhancedPathFeatures에 유지 |
| `obstacle_relations` 없는 기존 결과 파일 | `_obstacle_relation_similarity()` 에서 1.0(중립) 반환 |
| `compute_composite_similarity()` 가중치 변경 | 기존 4차원 → 5차원, 총합 1.0 유지 |

---

## 8. 실행 방법

```bash
cd d:/DINNO/DEV/AI-AutoRouting/KGraphGen03/Analyzer

# Phase 1+2 분석 (장애물관계 포함)
python AnalyzeRoutingAi_V2.py --phase all

# V2 자동 설계 (장애물관계 유사도 반영)
python AutoRoutingDesigner_V2.py \
  --json_input ./data-v10/CMP_KSCTA01_*.json \
  --equipment kscta01 --utility AKWW --size 20A \
  --start 5500,28200,15495 --dest 5300,27000,15495
```

---

## 9. 향후 과제

1. **가중치 자동 튜닝**: 실제 설계 검증 결과를 피드백하여 유형별 가중치 최적화
2. **장애물 회피 경로 생성 연동**: ObstacleRelationFeatures를 Phase 3 자동 경로 생성 시 우회 전략 선택에 활용
3. **3D 뷰어 시각화**: 유형별 장애물을 색상으로 구분 표시, 경로-장애물 관계선 렌더링
4. **추가 장애물 유형 확장**: 덕트(Duct), 케이블트레이(Cable Tray) 등 추가 BIM 객체와의 관계 분석
