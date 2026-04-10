# RoutingAI - 3D 배관 자동 라우팅 시스템

## 프로젝트 구조

```
RoutingAI/
├── src/                              # 소스 코드
│   ├── AnalyzeRoutingAi_V2.py        # [Phase 1+2] BFS 경로 추출 + 그룹 클러스터링 (핵심)
│   ├── AnalyzeRoutingAi.py           # [Phase 1+2] V1 (레거시)
│   ├── AutoRoutingDesigner.py        # [Phase 3] 자동 경로 설계 V1
│   ├── AutoRoutingDesigner_V2.py     # [Phase 3] 자동 경로 설계 V2 ★ (장애물 유형별 공간관계)
│   ├── AnalyzeRoutingPath.py         # 라우팅 경로 분석
│   ├── AnalyzeBranching.py           # 분기 분석
│   ├── analyze_group_pipes.py        # 배관 그룹화 (공차/클러스터 기반)
│   ├── import_obstacles_json.py      # 장애물 JSON → DB 임포트
│   ├── import_duct_poc_cluster.py    # 덕트 POC 클러스터 임포트
│   ├── import_pipe_group_analyze.py  # 배관 그룹 분석 결과 임포트
│   ├── import_objects.py             # BIM 객체 임포트
│   ├── duct_poc_clustering.py        # 덕트 POC 클러스터링
│   └── VisualizeRouting3D.py         # 3D 시각화 (Python/Plotly)
│
├── visualization/                    # 웹 기반 3D 뷰어
│   ├── VisualizeGroupPipe3D.html     # 그룹 배관 + 자동 경로 뷰어 (Three.js)
│   └── VisualizeAutoRouting3D.html   # V2 뷰어 (장비BBox + 기둥 + 자동경로)
│
├── data/
│   ├── input/                        # 원본 설계 JSON (7개 장비)
│   │   ├── CLEAN_WTNHJ02_*.json
│   │   ├── CMP_KSCTA01_*.json
│   │   ├── CVD_TNMHJ02_*.json
│   │   ├── DIFF_DANHJ14_*.json
│   │   ├── ETCH_ELOHJ07_*.json
│   │   ├── METAL_SLWHJ02_*.json
│   │   └── PHOTO_PSTWA03_*.json
│   │
│   ├── RoutingResults/               # Phase 1 결과 (장비별 경로 JSON, 30개)
│   ├── GroupPipeResults/             # Phase 2 결과 (그룹 분석 JSON+CSV)
│   └── AutoRoutingResults/           # Phase 3 결과 (자동 생성 경로 JSON)
│
├── doc/                              # 문서
│   ├── RoutingAI-dev.md              # ★ 장애물 유형별 공간관계 확장 개발보고서
│   ├── RoutingAI_Development_Document.md  # V2 개발 문서 (전체 시스템)
│   ├── AnalyzeRoutingAi_V2.md        # V2 분석 엔진 상세 문서
│   ├── CLAUDE.md                     # 프로젝트 가이드
│   ├── PostgreSQL_Schema_*.docx/pdf  # DB 스키마 문서
│   └── 데이터베이스_테이블정의서*.docx
│
└── requirements.txt                  # Python 의존성
```

## 파이프라인 흐름

```
[Phase 1: 경로 추출]
  data/input/*.json → AnalyzeRoutingAi_V2.py --phase routing
  → data/RoutingResults/*_Path.json
  ★ SpatialContext 로드 → 장애물 유형별 분류 → obstacle_relations 저장

[Phase 2: 그룹 클러스터링]
  data/RoutingResults/*.json → AnalyzeRoutingAi_V2.py --phase grouping
  → data/GroupPipeResults/group_pipe_results_*.json
  ★ 5차원 유사도: arrow + vector + range + length + obstacle_relation

[Phase 3: 자동 경로 설계]
  data/input/*.json + GroupPipeResults/*.json
  → AutoRoutingDesigner_V2.py --equipment ... --start ... --dest ...
  → data/AutoRoutingResults/auto_routing_v2_*.json
  ★ 8차원 유사도: 기존 4개 + 장비상대 + 종단점 + 장애물관계(유형별) + 레벨

[시각화]
  → VisualizeAutoRouting3D.html (장비/기둥/자동경로 통합 3D 뷰)
```

## 실행 방법

```bash
# Phase 1+2: 경로 추출 및 그룹 분석
python src/AnalyzeRoutingAi_V2.py --phase all --input data/input

# Phase 3: 자동 경로 설계 (V2 - 장애물 유형별 공간관계 반영)
python src/AutoRoutingDesigner_V2.py \
    --json_input data/input/CMP_KSCTA01_*.json \
    --group_results data/GroupPipeResults/group_pipe_results_20260405194814.json \
    --equipment kscta01 --utility AKWW --size 20A \
    --start 5500,28200,15495 --dest 5300,27000,15495

# 공간 컨텍스트 확인
python src/AutoRoutingDesigner_V2.py --json_input data/input/CMP_*.json --spatial_summary

# 그룹 목록 조회
python src/AutoRoutingDesigner_V2.py --list_groups --filter_equipment kscta01

# 시각화 (로컬 서버 필요)
python -m http.server 8080
# → http://localhost:8080/visualization/VisualizeAutoRouting3D.html
```

## 장애물 유형 분류 (ObstacleCategory)

| ObstacleCategory | ddworksType | ostType | 실제 객체 | 수량 (KSCTA01) |
|---|---|---|---|---|
| STRUCTURAL_COLUMN | COLUMN_STRUCTURE | OST_StructuralColumns | 구조 기둥 | 18 |
| POST | COLUMN_ARCHITECTURE | OST_Columns | Access Floor 포스트 | 192 |
| H_BEAM | BEAM_ARCHITECTURE | OST_BeamStartSegment | H-Beam | 32 |
| STRUCTURAL_BEAM | BEAM_STRUCTURE | OST_StructuralFraming | 구조 보 | 10 |
| GRATING | FLOOR_ARCHITECTURE | OST_Floors | 그레이팅 | 203 |
| CEILING | CEILING_ARCHITECTURE | OST_Ceilings | 천장 | 1 |

## 장애물 유형별 공간관계 특징량 (18개)

| 유형 | 특징량 | 설명 |
|------|--------|------|
| 구조기둥 | col_count_nearby | 경로 근방 기둥 수 |
| 구조기둥 | col_min_distance / col_avg_distance | 최근접/평균 거리 (mm) |
| 구조기둥 | col_crossings | 경로-기둥 교차 수 |
| 구조기둥 | col_relative_pattern | 기둥 좌/우 배치 패턴 (L/R/B) |
| 포스트 | post_count_nearby | 경로 근방 포스트 수 |
| 포스트 | post_density | 포스트 밀도 (0~1) |
| 포스트 | post_grid_alignment | 포스트 그리드 정렬도 (0~1) |
| H-Beam | beam_count_crossing | 빔 교차 수 |
| H-Beam | beam_min_clearance | 최소 수직 간격 (mm) |
| H-Beam | beam_parallel_ratio | 경로와 빔의 평행 비율 |
| 그레이팅 | grating_coverage | 경로 하부 커버리지 (0~1) |
| 그레이팅 | grating_count_below | 경로 아래 그레이팅 수 |
| 그레이팅 | grating_gap_count | 그레이팅 빈틈(개구부) 수 |

## 유사도 가중치

### AutoRoutingDesigner_V2 (8차원)

| 특징량 | 가중치 | 비고 |
|--------|--------|------|
| arrow | 0.12 | 방향 패턴 |
| vector | 0.12 | 벡터 시퀀스 |
| range | 0.08 | 공간 범위 |
| length | 0.08 | 경로 길이 |
| equip_relative | 0.15 | 장비 상대 좌표 |
| terminal | 0.15 | 종단점 |
| **obstacle** | **0.20** | **유형별 세분화** (기둥0.35 + 빔0.30 + 그레이팅0.20 + 포스트0.15) |
| level | 0.10 | SpaceInfo 레벨 |

### AnalyzeRoutingAi_V2 (5차원)

| 특징량 | 가중치 |
|--------|--------|
| arrow | 0.25 |
| vector | 0.25 |
| range | 0.15 |
| length | 0.15 |
| **obstacle_relation** | **0.20** |

## 문서 안내

| 문서 | 내용 |
|------|------|
| [doc/RoutingAI-dev.md](doc/RoutingAI-dev.md) | 장애물 유형별 공간관계 확장 개발보고서 (최신) |
| [doc/RoutingAI_Development_Document.md](doc/RoutingAI_Development_Document.md) | V2 전체 시스템 개발 문서 |
| [doc/AnalyzeRoutingAi_V2.md](doc/AnalyzeRoutingAi_V2.md) | Phase 1+2 분석 엔진 상세 |

## 최종 업데이트

- 2026-04-10: 장애물 유형별 공간관계 특징량 확장 (18개 항목, 유사도 세분화)
- 2026-04-05: V2 개발 완료 (장비형상/장애물/종단점 고려 자동 설계)
