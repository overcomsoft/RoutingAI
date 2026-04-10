# Claude Developer Instructions for KGraphGen03 Analyzer Project

## 📋 Project Overview

**프로젝트명**: KGraphGen03 - AI-AutoRouting System  
**목표**: 3D 배관 설계 데이터의 자동 분석, 최적화 라우팅 경로 생성, 및 시각화

**핵심 역할**:
- 3D 배관 JSON 데이터 파싱 및 그래프 기반 분석
- 배관 그룹화 및 클러스터링 알고리즘 적용
- 라우팅 경로 최적화 및 충돌 방지
- 데이터베이스 저장 및 가시화(3D, HTML)

---

## 📁 Project Structure

```
Analyzer/
├── 📊 Data Processing Scripts (메인 분석 스크립트)
│   ├── analyze_group_pipes.py         - 배관 그룹화 및 클러스터링
│   ├── AnalyzeRoutingPath.py          - 라우팅 경로 분석
│   ├── duct_poc_clustering.py         - 덕트 POC(Point of Connection) 분석
│   ├── analyze_group_pipes.py         - 유틸리티별 그룹핑
│   
├── 🗄️ Database
│   ├── bk_autoroutingv7.sql           - 데이터베이스 스키마
│   ├── generate_db_dictionary.py      - DB 딕셔너리 생성
│   ├── generate_schema_docs.py        - DB 스키마 문서 생성
│   ├── debug_db.py                    - DB 디버깅 유틸리티
│   
├── 📈 Visualization
│   ├── VisualizeRouting3D.py          - 3D 라우팅 경로 시각화
│   ├── Visualize_WTNHJ02_.html        - HTML 기반 시각화 (Sample)
│   
├── 💾 Input Data
│   ├── data-v10/                      - v10 형식 JSON 데이터
│   │   ├── CLEAN_WTNHJ02_*.json
│   │   ├── CMP_KSCTA01_*.json
│   │   ├── CVD_TNMHJ02_*.json
│   │   ├── DIFF_DANHJ14_*.json
│   │   ├── ETCH_ELOHJ07_*.json
│   │   ├── METAL_SLWHJ02_*.json
│   │   ├── PHOTO_PSTWA03_*.json
│   
├── 📤 Output
│   ├── RoutingResults/                - 라우팅 경로 JSON 결과
│   │   └── [equipment]_Path.json
│   ├── output/                        - 처리된 데이터 (CSV, DBF, SHP)
│   │   └── group_rule_data-v10_*.csv
│   
└── ⚙️ Configuration & Build
    ├── requirements.txt               - Python 의존성
    ├── build_exe.bat                  - 실행 파일(EXE) 빌드 스크립트
    └── force_cleanup.py               - 캐시/임시 파일 정리
```

---

## 🔧 Core Technologies & Dependencies

**Python 버전**: 3.8+

**핵심 라이브러리**:
- `pandas` - 데이터 처리 및 CSV 변환
- `geopandas` + `shapely` - 지리 데이터 및 기하학 계산
- `psycopg2` - PostgreSQL 데이터베이스 연결
- `openpyxl` - Excel 파일 처리
- `pyinstaller` - EXE 빌드 및 배포
- `fpdf` - PDF 보고서 생성

---

## 🎯 Key Concepts & Domain Knowledge

### 1. **Data Model**
```
JSON 구조:
{
  "Nodes": [
    {"guid": "...", "x": 0.0, "y": 0.0, "z": 0.0, ...}
  ],
  "Edges": [
    {"guid": "...", "startNodeGuid": "...", "endNodeGuid": "...", "diameter": 25.4, "material": "..."}
  ],
  "Equipment": [
    {"guid": "...", "name": "...", "POC": [...]}
  ]
}
```

### 2. **Core Algorithms**

#### 배관 그룹화 (PipeGrouping)
1. **공차 기반 필터링** (Tolerance-based): 3D 좌표 근처도에 따라 파이프 그룹화
   - `TOL_Z_ELEVATION`: Z축 높이 공차 (mm)
   - `MAX_SPACING`: 파이프 간 최대 간격 (mm)

2. **방향 벡터 검증** (Direction Validation): 파이프 정렬/평행성 검증
   - `TOL_ANGLE_DEG`: 각도 공차 (degrees)
   - 내적(Dot Product)을 통한 벡터 유사성 계산

3. **밀집 공간 클러스터링** (Dense Clustering): BFS 기반 최종 군집화
   - 수직, 수평 배관 분류
   - Bounding Box 계산 (Min/Max X, Y, Z)

#### 라우팅 경로 최적화 (Routing Path Optimization)
- 장비 간 최단 경로 계산
- 충돌 방지(Collision Avoidance) 알고리즘
- 비용 함수(거리, 방향 변화 등) 기반 최적화

### 3. **전역 상수 (Global Constants)**
```python
TOL_Z_ELEVATION = 50        # mm (수직 공차)
MAX_SPACING = 100           # mm (배관 간 최대 간격)
TOL_ANGLE_DEG = 15          # degrees (각도 공차)
MIN_PIPE_LENGTH = 10        # mm (최소 배관 길이)
CLUSTER_PROXIMITY = 200     # mm (클러스터 근접 기준)
```

---

## 📌 Main Scripts & Responsibilities

### **analyze_group_pipes.py**
**목적**: 3D 배관 데이터 그룹화 및 통계 분석

**실행 방법**:
```bash
python analyze_group_pipes.py ./data-v10 --mode equipment
# or
python analyze_group_pipes.py ./data-v10 --mode utility
```

**주요 함수**:
- `analyze_json(file_path, mode)` - JSON 파일 파싱 및 파이프라인 실행
- `group_vertical_pipes(pipes, tolerance)` - 수직 배관 그룹화
- `group_horizontal_pipes(pipes, tolerance)` - 수평 배관 그룹화
- `create_pipe_group(pipes, space_info)` - 그룹 통계화
- `save_to_csv(output_file, data)` - CSV 저장
- `save_to_sqlite(output_file, data)` - DB 저장

**출력**:
- CSV 파일 (`group_rule_data-v10_*.csv`)
- SQLite DB 레코드

---

### **AnalyzeRoutingPath.py**
**목적**: 라우팅 경로 분석 및 최적화

**주요 기능**:
- 장비별 연결 경로 추출
- 최단 경로 계산 (Dijkstra or A*)
- 충돌 감지

**입력**: RoutingResults/*.json  
**출력**: 최적화된 경로 데이터

---

### **VisualizeRouting3D.py**
**목적**: 3D 배관 구조 시각화

**출력**:
- HTML 인터랙티브 지도
- 3D 좌표 시각화 (Plotly/Three.js)

---

## 📊 Workflow: Data Flow

```
Input JSON Files (data-v10/*.json)
    ↓
[analyze_group_pipes.py]
    ├─ Parse Nodes, Edges, Equipment
    ├─ Build Graph (GUID 기반)
    ├─ Extract Pipes (BFS)
    ├─ Classify & Group (Vertical/Horizontal)
    ├─ Clustering (Proximity-based)
    └─ Statistics (Bounds, Spacing, Elevation)
    ↓
[Group Data] → CSV + SQLite DB
    ↓
[AnalyzeRoutingPath.py]
    ├─ Load Group Data
    ├─ Calculate Optimal Paths
    ├─ Detect Collisions
    └─ Generate Path JSON
    ↓
[RoutingResults/*.json]
    ↓
[VisualizeRouting3D.py]
    ├─ Render 3D Model
    ├─ Create HTML
    └─ Interactive Visualization
    ↓
[Visualize_*.html] - Final Output
```

---

## ✅ Development Guidelines

### Code Style & Conventions
1. **인코딩**: UTF-8 (한글 주석 지원)
2. **변수 네이밍**: snake_case (함수, 변수), PascalCase (클래스)
3. **문서화**: 각 함수는 한글/영문 docstring 포함
4. **에러 처리**: try-except로 JSON 파싱 등 예외 처리
5. **로깅**: print() 대신 logging 모듈 사용 권장

### Data Validation
- JSON 파일 존재 여부 확인
- Nodes/Edges 데이터 유효성 검증
- GUID 중복 체크
- 좌표 범위 검증 (음수 제외)

### Testing & Debugging
- `debug_db.py` - DB 쿼리 테스트
- 샘플 데이터(`data-v10/`)로 각 모듈 검증
- CSV 출력물 수동 검토

### Version Control
- 주요 변경사항은 주석으로 문서화
- 커밋 메시지: 영문 또는 한글 명확히
- 데이터 파일은 `.gitignore`에 포함 (필요시)

---

## 🔍 Common Issues & Troubleshooting

### JSON 파싱 오류
**증상**: "JSON root is a list" 경고  
**원인**: JSON 형식이 배열 형태  
**해결**: `data.get("Nodes")` 등으로 적절한 필드 접근

### GUID 매칭 실패
**증상**: 엣지의 시작/종료 노드를 찾을 수 없음  
**원인**: GUID 오타 또는 데이터 불일치  
**해결**: GUID 인덱싱 검증, 데이터 정합성 확인

### 메모리 오버플로우 (대용량 파일)
**증상**: Python 메모리 부족  
**해결**: 청크 기반 처리, 불필요한 쿼리 최적화

### 데이터베이스 연결 실패
**증상**: psycopg2 연결 오류  
**확인사항**:
- PostgreSQL 서버 실행 여부
- 연결 문자열 (host, port, user, password)
- `bk_autoroutingv7.sql`로 테이블 생성 여부

---

## 📝 Task Assignment Template

AI 개발자 할당 시 다음 정보 포함:

```
[작업명]
파이프 그룹화 알고리즘 개선

[상세 요구사항]
- 클러스터 근접도 기준값(CLUSTER_PROXIMITY) 조정 시 성능 비교
- 새로운 거리 메트릭(예: Euclidean vs Manhattan) 추가

[입력 데이터]
- data-v10/ 정상 데이터

[예상 출력]
- 수정된 analyze_group_pipes.py
- 성능 비교 보고서 (CSV)

[참고 자료]
- [analyze_group_pipes.py](./analyze_group_pipes.py) (Line 100~150 클러스터링 로직)
- 상수 정의: TOL_Z_ELEVATION, MAX_SPACING (Line 25~30)

[완료 기준]
- 테스트 데이터로 동작 확인
- 기존 출력과 호환성 유지
- 코드 주석 및 docstring 추가
```

---

## 🚀 Quick Start for New Tasks

1. **코드 파악**
   ```bash
   python analyze_group_pipes.py ./data-v10 --mode equipment
   ```
   - 정상 동작 확인, 로그 분석

2. **관련 파일 검토**
   - 메인 스크립트 읽기
   - 상수/설정값 확인
   - 데이터 구조 파악

3. **수정/추가**
   - IDE에서 파일 편집
   - 작은 변경사항부터 테스트
   - 기존 동작 유지 확인

4. **검증**
   - 샘플 데이터로 재실행
   - 출력물 검토 (CSV, 로그)
   - 엣지 케이스 테스트

5. **문서화**
   - 코드 주석 추가
   - 함수 docstring 작성
   - 이 파일(claude.md) 필요시 업데이트

---

## 📞 Project Contact & Context

**담당자**: [프로젝트 소유자]  
**마지막 업데이트**: 2026-03-29  
**현재 상태**: Active Development

**알려진 제약사항**:
- PostgreSQL 선택사항 (SQLite도 지원)
- v10 데이터 형식만 현재 처리
- 대용량 파일(>500MB) 성능 미최적화

---

## 📚 Additional Resources

- Python 3.8+ Official Docs
- GeoPandas Documentation: https://geopandas.org/
- PostGIS/PostgreSQL: https://postgis.net/
- JSON Schema Validation: https://json-schema.org/

---

**이 문서는 AI 개발자(Claude)가 KGraphGen03 프로젝트에서 효율적으로 작업하기 위한 가이드입니다.**

