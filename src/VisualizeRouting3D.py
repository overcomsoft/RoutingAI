# =====================================================================================
# [실행 명령어]
#   python VisualizeRouting3D.py ./RoutingResults
#   python VisualizeRouting3D.py ./RoutingResults/CMP_KSCTA01_Path.json
# =====================================================================================
"""
VisualizeRouting3D.py  — 배관 경로 3D 시각화
==============================================

[프로그램 개요]
  Phase 1(AnalyzeRoutingAi_V2.py)에서 추출한 배관 경로 JSON을
  Plotly 3D Scatter를 사용하여 인터랙티브 HTML 파일로 시각화합니다.

[전체 흐름도]
  ┌──────────────────────────────────────────────────────────────┐
  │  1. 입력 경로(디렉토리 또는 단일 JSON) 파싱                  │
  │  2. 경로 JSON 로드 (load_path_json)                          │
  │  3. 3D 시각화 구성 (visualize_paths):                        │
  │     ├─ 시작점(POC) 마커: 빨간 다이아몬드                     │
  │     ├─ 경로 선: 색상 팔레트로 구분된 3D 라인                 │
  │     └─ 종단점 마커: 검정 사각형                              │
  │  4. HTML 파일로 저장 (Visualize_{장비명}.html)               │
  └──────────────────────────────────────────────────────────────┘

[주요 함수]
  - load_path_json(file_path)  : 경로 JSON 파일 로드
  - visualize_paths(data)      : Plotly 3D 그래프 생성 + HTML 저장
  - main()                     : CLI 진입점 (디렉토리/파일 자동 구분)

[주요 변수]
  - colors                     : 경로 구분용 색상 팔레트 (10색)
  - fig                        : Plotly Figure 객체
"""

import json
import os
import glob
import plotly.graph_objects as go
import sys

def load_path_json(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def visualize_paths(data):
    fig = go.Figure()
    
    eq_name = data.get("equipment_name", "Unknown")
    poc_paths = data.get("poc_paths", [])
    
    # 색상 팔레트
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    color_idx = 0

    path_count = 0
    
    for poc_idx, poc_item in enumerate(poc_paths):
        start_poc_info = poc_item.get("start_poc_info", {})
        start_coord = start_poc_info.get("position", [0,0,0])
        poc_id = poc_item.get("start_poc_id", f"POC_{poc_idx}")
        poc_util = start_poc_info.get("utility", "-")
        
        # 시작점 마커
        fig.add_trace(go.Scatter3d(
            x=[start_coord[0]], y=[start_coord[1]], z=[start_coord[2]],
            mode='markers+text',
            marker=dict(size=8, color='red', symbol='diamond'),
            text=[f"START: {poc_id} ({poc_util})"],
            name=f"Start {poc_id}",
            hoverinfo='text'
        ))

        for p_idx, path_data in enumerate(poc_item.get("paths", [])):
            steps = path_data.get("steps", [])
            label = path_data.get("terminal_label", "End")
            
            x, y, z = [], [], []
            hover_texts = []
            
            for step in steps:
                if step["kind"] == "NODE":
                    pos = step["data"].get("position")
                    if pos:
                        x.append(pos[0])
                        y.append(pos[1])
                        z.append(pos[2])
                        hover_texts.append(f"Node: {step['data'].get('id')}<br>Type: {step['data'].get('type')}")
                # EDGE는 선을 잇는 용도이므로 좌표가 중복될 수 있어 NODE 위주로 수집
            
            if x:
                path_color = colors[color_idx % len(colors)]
                color_idx += 1
                path_count += 1
                
                # 경로 선 그리기
                fig.add_trace(go.Scatter3d(
                    x=x, y=y, z=z,
                    mode='lines+markers',
                    line=dict(color=path_color, width=4),
                    marker=dict(size=3, color=path_color),
                    name=f"{poc_util} Path {p_idx+1}",
                    text=hover_texts,
                    hoverinfo='text'
                ))
                
                # 종단점 마커
                fig.add_trace(go.Scatter3d(
                    x=[x[-1]], y=[y[-1]], z=[z[-1]],
                    mode='markers',
                    marker=dict(size=6, color='black', symbol='square'),
                    name=f"End: {label}",
                    hovertext=f"Terminal: {label}<br>ID: {path_data.get('end_node_id')}",
                    hoverinfo='text'
                ))

    fig.update_layout(
        title=f"Routing Paths Visualization - {eq_name} ({path_count} paths)",
        scene=dict(
            xaxis_title='X (mm)',
            yaxis_title='Y (mm)',
            zaxis_title='Z (mm)',
            aspectmode='data' # 실제 비율 유지
        ),
        margin=dict(r=0, l=0, b=0, t=40),
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01)
    )
    
    safe_eq_name = "".join(c if c.isalnum() else "_" for c in eq_name)
    output_html = f"Visualize_{safe_eq_name}.html"
    fig.write_html(output_html)
    print(f"Visualization saved to {output_html}")
    # fig.show() # 로컬 환경이면 브라우저 실행

def main():
    result_dir = "./RoutingResults"
    files = glob.glob(os.path.join(result_dir, "*.json"))
    
    if not files:
        print("No path JSON files found in RoutingResults.")
        return

    # 인자가 있으면 첫 번째 인자를 파일명으로 사용
    if len(sys.argv) > 1:
        target = sys.argv[1]
        if not os.path.exists(target):
            # 파일명이 완전하지 않으면 검색 시도
            matches = [f for f in files if target in f]
            if matches:
                target = matches[0]
            else:
                print(f"File not found: {target}")
                return
    else:
        # 인자가 없으면 파일 목록 출력 후 선택 (또는 첫 번째 파일 자동 선택)
        print("Available path files:")
        for i, f in enumerate(files):
            print(f"[{i}] {os.path.basename(f)}")
        
        # 여기서는 자동화 테스트를 위해 첫 번째 파일을 기본으로 함
        target = files[0]
        print(f"\nAuto-selecting first file: {target}")

    print(f"Loading {target}...")
    data = load_path_json(target)
    visualize_paths(data)

if __name__ == "__main__":
    main()
