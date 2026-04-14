"""
VisualizeVacuumPumpPaths.py
============================
Vaccum->Pump 배관경로 추출 결과 시각화 스크립트 (Plotly 기반)

ExtractVacuumToPumpPaths.py 의 출력(data/VacuumPumpPaths/*.json)을 읽어
통합 HTML 대시보드를 생성한다.

  1. 3D 배관 경로 (장비별 색상, 인터랙티브)
  2. 방향 패턴(direction_pattern) 빈도 분포
  3. 장비별 경로 수 & 평균 스텝 수
  4. 유틸리티별 경로 수 & 평균 총 길이
  5. 세그먼트 방향(R/H/D) 비율 (전체 파이 + 장비별 스택바)
  6. 경로별 총 길이 분포 (히스토그램 + 장비별 박스플롯)

================================================================================
[실행 명령어]
================================================================================

  # 기본 실행 (data/VacuumPumpPaths -> 같은 디렉토리에 HTML 저장)
  python src/VisualizeVacuumPumpPaths.py

  # 입력/출력 디렉토리 지정
  python src/VisualizeVacuumPumpPaths.py -i ./data/VacuumPumpPaths -o ./data/VacuumPumpPaths
"""

import json
import glob
import os
import math
import argparse
from collections import Counter, defaultdict

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np


# ============================================================================
# 데이터 로드
# ============================================================================
def load_all_paths(input_dir: str) -> list[dict]:
    """VacuumPumpPaths 디렉토리에서 *_vacuum_pump_paths.json 파일을 모두 로드한다."""
    pattern = os.path.join(input_dir, "*_vacuum_pump_paths.json")
    files = sorted(glob.glob(pattern))
    all_paths = []
    for fp in files:
        with open(fp, "r", encoding="utf-8") as f:
            data = json.load(f)
            all_paths.extend(data)
    return all_paths


def compute_total_length(path_record: dict) -> float:
    """경로의 총 길이(mm)를 변위 벡터의 length 합계로 산출한다."""
    return sum(d["length"] for d in path_record.get("displacements", []))


def count_direction_segments(path_record: dict) -> Counter:
    """경로의 방향별(R/H/D) 세그먼트 수를 카운트한다."""
    raw = path_record.get("direction_raw", "")
    if not raw:
        return Counter()
    return Counter(raw.split("-"))


# ============================================================================
# 색상 팔레트
# ============================================================================
EQUIPMENT_COLORS = {}
COLOR_PALETTE = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
]


def get_equip_color(name: str) -> str:
    """장비명에 고유 색상을 할당하여 반환한다."""
    if name not in EQUIPMENT_COLORS:
        idx = len(EQUIPMENT_COLORS) % len(COLOR_PALETTE)
        EQUIPMENT_COLORS[name] = COLOR_PALETTE[idx]
    return EQUIPMENT_COLORS[name]


# ============================================================================
# Chart 1: 3D 배관 경로
# ============================================================================
def create_3d_paths(all_paths: list[dict]) -> go.Figure:
    fig = go.Figure()
    legend_added = set()

    for rec in all_paths:
        positions = []
        for step in rec.get("path", []):
            if step["kind"] == "NODE" and step.get("position"):
                pos = step["position"]
                if isinstance(pos, list) and len(pos) >= 3:
                    positions.append(pos)
        if len(positions) < 2:
            continue

        equip = rec["equipment_name"]
        color = get_equip_color(equip)
        show_legend = equip not in legend_added
        legend_added.add(equip)

        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]

        poc = rec["start_poc_name"]
        utility = rec["start_poc_utility"]
        pattern = rec["direction_pattern"]
        total_len = compute_total_length(rec) / 1000

        fig.add_trace(go.Scatter3d(
            x=xs, y=ys, z=zs,
            mode="lines+markers",
            line=dict(color=color, width=4),
            marker=dict(size=2, color=color),
            name=equip, legendgroup=equip, showlegend=show_legend,
            hovertemplate=(
                f"<b>{equip}</b> [{poc}]<br>"
                f"유틸리티: {utility}<br>"
                f"패턴: {pattern}<br>"
                f"길이: {total_len:.1f}m<br>"
                "좌표: (%{x:.0f}, %{y:.0f}, %{z:.0f})<extra></extra>"
            ),
        ))

        # 시작점
        fig.add_trace(go.Scatter3d(
            x=[xs[0]], y=[ys[0]], z=[zs[0]],
            mode="markers",
            marker=dict(size=5, color=color, symbol="circle"),
            legendgroup=equip, showlegend=False,
            hovertemplate=f"<b>시작</b> {poc}<extra></extra>",
        ))
        # Pump 종단점
        fig.add_trace(go.Scatter3d(
            x=[xs[-1]], y=[ys[-1]], z=[zs[-1]],
            mode="markers",
            marker=dict(size=7, color="red", symbol="diamond"),
            legendgroup=equip, showlegend=False,
            hovertemplate=f"<b>Pump</b> {rec['end_node_name']}<extra></extra>",
        ))

    fig.update_layout(
        title=dict(text="1. Vaccum -> Pump 3D 배관 경로", font=dict(size=16)),
        scene=dict(
            xaxis_title="X (mm)", yaxis_title="Y (mm)", zaxis_title="Z (mm)",
            aspectmode="data",
        ),
        legend=dict(font=dict(size=11)),
        height=700,
    )
    return fig


# ============================================================================
# Chart 2: 방향 패턴 빈도 분포
# ============================================================================
def create_direction_pattern_chart(all_paths: list[dict]) -> go.Figure:
    counter = Counter(r["direction_pattern"] for r in all_paths)
    patterns = sorted(counter.keys(), key=lambda k: counter[k], reverse=True)
    counts = [counter[p] for p in patterns]

    fig = go.Figure()
    fig.add_trace(go.Bar(
        y=patterns, x=counts, orientation="h",
        marker_color="#4c72b0",
        text=counts, textposition="outside",
        hovertemplate="패턴: %{y}<br>경로 수: %{x}<extra></extra>",
    ))
    fig.update_layout(
        title=dict(text="2. 방향 패턴(Direction Pattern) 빈도 분포", font=dict(size=16)),
        xaxis_title="경로 수",
        yaxis=dict(autorange="reversed", tickfont=dict(family="monospace", size=12)),
        height=max(350, len(patterns) * 45),
    )
    return fig


# ============================================================================
# Chart 3: 장비별 경로 수 & 평균 스텝 수
# ============================================================================
def create_equipment_stats_chart(all_paths: list[dict]) -> go.Figure:
    equip_paths = defaultdict(list)
    for r in all_paths:
        equip_paths[r["equipment_name"]].append(r)

    equips = sorted(equip_paths.keys())
    path_counts = [len(equip_paths[e]) for e in equips]
    avg_steps = [np.mean([r["step_count"] for r in equip_paths[e]]) for e in equips]
    colors = [get_equip_color(e) for e in equips]

    fig = make_subplots(specs=[[{"secondary_y": True}]])

    fig.add_trace(go.Bar(
        x=equips, y=path_counts,
        marker_color=colors,
        text=path_counts, textposition="outside",
        name="경로 수",
        hovertemplate="장비: %{x}<br>경로 수: %{y}<extra></extra>",
    ), secondary_y=False)

    fig.add_trace(go.Scatter(
        x=equips, y=avg_steps,
        mode="lines+markers+text",
        marker=dict(size=12, color="#d62728", symbol="diamond"),
        line=dict(color="#d62728", width=2),
        text=[f"{s:.0f}" for s in avg_steps],
        textposition="top center",
        textfont=dict(color="#d62728", size=12),
        name="평균 스텝 수",
        hovertemplate="장비: %{x}<br>평균 스텝: %{y:.1f}<extra></extra>",
    ), secondary_y=True)

    fig.update_layout(
        title=dict(text="3. 장비별 경로 수 & 평균 스텝 수", font=dict(size=16)),
        yaxis_title="경로 수",
        yaxis2_title="평균 스텝 수",
        height=450,
        legend=dict(x=0.01, y=0.99),
    )
    return fig


# ============================================================================
# Chart 4: 유틸리티별 경로 수 & 평균 총 길이
# ============================================================================
def create_utility_stats_chart(all_paths: list[dict]) -> go.Figure:
    util_paths = defaultdict(list)
    for r in all_paths:
        util_paths[r["start_poc_utility"]].append(r)

    utils = sorted(util_paths.keys())
    path_counts = [len(util_paths[u]) for u in utils]
    avg_lengths = [np.mean([compute_total_length(r) / 1000 for r in util_paths[u]]) for u in utils]

    util_colors = ["#4c72b0", "#dd8452", "#55a868", "#c44e52", "#8172b3"]

    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=["유틸리티별 경로 수", "유틸리티별 평균 경로 길이(m)"])

    colors = [util_colors[i % len(util_colors)] for i in range(len(utils))]

    fig.add_trace(go.Bar(
        x=utils, y=path_counts, marker_color=colors,
        text=path_counts, textposition="outside",
        showlegend=False,
        hovertemplate="%{x}: %{y}개<extra></extra>",
    ), row=1, col=1)

    fig.add_trace(go.Bar(
        x=utils, y=avg_lengths, marker_color=colors,
        text=[f"{l:.1f}m" for l in avg_lengths], textposition="outside",
        showlegend=False,
        hovertemplate="%{x}: %{y:.1f}m<extra></extra>",
    ), row=1, col=2)

    fig.update_layout(
        title=dict(text="4. 유틸리티별 통계", font=dict(size=16)),
        height=400,
    )
    fig.update_yaxes(title_text="경로 수", row=1, col=1)
    fig.update_yaxes(title_text="평균 길이 (m)", row=1, col=2)
    return fig


# ============================================================================
# Chart 5: 세그먼트 방향(R/H/D) 비율
# ============================================================================
def create_direction_ratio_chart(all_paths: list[dict]) -> go.Figure:
    total_counter = Counter()
    equip_counters = defaultdict(Counter)
    for r in all_paths:
        c = count_direction_segments(r)
        total_counter += c
        equip_counters[r["equipment_name"]] += c

    dir_keys = ["R", "H", "D"]
    dir_labels = ["R (수직)", "H (수평)", "D (경사)"]
    dir_colors = ["#e74c3c", "#3498db", "#f39c12"]

    fig = make_subplots(
        rows=1, cols=2,
        specs=[[{"type": "pie"}, {"type": "bar"}]],
        subplot_titles=["전체 방향 비율", "장비별 방향 비율 (%)"],
    )

    # 왼쪽: 전체 파이
    sizes = [total_counter.get(k, 0) for k in dir_keys]
    fig.add_trace(go.Pie(
        labels=dir_labels, values=sizes,
        marker=dict(colors=dir_colors),
        hole=0.4,
        textinfo="percent+label",
        textfont=dict(size=12),
    ), row=1, col=1)

    # 오른쪽: 장비별 스택바
    equips = sorted(equip_counters.keys())
    for i, (dk, dl, dc) in enumerate(zip(dir_keys, dir_labels, dir_colors)):
        vals = []
        for e in equips:
            c = equip_counters[e]
            s = sum(c.values()) or 1
            vals.append(c.get(dk, 0) / s * 100)
        fig.add_trace(go.Bar(
            y=equips, x=vals, orientation="h",
            name=dl, marker_color=dc,
            hovertemplate="%{y}: %{x:.1f}%<extra></extra>",
        ), row=1, col=2)

    fig.update_layout(
        title=dict(text="5. 세그먼트 방향(R/H/D) 비율", font=dict(size=16)),
        barmode="stack",
        height=450,
        legend=dict(x=0.55, y=0.01),
    )
    fig.update_xaxes(title_text="비율 (%)", range=[0, 100], row=1, col=2)
    return fig


# ============================================================================
# Chart 6: 경로별 총 길이 분포
# ============================================================================
def create_length_distribution_chart(all_paths: list[dict]) -> go.Figure:
    lengths_m = [compute_total_length(r) / 1000 for r in all_paths]

    equip_lengths = defaultdict(list)
    for r in all_paths:
        equip_lengths[r["equipment_name"]].append(compute_total_length(r) / 1000)

    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=["경로 길이 분포 (히스토그램)", "장비별 경로 길이 (박스플롯)"])

    # 왼쪽: 히스토그램
    fig.add_trace(go.Histogram(
        x=lengths_m, nbinsx=max(5, len(lengths_m) // 2),
        marker_color="#4c72b0",
        name="경로 수",
        hovertemplate="길이: %{x:.1f}m<br>수: %{y}<extra></extra>",
    ), row=1, col=1)

    # 평균선
    mean_val = np.mean(lengths_m)
    fig.add_vline(x=mean_val, line_dash="dash", line_color="#d62728",
                  annotation_text=f"평균: {mean_val:.1f}m",
                  annotation_font_color="#d62728",
                  row=1, col=1)

    # 오른쪽: 장비별 박스플롯
    equips = sorted(equip_lengths.keys())
    for e in equips:
        fig.add_trace(go.Box(
            y=equip_lengths[e],
            name=e,
            marker_color=get_equip_color(e),
            boxmean=True,
            hovertemplate="%{y:.1f}m<extra></extra>",
        ), row=1, col=2)

    fig.update_layout(
        title=dict(text="6. 경로별 총 길이 분포", font=dict(size=16)),
        height=450,
        showlegend=False,
    )
    fig.update_xaxes(title_text="총 경로 길이 (m)", row=1, col=1)
    fig.update_yaxes(title_text="경로 수", row=1, col=1)
    fig.update_yaxes(title_text="총 경로 길이 (m)", row=1, col=2)
    return fig


# ============================================================================
# 요약 테이블 HTML
# ============================================================================
def create_summary_table_html(all_paths: list[dict]) -> str:
    """경로 요약 테이블 HTML을 생성한다."""
    rows_html = ""
    for i, r in enumerate(all_paths, 1):
        total_m = compute_total_length(r) / 1000
        color = get_equip_color(r["equipment_name"])
        rows_html += f"""
        <tr>
            <td>{i}</td>
            <td style="color:{color}; font-weight:bold">{r['equipment_name']}</td>
            <td>{r['start_poc_name']}</td>
            <td>{r['start_poc_utility']}</td>
            <td><code>{r['direction_pattern']}</code></td>
            <td>{r['step_count']}</td>
            <td>{total_m:.1f}</td>
            <td>{r['end_node_name']}</td>
        </tr>"""

    lengths = [compute_total_length(r) / 1000 for r in all_paths]
    steps = [r["step_count"] for r in all_paths]

    return f"""
    <div style="margin:20px; font-family:sans-serif;">
        <h2>Vaccum -> Pump 경로 분석 요약 (총 {len(all_paths)}개 경로)</h2>
        <div style="display:flex; gap:30px; margin-bottom:15px;">
            <div style="background:#f0f4f8; padding:12px 20px; border-radius:8px;">
                <b>길이</b>: 평균 {np.mean(lengths):.1f}m / 최소 {min(lengths):.1f}m / 최대 {max(lengths):.1f}m
            </div>
            <div style="background:#f0f4f8; padding:12px 20px; border-radius:8px;">
                <b>스텝</b>: 평균 {np.mean(steps):.0f} / 최소 {min(steps)} / 최대 {max(steps)}
            </div>
            <div style="background:#f0f4f8; padding:12px 20px; border-radius:8px;">
                <b>장비</b>: {len(set(r['equipment_name'] for r in all_paths))}개 /
                <b>패턴</b>: {len(set(r['direction_pattern'] for r in all_paths))}종류
            </div>
        </div>
        <table border="1" cellpadding="6" cellspacing="0"
               style="border-collapse:collapse; font-size:13px; width:100%;">
            <thead style="background:#2c3e50; color:white;">
                <tr>
                    <th>No</th><th>장비</th><th>PoC</th><th>유틸리티</th>
                    <th>방향 패턴</th><th>스텝</th><th>길이(m)</th><th>종단 Pump</th>
                </tr>
            </thead>
            <tbody>{rows_html}</tbody>
        </table>
    </div>"""


# ============================================================================
# 통합 HTML 대시보드 생성
# ============================================================================
def generate_dashboard(all_paths: list[dict], output_dir: str):
    """모든 차트를 통합한 HTML 대시보드를 생성한다."""

    # 각 차트 생성
    charts = [
        create_3d_paths(all_paths),
        create_direction_pattern_chart(all_paths),
        create_equipment_stats_chart(all_paths),
        create_utility_stats_chart(all_paths),
        create_direction_ratio_chart(all_paths),
        create_length_distribution_chart(all_paths),
    ]

    # 개별 HTML div 생성
    chart_divs = []
    for i, fig in enumerate(charts):
        div_html = fig.to_html(full_html=False, include_plotlyjs=(i == 0),
                               div_id=f"chart_{i+1}")
        chart_divs.append(div_html)

    # 요약 테이블
    summary_html = create_summary_table_html(all_paths)

    # 통합 HTML
    dashboard_html = f"""<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>Vaccum -> Pump 배관경로 분석 대시보드</title>
    <style>
        body {{
            font-family: 'Malgun Gothic', 'NanumGothic', sans-serif;
            background: #f5f7fa;
            margin: 0;
            padding: 20px;
        }}
        .header {{
            background: linear-gradient(135deg, #2c3e50 0%, #3498db 100%);
            color: white;
            padding: 25px 30px;
            border-radius: 12px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }}
        .header h1 {{ margin: 0; font-size: 24px; }}
        .header p {{ margin: 5px 0 0; opacity: 0.85; font-size: 14px; }}
        .chart-container {{
            background: white;
            border-radius: 10px;
            margin-bottom: 20px;
            padding: 15px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.08);
        }}
        table {{ width: 100%; }}
        table tr:nth-child(even) {{ background: #f8f9fa; }}
        table tr:hover {{ background: #e8f4fd; }}
        code {{ background: #eef; padding: 2px 6px; border-radius: 3px; font-size: 12px; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>Vaccum -> Pump 배관경로 분석 대시보드</h1>
        <p>총 {len(all_paths)}개 경로 | {len(set(r['equipment_name'] for r in all_paths))}개 장비 |
           {len(set(r['direction_pattern'] for r in all_paths))}종류 패턴</p>
    </div>

    {summary_html}

    {"".join(f'<div class="chart-container">{div}</div>' for div in chart_divs)}

</body>
</html>"""

    out_path = os.path.join(output_dir, "vacuum_pump_dashboard.html")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(dashboard_html)
    print(f"  대시보드 저장: {out_path}")

    # 개별 차트 HTML도 저장
    chart_names = [
        "chart1_3d_paths",
        "chart2_direction_patterns",
        "chart3_equipment_stats",
        "chart4_utility_stats",
        "chart5_direction_ratio",
        "chart6_length_distribution",
    ]
    for name, fig in zip(chart_names, charts):
        fp = os.path.join(output_dir, f"{name}.html")
        fig.write_html(fp, include_plotlyjs="cdn")
        print(f"  [{name}] -> {fp}")


# ============================================================================
# 콘솔 요약 출력
# ============================================================================
def print_summary_table(all_paths: list[dict]):
    print(f"\n{'='*80}")
    print(f"  Vaccum -> Pump 경로 분석 요약  (총 {len(all_paths)}개 경로)")
    print(f"{'='*80}")
    print(f"{'No':>3}  {'장비':^14}  {'PoC':^10}  {'유틸리티':^10}  "
          f"{'패턴':^16}  {'스텝':>4}  {'길이(m)':>8}")
    print(f"{'-'*80}")

    for i, r in enumerate(all_paths, 1):
        total_m = compute_total_length(r) / 1000
        print(f"{i:3d}  {r['equipment_name']:^14s}  {r['start_poc_name']:^10s}  "
              f"{r['start_poc_utility']:^10s}  {r['direction_pattern']:^16s}  "
              f"{r['step_count']:4d}  {total_m:8.1f}")

    lengths = [compute_total_length(r) / 1000 for r in all_paths]
    steps = [r["step_count"] for r in all_paths]
    print(f"{'-'*80}")
    print(f"  길이 : 평균 {np.mean(lengths):.1f}m  |  최소 {min(lengths):.1f}m  |  최대 {max(lengths):.1f}m")
    print(f"  스텝 : 평균 {np.mean(steps):.0f}     |  최소 {min(steps)}     |  최대 {max(steps)}")
    print(f"  장비 : {len(set(r['equipment_name'] for r in all_paths))}개")
    print(f"  패턴 : {len(set(r['direction_pattern'] for r in all_paths))}종류")
    print(f"{'='*80}\n")


# ============================================================================
# 메인
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="Vaccum->Pump 배관경로 결과 시각화")
    parser.add_argument(
        "--input", "-i",
        default=os.path.join(os.path.dirname(__file__), "..", "data", "VacuumPumpPaths"),
        help="입력 디렉토리 (ExtractVacuumToPumpPaths 출력 경로)",
    )
    parser.add_argument(
        "--output", "-o", default=None,
        help="출력 디렉토리 (기본: 입력 디렉토리와 동일)",
    )
    args = parser.parse_args()

    input_dir = os.path.abspath(args.input)
    output_dir = os.path.abspath(args.output) if args.output else input_dir
    os.makedirs(output_dir, exist_ok=True)

    print(f"입력 디렉토리: {input_dir}")
    all_paths = load_all_paths(input_dir)
    if not all_paths:
        print("[ERROR] 경로 데이터를 찾을 수 없습니다.")
        return

    print(f"로드 완료: {len(all_paths)}개 경로\n")

    # 콘솔 요약
    print_summary_table(all_paths)

    # 대시보드 생성
    print("대시보드 생성 중...")
    generate_dashboard(all_paths, output_dir)

    print(f"\n완료! 브라우저에서 vacuum_pump_dashboard.html 을 열어주세요.")


if __name__ == "__main__":
    main()
