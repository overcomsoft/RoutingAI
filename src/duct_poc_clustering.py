# ==============================================================================
# 실행 방법 (Command Line)
# python duct_poc_clustering.py <DATA_DIRECTORY_PATH>
# 예시: python duct_poc_clustering.py ./data-v10
# 예시: python duct_poc_clustering.py ./data-v11
# [전체적인 흐름도 및 알고리즘]
# 1. 인자로 받은 디렉토리 내의 모든 JSON 파일을 재귀적으로 탐색합니다.
# 2. 각 JSON 파일에서 노드(Nodes)와 장비(Equipment) 중 덕트(DUCT) 타입의 정보를 추출하여 해시맵(Dictionary)을 구성합니다.
# 3. 장비(Equipment)가 소유한 PoC(Start PoC)와 그 종단점(End PoC)이 지칭하는 대상이 덕트(Duct)인지 확인합니다.
# 4. 덕트에 연결된 PoC(TakeOff 포인트 등)의 좌표(x,y,z)를 가져옵니다.
# 5. 덕트의 3D 바운딩 박스를 통해, 각 PoC가 결속된 덕트의 대상 면적 방향(Face: 상단/좌측/우측 등)을 판별해냅니다.
# 6. 식별된(추출된) 덕트 PoC 목록을 장비명-덕트ID 기준으로 그룹화합니다.
# 7. 그룹화된 데이터를 기반으로, 각 유틸리티별 집합에 대한 PoC 간의 내부 최소, 최대 유클리디안 거리를 연산하여 클러스터링을 구성합니다.
# 8. 클러스터 그룹 내 PoC 포인트들을 감싸는 좌표축별 Range와 직육면체 3D 바운딩 박스를 계산해냅니다.
# 9. 모든 JSON 파일의 검사가 끝나면 CSV, Excel 포맷으로 내보내기를 수행합니다.
#
# [주요 함수 설명]
# - calc_dist(p1, p2): 3차원 유클리드 거리를 산출.
# - detect_poc_face(poc_pos, duct_bbox): 바운딩 박스를 기준으로 대상 PoC가 꽂히는 면을 (TOP/LEFT/RIGHT/UNKNOWN)으로 판정합니다.
# - min_distance(pocs): 단일 클러스터/유틸리티 내부 노드간의 최단 이격 거리를 구합니다.
# - max_distance(pocs): 클러스터/유틸리티 내부 노드간 뻗어나간 장축(최대선) 거리를 구합니다.
# - extract_duct_poc_analysis(directory): 디렉토리 대상 JSON 파일 핵심 분석 루프.
# - export_results(records, output_prefix): CSV, Excel 파일 생성.
#
# [주요 변수 설명]
# - records: 최종 CSV, Excel 통계 익스포트에 사용할 리스트 형식의 딕셔너리 테이블.
# - node_map: JSON `nodes` 객체들을 GUID 기준으로 빠른 탐색(O(1))이 가능하게 만든 맵.
# - duct_info_map: 분석 대상 장비 중 '덕트(DUCT)'만을 거른 사전형 정보.
# ==============================================================================

import os
import json
import math
import sys
import glob
import csv
import pandas as pd
from collections import defaultdict
from datetime import datetime

def calc_dist(p1, p2):
    """ 두 3D 좌표 사이의 유클리디안 거리를 계산합니다. """
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

def min_distance(pocs):
    if len(pocs) < 2:
        return 0.0
    min_dist = float('inf')
    for i in range(len(pocs)):
        for j in range(i + 1, len(pocs)):
            dist = calc_dist(pocs[i]['pos'], pocs[j]['pos'])
            if dist < min_dist:
                min_dist = dist
    return round(min_dist, 2)

def max_distance(pocs):
    if len(pocs) < 2:
        return 0.0
    max_dist = -1.0
    for i in range(len(pocs)):
        for j in range(i + 1, len(pocs)):
            dist = calc_dist(pocs[i]['pos'], pocs[j]['pos'])
            if dist > max_dist:
                max_dist = dist
    return round(max_dist, 2)

def extract_duct_poc_analysis(directory):
    records = []
    search_pattern = os.path.join(directory, "**", "*.json")
    json_files = glob.glob(search_pattern, recursive=True)
    
    if not json_files:
        print(f"지정된 경로({directory})에 JSON 파일이 존재하지 않습니다.")
        return []

    print(f"디렉토리 내 {len(json_files)}개의 JSON 파일을 분석합니다...")

    for file_path in json_files:
        file_name = os.path.basename(file_path)
        print(f"[{file_name}] 분석 시작...")
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            print(f"   [Error] 파일 읽기 오류: {file_name} -> {e}")
            continue
            
        nodes = data.get("Nodes", data.get("nodes", []))
        # 노드 맵 생성: guid와 id 모두로 검색 가능하도록 인덱싱
        node_map = {}
        for n in nodes:
            g = n.get("guid") or n.get("Guid")
            i = n.get("id")
            if g: node_map[g] = n
            if i: node_map[i] = n

        equipments = data.get("Equipment", [])
        edges = data.get("Edges", [])
        space_info = data.get("FileInfo", {}).get("SpaceInfo", [])
        
        print(f"   [Data] 전체 노드(PoC): {len(nodes)}개, 전체 엣지: {len(edges)}개, 전체 장비: {len(equipments)}개")
        
        # duct_info_map: guid -> {'center_z', 'h1', 'h2', 'level'}
        duct_info_map = {}
        for edge in edges:
            g = edge.get('guid') or edge.get('Guid')
            if not g:
                continue
            pos = edge.get('position', [])
            size_str = edge.get('size', '')
            center_z = None
            h1, h2 = 0.0, 0.0
            level_name = "N/A"
            
            if pos and size_str:
                try:
                    parts = size_str.lower().replace(' ', '').split('x')
                    dim1_mm = float(parts[0]) if len(parts) >= 1 else 0.0
                    dim2_mm = float(parts[1]) if len(parts) >= 2 else 0.0
                    h1, h2 = dim1_mm / 2.0, dim2_mm / 2.0
                    center_z = pos[2] if isinstance(pos, list) and len(pos) >= 3 else 0.0
                    
                    # Calculate Level from SpaceInfo
                    if space_info:
                        for space in space_info:
                            boundary = space.get("boundary", {})
                            lvl_min_z = boundary.get("min", {}).get("z")
                            lvl_max_z = boundary.get("max", {}).get("z")
                            if lvl_min_z is not None and lvl_max_z is not None:
                                if lvl_min_z <= center_z <= lvl_max_z:
                                    level_name = space.get("levelName") or "N/A"
                                    break
                except Exception:
                    center_z = None
            duct_info_map[g] = {'center_z': center_z, 'h1': h1, 'h2': h2, 'level': level_name}
        
        if duct_info_map:
            print(f"   [Step 1] Edges 데이터에서 {len(duct_info_map)}개의 덕트 기하 정보 식별")

        # Filename splitting for metadata
        fn_parts = file_name.replace('.json', '').split('_')
        f_process = fn_parts[0] if len(fn_parts) > 0 else "N/A"
        f_maker = fn_parts[1] if len(fn_parts) > 1 else "N/A"
        
        f_bay = "N/A"
        if len(fn_parts) > 2:
            # 3번째 구간이 6자리 숫자(날짜 YYMMDD)라면 BAY는 생략된 것으로 간주
            if not (fn_parts[2].isdigit() and len(fn_parts[2]) == 6):
                f_bay = fn_parts[2]


        duct_equipments = defaultdict(set)
        duct_pocs_dict = {}
        
        # [Step 2] Equipment -> ends -> type: "DUCT" 기반 추출 로직
        for eq in equipments:
            eq_name = eq.get('name', 'Unknown')
            eq_maker = eq.get('maker', '')
            ends = eq.get('ends', [])
            
            for end_obj in ends:
                if end_obj.get('type') == 'DUCT':
                    duct_guid = end_obj.get('guid') or end_obj.get('Guid')
                    duct_id = end_obj.get('id') or duct_guid
                    
                    if eq_name and eq_name != 'Unknown':
                        duct_equipments[duct_id].add(eq_name)
                    
                    # 덕트 객체 내의 pocList 순회
                    for poc in end_obj.get('pocList', []):
                        poc_id = poc.get('id')
                        utility = poc.get('utility', 'Unknown')
                        
                        # Nodes 컬렉션에서 ID 매칭을 통한 좌표 획득
                        node = node_map.get(poc_id)
                        if node and node.get('position'):
                            pos = node.get('position')
                            poc_id = node.get('id') or poc_id
                            
                            key = (duct_id, poc_id)
                            if key not in duct_pocs_dict:
                                duct_info = duct_info_map.get(duct_guid, {})
                                
                                duct_pocs_dict[key] = {
                                    'eq_maker': eq_maker,
                                    'duct_id': duct_id,
                                    'utility': utility,
                                    'poc_id': poc_id,
                                    'pos': pos,
                                    'bay': node.get('bay', 'N/A'),
                                    'center_z': duct_info.get('center_z'),
                                    'h1': duct_info.get('h1', 0.0),
                                    'h2': duct_info.get('h2', 0.0),
                                    'level': duct_info.get('level', 'N/A')
                                }
        
        duct_pocs = list(duct_pocs_dict.values())
        
        if duct_pocs:
            print(f"   [Step 2] 덕트에 연결된 고유 PoC {len(duct_pocs)}개 추출 완료")
        else:
            print(f"   [Step 2] 덕트 연결 PoC를 찾지 못했습니다.")
                            
        duct_groups = defaultdict(list)
        for dp in duct_pocs:
            duct_groups[dp['duct_id']].append(dp)
            
        for duct_id, pocs in duct_groups.items():
            eq_names = ", ".join(sorted(duct_equipments[duct_id])) if duct_equipments[duct_id] else "Unknown"
            total_poc_count = len(pocs)
            
            # 1차 그룹: 유틸리티별
            util_groups = defaultdict(list)
            for p in pocs:
                util_groups[p['utility']].append(p)
                
            # 2차 그룹: 3D 거리 기반 클러스터링 (최대 허용 이격 거리 600mm)
            clusters = []
            MAX_SPACING = 600.0
            for utility, util_pocs in util_groups.items():
                if not util_pocs:
                    continue
                    
                visited = [False] * len(util_pocs)
                for i in range(len(util_pocs)):
                    if visited[i]:
                        continue
                        
                    # BFS/DFS 연결 노드 탐색
                    cur_cluster = [util_pocs[i]]
                    visited[i] = True
                    queue = [util_pocs[i]]
                    
                    while queue:
                        curr_p = queue.pop(0)
                        for j in range(len(util_pocs)):
                            if not visited[j]:
                                dist = calc_dist(curr_p['pos'], util_pocs[j]['pos'])
                                if dist <= MAX_SPACING:
                                    visited[j] = True
                                    cur_cluster.append(util_pocs[j])
                                    queue.append(util_pocs[j])
                                    
                    clusters.append({'utility': utility, 'pocs': cur_cluster})
                
            cluster_count = len(clusters)
            cluster_idx = 1
            for c_info in clusters:
                utility = c_info['utility']
                cpocs = c_info['pocs']
                
                poc_id_list = ", ".join([p['poc_id'] for p in cpocs])
                poc_pos_list = " | ".join([f"({round(p['pos'][0],2)}, {round(p['pos'][1],2)}, {round(p['pos'][2],2)})" for p in cpocs])
                
                # 기존 좌표 기준 바운딩 박스
                xs = [p['pos'][0] for p in cpocs]
                ys = [p['pos'][1] for p in cpocs]
                zs = [p['pos'][2] for p in cpocs]
                
                b_min_x, b_max_x = min(xs), max(xs)
                b_min_y, b_max_y = min(ys), max(ys)
                b_min_z, b_max_z = min(zs), max(zs)
                
                bound_width = round(b_max_x - b_min_x, 2)
                bound_depth = round(b_max_y - b_min_y, 2)
                bound_height = round(b_max_z - b_min_z, 2)
                
                # ──────────────────────────────────────────────────────────────────────
                # space_min/max 계산 (정렬된 고유 좌표 간격 방식)
                def axis_gaps(vals):
                    unique_sorted = sorted(set(round(v, 1) for v in vals))
                    if len(unique_sorted) < 2:
                        return [0.0]
                    return [unique_sorted[i+1] - unique_sorted[i] 
                            for i in range(len(unique_sorted) - 1)]

                x_gaps = axis_gaps(xs)
                y_gaps = axis_gaps(ys)
                z_gaps = axis_gaps(zs)

                space_min_x = round(min(x_gaps), 2)
                space_max_x = round(max(x_gaps), 2)
                space_min_y = round(min(y_gaps), 2)
                space_max_y = round(max(y_gaps), 2)
                space_min_z = round(min(z_gaps), 2)
                space_max_z = round(max(z_gaps), 2)

                if space_max_z > 25.0:
                    poc_face_val = 'side'
                else:
                    avg_z = sum(zs) / len(zs)
                    center_z = cpocs[0].get('center_z')
                    h1, h2 = cpocs[0].get('h1', 0.0), cpocs[0].get('h2', 0.0)
                    dz = abs(avg_z - center_z) if center_z is not None else 9999.0
                    
                    if abs(dz - h1) <= 30.0 or abs(dz - h2) <= 30.0:
                        poc_face_val = 'top'
                    else:
                        if space_max_x > 20.0 and space_max_y > 20.0:
                            poc_face_val = 'top'
                        else:
                            poc_face_val = 'side'
                
                # 최소 간격 로그 출력
                print(f"   [Step 3] Cluster {cluster_idx} ({utility}): Min Spacing (X:{space_min_x}, Y:{space_min_y}, Z:{space_min_z})")

                records.append({
                    '파일명': file_name,
                    'PROCESS': f_process,
                    'MAKER': cpocs[0].get('eq_maker', ''),
                    'BAY': cpocs[0].get('bay', f_bay),
                    'LEVEL': cpocs[0].get('level', 'N/A'),
                    '장비명': eq_names,
                    '덕트ID': duct_id,
                    '유틸리티': utility,
                    '전체PoC수량': total_poc_count,
                    '클러스터링수량': cluster_count,
                    '클러스터링ID': f"C{cluster_idx}",
                    'space_min_x': round(space_min_x, 2),
                    'space_max_x': round(space_max_x, 2),
                    'space_min_y': round(space_min_y, 2),
                    'space_max_y': round(space_max_y, 2),
                    'space_min_z': round(space_min_z, 2),
                    'space_max_z': round(space_max_z, 2),
                    'BoundBox최소좌표': f"({round(b_min_x,2)}, {round(b_min_y,2)}, {round(b_min_z,2)})",
                    'BoundBox최대좌표': f"({round(b_max_x,2)}, {round(b_max_y,2)}, {round(b_max_z,2)})",
                    'Bound너비(X)': bound_width,
                    'Bound깊이(Y)': bound_depth,
                    'Bound높이(Z)': bound_height,
                    'PoCID리스트': poc_id_list,
                    'duct_face': poc_face_val,
                    'PoC좌표': poc_pos_list
                })
                cluster_idx += 1
                
    return records

def export_results(records, output_prefix):
    if not records:
        print("데이터 없음.")
        return

    csv_fn = f"{output_prefix}.csv"
    xlsx_fn = f"{output_prefix}.xlsx"
    
    fieldnames = [
        '파일명', 'PROCESS', 'MAKER', 'BAY', 'LEVEL', '장비명', '덕트ID', '유틸리티', 
        '전체PoC수량', '클러스터링수량', '클러스터링ID',
        'space_min_x', 'space_max_x', 'space_min_y', 'space_max_y', 'space_min_z', 'space_max_z',
        'BoundBox최소좌표', 'BoundBox최대좌표', 'Bound너비(X)', 'Bound깊이(Y)', 'Bound높이(Z)',
        'PoCID리스트', 'duct_face', 'PoC좌표'
    ]
    
    try:
        with open(csv_fn, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)
        print(f"CSV 완료: {csv_fn}")
    except Exception as e: print(f"CSV 오류: {e}")

    try:
        df = pd.DataFrame(records, columns=fieldnames)
        df.to_excel(xlsx_fn, index=False)
        print(f"Excel 완료: {xlsx_fn}")
    except Exception as e: print(f"Excel 오류: {e}")

def main():
    if len(sys.argv) < 2:
        print("사용법: python duct_poc_clustering.py <폴더>")
        return
    
    target_dir = sys.argv[1]
    prefix = f"duct_poc_cluster_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    records = extract_duct_poc_analysis(target_dir)
    export_results(records, prefix)

if __name__ == "__main__":
    main()
