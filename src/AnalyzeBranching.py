"""
[실행 명령어]
python AnalyzeBranching.py --input ./data-v10 --output ./BranchAnalysisResults

[프로그램 개요]
본 프로그램은 배관 설계 데이터(JSON)를 분석하여 장비(Equipment) 및 유틸리티(Utility)별로
배관 네트워크 내의 분기점(Tee, Cross, Branch 등)의 현황을 파악하고 정밀한 통계를 산출합니다.
모든 결과물에는 장비의 프로세스(Process), 제조사(Maker), 이름(Name), ID 및 설치 공간(Space) 정보가 포함됩니다.
또한 각 분기점에서 연결된 실제 터미널 노드(PoC, Takeoff, Nozzle 등)의 ID를 추적하여 제공합니다.

[전체적인 기능 흐름도]
1. 초기화: 명령어 인자를 파싱하고 결과 저장 디렉토리 생성
2. 파일 순회: 지정된 디렉토리 내의 모든 배관 설계 JSON 파일을 검색
3. 데이터 로딩 및 인덱싱 (BranchGraph.load_from_json): 
   - 노드(Nodes)와 엣지(Edges)를 메모리에 로드
   - 엣지 정보를 바탕으로 인접 리스트(Adjacency List) 구축
   - 장비의 PoC(Connection Points)를 노드 인덱스에 통합
4. 분기 탐색 (BranchGraph.analyze_branches):
   - 각 장비의 PoC를 시작점으로 설정
   - BFS(너비 우선 탐색) 알고리즘을 사용하여 해당 유틸리티 라인의 모든 연결 경로 추적
   - 각 노드 방문 시 분기점 판단 로직 수행
5. 분기점 및 터미널 추적 알고리즘:
   - 분기 판단: 노드의 'connectionGuidList' 속성 크기가 3개 이상인지 확인 (Tee: 3, Cross: 4 등)
   - 터미널 추적: 분기점에서 각 방향으로 배관을 따라가며 끝점(PoC, Takeoff, Nozzle 등) 탐색
6. 결과 집계 및 리포트 생성:
   - 중복 데이터 제거 및 장비/유틸리티별 그룹화
   - 상세 정보를 포함한 JSON 리포트 및 요약용 CSV 리포트 저장
"""

import json
import math
import os
import glob
import csv
import argparse
from collections import deque, defaultdict
from datetime import datetime

# ─────────────────────────────────────────────────────────────
# 전역 설정 및 상수
# ─────────────────────────────────────────────────────────────
DEFAULT_INPUT_DIR = "./data-v10"       # 기본 설계 데이터 입력 폴더
DEFAULT_OUTPUT_DIR = "./BranchAnalysisResults" # 분석 결과 저장 폴더

def _get_guid(obj):
    """
    객체에서 고유 식별자(GUID)를 추출하는 유틸리티 함수입니다.
    우선순위: guid -> id -> GUID 순서로 확인합니다.
    """
    return obj.get("guid") or obj.get("id") or obj.get("GUID")

def _get_human_id(obj):
    """
    객체에서 사람이 읽기 좋은 ID(예: POC0001, TEE02834)를 추출합니다.
    일부 객체는 'id'가 GUID이고 'name'이 실제 ID인 경우가 있으므로 이를 구분하여 처리합니다.
    """
    if not obj: return None
    hid = obj.get("id")
    name = obj.get("name")
    
    # 만약 hid가 GUID 형식(길고 하이픈 포함)이라면 name을 우선 사용
    if hid and len(hid) > 30 and "-" in hid:
        return name or hid
    return hid or name

def _get_position(node):
    """
    노드의 3D 좌표 정보를 추출합니다.
    다양한 JSON 포맷(리스트 방식 [x,y,z] 또는 딕셔너리 방식 {"x":..}) 및 필드명을 모두 대응합니다.
    """
    p = node.get("position") or node.get("pocPosition") or node.get("nodePosition")
    if isinstance(p, list) and len(p) >= 3:
        return [float(p[0]), float(p[1]), float(p[2])]
    if isinstance(p, dict):
        return [float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("z", 0.0))]
    return [0.0, 0.0, 0.0]

# ─────────────────────────────────────────────────────────────
# 1. BranchGraph (네트워크 토폴로지 분석 클래스)
# ─────────────────────────────────────────────────────────────

class BranchGraph:
    """
    배관 네트워크의 위상(Topology)을 분석하여 분기점과 터미널을 탐색하는 핵심 클래스입니다.
    """
    def __init__(self):
        self.node_by_guid = {}    # GUID를 키로 하는 노드 정보 테이블
        self.edge_by_guid = {}    # GUID를 키로 하는 엣지(배관) 정보 테이블
        self.adj = defaultdict(list) # 인접 리스트: 특정 노드 GUID -> 연결된 [(엣지, 이웃노드GUID), ...]
        self.equipment_list = []  # 파일 내 장비 목록

    def load_from_json(self, file_path):
        """설계 JSON 파일을 읽어 분석에 필요한 그래프 구조를 메모리에 구축합니다."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            nodes = data.get("Nodes", data.get("nodes", []))
            edges = data.get("Edges", data.get("edges", []))
            equips = data.get("Equipment", data.get("equipment", []))
            
            for n in nodes:
                g = _get_guid(n)
                if not g: continue
                self.node_by_guid[g] = n
                
                # 노드 자체의 연결 가이드 리스트를 사용하여 인접 관계 추가
                conn = n.get("connectionGuidList", [])
                for neighbor_guid in conn:
                    # 중복 방지를 위해 이미 동일한 이웃이 있는지 확인 (엣지명이 없을 경우 가상 엣지 생성)
                    existing_neighbors = [ng for ed, ng in self.adj[g]]
                    if neighbor_guid not in existing_neighbors:
                        # 가상 엣지 객체 생성 (기존 엣지 데이터가 없을 경우를 대비)
                        virtual_edge = {"id": "V-EDGE", "name": "VirtualEdge", "connectionGuidList": [g, neighbor_guid]}
                        self.adj[g].append((virtual_edge, neighbor_guid))
                        self.adj[neighbor_guid].append((virtual_edge, g))
            
            for e in edges:
                eg = _get_guid(e)
                if not eg: continue
                self.edge_by_guid[eg] = e
                # 엣지도 노드 맵에 추가하여 추적 시 ID를 찾을 수 있게 함
                self.node_by_guid[eg] = e
                
                conn = e.get("connectionGuidList", [])
                if len(conn) >= 2:
                    u, v = conn[0], conn[1]
                    # 중복 방지를 위해 이미 동일한 이웃이 있는지 확인
                    if v not in [ng for ed, ng in self.adj[u]]:
                        self.adj[u].append((e, v))
                    if u not in [ng for ed, ng in self.adj[v]]:
                        self.adj[v].append((e, u))
            
            self.equipment_list = [equips] if isinstance(equips, dict) else equips
            for eq in self.equipment_list:
                # 'pocList'와 'PocList' 두 가지 케이스 모두 대응
                p_list = eq.get("pocList", []) or eq.get("PocList", [])
                for p in p_list:
                    pg = _get_guid(p)
                    if pg: self.node_by_guid[pg] = p
            return True
        except Exception as e:
            print(f"파일 로딩 오류 ({file_path}): {e}")
            return False

    def get_neighbors(self, node_guid):
        """인접 리스트에서 특정 노드와 직접 연결된 이웃 노드 목록을 반환합니다."""
        return self.adj.get(node_guid, [])

    def is_terminal_node(self, node_guid, all_poc_guids):
        """해당 노드가 터미널(끝점: PoC, Takeoff, Nozzle 등)인지 판단합니다."""
        node = self.node_by_guid.get(node_guid)
        if not node: return False
        
        # PoC 리스트 포함 여부
        if node_guid in all_poc_guids: return True
        
        # 특정 터미널 타입 키워드 확인
        ntype = (node.get("type") or "").upper()
        if ntype in ["TAKEOFF", "NOZZLE", "TERMINAL", "POC", "EQUIPMENT"]: return True
        
        # 연결된 엣지가 1개인 말단 노드
        conn_list = node.get("connectionGuidList", [])
        if len(conn_list) == 1: return True
        
        return False

    def trace_to_terminal(self, start_guid, first_neighbor_guid, all_poc_guids):
        """
        특정 방향으로 배관을 따라가며 가장 처음 만나는 터미널 노드 또는 다른 분기점의 ID를 반환합니다.
        """
        curr_node_guid = first_neighbor_guid
        visited = {start_guid, first_neighbor_guid}
        
        # 최대 200단계까지만 추적 (무한루프 방지)
        for _ in range(200):
            node = self.node_by_guid.get(curr_node_guid)
            if not node: break
            
            # 터미널이거나 다른 분기점(연결 3개 이상)이면 반환
            conn_list = node.get("connectionGuidList", [])
            if self.is_terminal_node(curr_node_guid, all_poc_guids) or len(conn_list) >= 3:
                return _get_human_id(node) or curr_node_guid
                
            # 다음 경로 노드로 이동 (연결이 2개인 노드)
            neighbors = self.get_neighbors(curr_node_guid)
            next_step = None
            for _, neigh_guid in neighbors:
                if neigh_guid not in visited:
                    next_step = neigh_guid
                    break
            
            if not next_step: break
            visited.add(next_step)
            curr_node_guid = next_step
            
        last_node = self.node_by_guid.get(curr_node_guid)
        return _get_human_id(last_node) or curr_node_guid

    def get_branch_info(self, node_guid, all_poc_guids):
        """노드의 분기 여부를 판단하고 갈래별 터미널 노드 정보를 포함하여 반환합니다."""
        node = self.node_by_guid.get(node_guid)
        if not node: return None
        
        conn_list = node.get("connectionGuidList", [])
        prop_conn_count = len(conn_list)
        
        if prop_conn_count >= 3:
            ntype = (node.get("type") or "").upper()
            
            # 노드 자체의 연결 포트(connectionGuidList)를 기준으로 터미널 추적
            terminal_ids = []
            for neigh_guid in conn_list:
                # 해당 이웃 노드로 연결되는 경로 추적
                t_id = self.trace_to_terminal(node_guid, neigh_guid, all_poc_guids)
                if t_id: terminal_ids.append(t_id)

            return {
                "id": _get_human_id(node) or node_guid,
                "type": ntype if ntype else "BRANCH_NODE",
                "name": node.get("name", "Unknown"),
                "z": _get_position(node)[2],
                "space": node.get("level") or node.get("Level") or "-",
                "conn_count": prop_conn_count,
                "terminals": terminal_ids  # 갈래별 터미널 노드 ID 리스트
            }
        return None

    def analyze_branches(self):
        """모든 장비의 PoC에서 출발하여 해당 라인의 분기 객체를 식별합니다."""
        all_equipment_pocs = set()
        for eq in self.equipment_list:
            p_list = eq.get("pocList", []) or eq.get("PocList", [])
            for p in p_list:
                g = _get_guid(p)
                if g: all_equipment_pocs.add(g)

        analysis_results = defaultdict(dict)

        for eq in self.equipment_list:
            eq_name = eq.get("name") or eq.get("Name") or "Unknown"
            eq_id = _get_guid(eq) or "UnknownID"
            eq_process = eq.get("process") or eq.get("Process") or "-"
            eq_maker = eq.get("maker") or eq.get("Maker") or "-"
            
            p_list = eq.get("pocList", []) or eq.get("PocList", [])
            for poc in p_list:
                poc_guid = _get_guid(poc)
                if not poc_guid or poc_guid not in self.node_by_guid: continue
                
                util = poc.get("utility") or poc.get("Utility") or "-"
                key = (eq_name, eq_id, eq_process, eq_maker, util)
                
                queue = deque([(poc_guid, frozenset())]) 
                visited_nodes = {poc_guid}
                
                while queue:
                    curr_g, v_edges = queue.popleft()
                    
                    b_info = self.get_branch_info(curr_g, all_equipment_pocs)
                    if b_info:
                        analysis_results[key][curr_g] = b_info
                    
                    for edge, next_g in self.get_neighbors(curr_g):
                        e_sig = _get_guid(edge)
                        if e_sig in v_edges or next_g in visited_nodes: continue
                        if next_g in all_equipment_pocs and next_g != poc_guid: continue
                        
                        visited_nodes.add(next_g)
                        queue.append((next_g, v_edges | {e_sig}))
        return analysis_results

# ─────────────────────────────────────────────────────────────
# 2. Main (실행 및 리포트 자동 생성)
# ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="배관 분기 객체 분석 및 터미널 추적 도구")
    parser.add_argument("--input", default=DEFAULT_INPUT_DIR)
    parser.add_argument("--output", default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    json_files = glob.glob(os.path.join(args.input, "**", "*.json"), recursive=True)
    print(f"[Branch Analysis] 총 {len(json_files)}개 설계 파일 분석 시작...")

    total_stats = defaultdict(list)

    for fpath in json_files:
        graph = BranchGraph()
        if not graph.load_from_json(fpath): continue
        file_results = graph.analyze_branches()
        for k, branch_dict in file_results.items():
            total_stats[k].extend(list(branch_dict.values()))

    ts = datetime.now().strftime("%Y%m%d%H%M%S")
    json_path = os.path.join(args.output, f"branch_analysis_{ts}.json")
    csv_path = os.path.join(args.output, f"branch_analysis_{ts}.csv")

    json_out = []
    csv_rows = []

    for (eq, eid, prc, mkr, util), branches in total_stats.items():
        unique_branches = {b["id"]: b for b in branches}.values()
        type_counts = defaultdict(int)
        for b in unique_branches: type_counts[b["type"]] += 1
        
        json_out.append({
            "equipment_name": eq, "equipment_id": eid, 
            "equipment_process": prc, "equipment_maker": mkr,
            "utility": util, "total_branch_count": len(unique_branches),
            "type_summary": dict(type_counts),
            "branches": sorted(list(unique_branches), key=lambda x: (x["space"], x["z"]))
        })
        
        for b in unique_branches:
            csv_rows.append({
                "EQUIP_NAME": eq, "EQUIP_ID": eid, "EQUIP_PROCESS": prc, "EQUIP_MAKER": mkr,
                "UTILITY": util, "SPACE": b["space"], "BRANCH_TYPE": b["type"],
                "BRANCH_NAME": b["name"], "NODE_ID": b["id"], "HEIGHT_Z": round(b["z"], 1),
                "CONN_COUNT": b["conn_count"],
                "TERMINAL_NODES": ", ".join(b.get("terminals", []))
            })

    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(json_out, f, indent=2, ensure_ascii=False)
    
    csv_headers = ["EQUIP_NAME", "EQUIP_ID", "EQUIP_PROCESS", "EQUIP_MAKER", "UTILITY", "SPACE", "BRANCH_TYPE", "BRANCH_NAME", "NODE_ID", "HEIGHT_Z", "CONN_COUNT", "TERMINAL_NODES"]
    with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=csv_headers)
        writer.writeheader()
        writer.writerows(csv_rows)

    print(f"분석 완료! 결과: {csv_path}")

if __name__ == "__main__":
    main()
