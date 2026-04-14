# -*- coding: utf-8 -*-
# =====================================================================================
# [실행 명령어]
#   python import_obstacles_json.py ./data-v10 --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432
#   python import_obstacles_json.py ./data-v10 --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432 --clean
#   python import_obstacles_json.py ./data-v11 --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432 --clean
# =====================================================================================
"""
import_obstacles_json.py  — BIM 데이터 통합 임포트 시스템 (v3.0)
================================================================

[프로그램 개요]
  BIM 설계 JSON 파일로부터 장애물(Obstacles), 공간 레벨(SpaceInfo), 장비(Equipment),
  덕트/라테랄(Duct/Lateral) 정보를 통합 추출하여 PostgreSQL(PostGIS)에 적재합니다.

[전체 흐름도]
  ┌──────────────────────────────────────────────────────────────┐
  │  1. JSON 디렉토리 내 *.json 파일 검색                        │
  │  2. 데이터 추출 (extract_all_data_from_json):                │
  │     ├─ Obstacles: 장애물 BBox + ddworksType 분류             │
  │     ├─ SpaceInfo: 공간 레벨(CSF/A/F/CR) BBox                │
  │     ├─ Equipment: 장비 BBox + POC 좌표 (MultiPointZ)        │
  │     └─ Duct/Lateral: 덕트·라테랄 BBox + 유틸리티            │
  │  3. CSV 백업 파일 생성                                       │
  │  4. PostgreSQL 테이블 스키마 검증/생성:                      │
  │     ├─ TB_BIM_OBSTACLES (box3d)                              │
  │     ├─ TB_BIM_SPACE_INFO (box3d)                             │
  │     ├─ TB_BIM_EQUIPMENT (box3d + MultiPointZ)                │
  │     └─ TB_DUCT_LATERAL (box3d)                               │
  │  5. 기존 동일 파일 데이터 DELETE + 신규 INSERT               │
  │  6. 통계 보고 출력                                           │
  └──────────────────────────────────────────────────────────────┘

[주요 함수]
  - get_obstacle_schema_sql()      : TB_BIM_OBSTACLES CREATE 쿼리
  - get_space_schema_sql()         : TB_BIM_SPACE_INFO CREATE 쿼리
  - get_equipment_schema_sql()     : TB_BIM_EQUIPMENT CREATE 쿼리 (POC_GEOM 포함)
  - get_duct_lateral_schema_sql()  : TB_DUCT_LATERAL CREATE 쿼리
  - extract_all_data_from_json()   : JSON에서 4종 데이터 통합 추출
  - run_integration_import()       : CSV 생성 + DB 적재 전체 파이프라인

[주요 변수]
  - OBSTACLE_TABLE                 : "TB_BIM_OBSTACLES"
  - SPACE_TABLE                    : "TB_BIM_SPACE_INFO"
  - EQUIPMENT_TABLE                : "TB_BIM_EQUIPMENT"
  - DUCT_LATERAL_TABLE             : "TB_DUCT_LATERAL"
"""

import json
import os
import argparse
import csv
import psycopg2
from psycopg2 import extras
from collections import Counter

# 데이터베이스 테이블 이름 정의
OBSTACLE_TABLE = "TB_BIM_OBSTACLES"
SPACE_TABLE = "TB_BIM_SPACE_INFO"
EQUIPMENT_TABLE = "TB_BIM_EQUIPMENT"
DUCT_LATERAL_TABLE = "TB_DUCT_LATERAL" # 신규 추가

def get_obstacle_schema_sql():
    return f"""
    CREATE TABLE IF NOT EXISTS "{OBSTACLE_TABLE}" (
        "ID" SERIAL PRIMARY KEY,
        "SOURCE_FILE" TEXT,
        "MAIN_EQ_NAME" TEXT,
        "PROCESS" TEXT,
        "MAKER" TEXT,
        "OBJECT_ID" TEXT,
        "NAME" TEXT,
        "DDWORKS_TYPE" TEXT,
        "OST_TYPE" TEXT,
        "MIN_X" FLOAT, "MIN_Y" FLOAT, "MIN_Z" FLOAT,
        "MAX_X" FLOAT, "MAX_Y" FLOAT, "MAX_Z" FLOAT,
        "BBOX" box3d,
        "LENGTH_X" FLOAT, "LENGTH_Y" FLOAT, "LENGTH_Z" FLOAT,
        "POS_X" FLOAT, "POS_Y" FLOAT, "POS_Z" FLOAT,
        "IMPORT_TIME" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

def get_space_schema_sql():
    return f"""
    CREATE TABLE IF NOT EXISTS "{SPACE_TABLE}" (
        "ID" SERIAL PRIMARY KEY,
        "SOURCE_FILE" TEXT,
        "MAIN_EQ_NAME" TEXT,
        "PROCESS" TEXT,
        "MAKER" TEXT,
        "LEVEL_NAME" TEXT,
        "MIN_X" FLOAT, "MIN_Y" FLOAT, "MIN_Z" FLOAT,
        "MAX_X" FLOAT, "MAX_Y" FLOAT, "MAX_Z" FLOAT,
        "SIZE_X" FLOAT, "SIZE_Y" FLOAT, "SIZE_Z" FLOAT,
        "LENGTH_X" FLOAT, "LENGTH_Y" FLOAT, "LENGTH_Z" FLOAT,
        "BBOX" box3d,
        "IMPORT_TIME" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

def get_equipment_schema_sql():
    return f"""
    CREATE TABLE IF NOT EXISTS "{EQUIPMENT_TABLE}" (
        "ID" SERIAL PRIMARY KEY,
        "SOURCE_FILE" TEXT,
        "EQ_ID" TEXT,
        "NAME" TEXT,
        "TYPE" TEXT,
        "PROCESS" TEXT,
        "MAKER" TEXT,
        "POC_TOTAL_COUNT" INTEGER,
        "UTILITY_POC_INFO" JSONB,
        "POC_LIST" JSONB,
        "MIN_X" FLOAT, "MIN_Y" FLOAT, "MIN_Z" FLOAT,
        "MAX_X" FLOAT, "MAX_Y" FLOAT, "MAX_Z" FLOAT,
        "LENGTH_X" FLOAT, "LENGTH_Y" FLOAT, "LENGTH_Z" FLOAT,
        "BBOX" box3d,
        "POC_GEOM" GEOMETRY(MultiPointZ),
        "IS_MAIN" BOOLEAN DEFAULT FALSE,
        "IMPORT_TIME" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

def get_duct_lateral_schema_sql():
    return f"""
    CREATE TABLE IF NOT EXISTS "{DUCT_LATERAL_TABLE}" (
        "ID" SERIAL PRIMARY KEY,
        "SOURCE_FILE" TEXT,
        "MAIN_EQ_NAME" TEXT,
        "PROCESS" TEXT,
        "OBJECT_ID" TEXT,
        "NAME" TEXT,
        "TYPE" TEXT,
        "CATEGORY" TEXT, -- DUCT 또는 LATERAL
        "LEVEL" TEXT,
        "UTILITY" TEXT,
        "MATERIAL" TEXT,
        "SIZE" TEXT,
        "MIN_X" FLOAT, "MIN_Y" FLOAT, "MIN_Z" FLOAT,
        "MAX_X" FLOAT, "MAX_Y" FLOAT, "MAX_Z" FLOAT,
        "LENGTH_X" FLOAT, "LENGTH_Y" FLOAT, "LENGTH_Z" FLOAT,
        "POS_X" FLOAT, "POS_Y" FLOAT, "POS_Z" FLOAT,
        "BBOX" box3d,
        "IMPORT_TIME" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

def validate_and_recreate_table(cur, table_name, expected_cols):
    cur.execute(f"SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = '{table_name}');")
    if not cur.fetchone()[0]:
        return
    cur.execute(f"SELECT column_name FROM information_schema.columns WHERE table_name = '{table_name}';")
    current_cols = set(row[0] for row in cur.fetchall())
    if not expected_cols.issubset(current_cols):
        print(f"[Schema] '{table_name}' 구조 변경 감지. 재구성합니다.")
        cur.execute(f'DROP TABLE IF EXISTS "{table_name}" CASCADE;')

def extract_all_data_from_json(json_path):
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        print(f"[Error] JSON 파싱 실패 ({json_path}): {e}")
        return None

    file_info = data.get("FileInfo", {})
    source_file = os.path.basename(json_path)
    
    equipment_raw = data.get("Equipment", [])
    eq_list = []
    main_eq = equipment_raw[0] if equipment_raw else {}
    main_eq_name = main_eq.get("name", "N/A")
    main_process = main_eq.get("process", "N/A")
    main_maker = main_eq.get("maker", "N/A")

    # CR(Clean Room) 레벨의 Z 범위 추출 → Main장비 판단 기준
    cr_min_z, cr_max_z = None, None
    spaces_raw_for_cr = file_info.get("SpaceInfo", [])
    for sp in spaces_raw_for_cr:
        level_name = (sp.get("levelName") or "").strip().upper()
        if level_name == "CR":
            boundary = sp.get("boundary", {})
            cr_mn = boundary.get("min", {})
            cr_mx = boundary.get("max", {})
            cr_min_z = cr_mn.get("z", 0)
            cr_max_z = cr_mx.get("z", 0)
            break

    # 1. 장비 (Equipment) - 개별 장비의 boundaryBox 사용
    for eq in equipment_raw:
        pocs = eq.get("pocList", [])
        pts = [f"{p.get('pocPosition')[0]} {p.get('pocPosition')[1]} {p.get('pocPosition')[2]}"
               for p in pocs if p.get("pocPosition")]
        poc_geom_wkt = f"MULTIPOINT Z({', '.join(pts)})" if pts else None
        # 장비 개별 boundaryBox 사용 (없으면 GroupBoundaryBox 폴백)
        bbox_raw = eq.get("boundaryBox") or eq.get("boundary") or file_info.get("GroupBoundaryBox", {})
        mn = bbox_raw.get("min", {"x":0,"y":0,"z":0})
        mx = bbox_raw.get("max", {"x":0,"y":0,"z":0})

        mnx, mny, mnz = mn.get("x", 0), mn.get("y", 0), mn.get("z", 0)
        mxx, mxy, mxz = mx.get("x", 0), mx.get("y", 0), mx.get("z", 0)

        # Main장비 판단: 장비의 Z 범위가 CR 레벨과 겹치면 Main장비
        is_main = False
        if cr_min_z is not None and cr_max_z is not None:
            is_main = (mxz >= cr_min_z and mnz <= cr_max_z)

        eq_list.append({
            "source_file": source_file, "eq_id": eq.get("id"), "name": eq.get("name"),
            "type": eq.get("type"), "process": eq.get("process"), "maker": eq.get("maker"),
            "poc_total_count": eq.get("totalPocCount", 0), "utility_poc_info": json.dumps(eq.get("utilityPocCount", {})),
            "poc_list": json.dumps(eq.get("pocList", [])),
            "min_x": mnx, "min_y": mny, "min_z": mnz,
            "max_x": mxx, "max_y": mxy, "max_z": mxz,
            "length_x": abs(mxx - mnx), "length_y": abs(mxy - mny), "length_z": abs(mxz - mnz),
            "poc_geom_wkt": poc_geom_wkt,
            "is_main": is_main
        })

    # 2. 장애물 (Obstacles)
    obstacles_raw = data.get("Obstacles", [])
    obs_list = []
    for obs in obstacles_raw:
        boundary = obs.get("boundary", {})
        mn, mx = boundary.get("min", {}), boundary.get("max", {})
        length, pos = obs.get("length", {}), obs.get("position", [0,0,0])
        obs_list.append({
            "source_file": source_file, "main_eq_name": main_eq_name, "process": main_process, "maker": main_maker,
            "object_id": obs.get("obstacleId"), "name": obs.get("name"), 
            "ddworks_type": obs.get("ddworksType"), "ost_type": obs.get("ostType"),
            "min_x": mn.get("x",0), "min_y": mn.get("y",0), "min_z": mn.get("z",0),
            "max_x": mx.get("x",0), "max_y": mx.get("y",0), "max_z": mx.get("z",0),
            "length_x": length.get("x",0), "length_y": length.get("y",0), "length_z": length.get("z",0),
            "pos_x": pos[0], "pos_y": pos[1], "pos_z": pos[2]
        })

    # 3. 공간 (Space Info)
    spaces_raw = file_info.get("SpaceInfo", [])
    space_list = []
    for sp in spaces_raw:
        level_name = sp.get("levelName")
        boundary = sp.get("boundary", {})
        mn, mx = boundary.get("min", {}), boundary.get("max", {})
        mnx, mny, mnz = mn.get("x",0), mn.get("y",0), mn.get("z",0)
        mxx, mxy, mxz = mx.get("x",0), mx.get("y",0), mx.get("z",0)
        space_list.append({
            "source_file": source_file, "main_eq_name": main_eq_name, "process": main_process, "maker": main_maker,
            "level_name": level_name, "min_x": mnx, "min_y": mny, "min_z": mnz, "max_x": mxx, "max_y": mxy, "max_z": mxz,
            "size_x": abs(mxx-mnx), "size_y": abs(mxy-mny), "size_z": abs(mxz-mnz),
            "length_x": abs(mxx-mnx), "length_y": abs(mxy-mny), "length_z": abs(mxz-mnz)
        })

    # 4. 덕트 및 라테랄 (Duct & Lateral)
    dl_list = []

    # 4-1. Equipment.ends에서 DUCT/LATERAL/BRANCH PIPE만 추출 (EQUIPMENT, ETC 등 제외)
    VALID_DL_TYPES = {"DUCT", "LATERAL", "BRANCH PIPE", "LATERAL PIPE"}
    for eq in equipment_raw:
        for end in eq.get("ends", []):
            end_type = str(end.get("type", "")).strip().upper()
            if end_type not in VALID_DL_TYPES:
                continue
            bbox = end.get("boundaryBox", end.get("boundary", {}))
            mn = bbox.get("min", {})
            mx = bbox.get("max", {})
            if not mn or not mx:
                continue
            # 카테고리 결정: BRANCH PIPE → BRANCH PIPE, LATERAL/LATERAL PIPE → LATERAL, 나머지 → DUCT
            if "BRANCH" in end_type:
                category = "BRANCH PIPE"
            elif "LATERAL" in end_type:
                category = "LATERAL"
            else:
                category = "DUCT"
            npos = end.get("nodePosition", [0, 0, 0])
            mnx, mny, mnz = mn.get("x", 0), mn.get("y", 0), mn.get("z", 0)
            mxx, mxy, mxz = mx.get("x", 0), mx.get("y", 0), mx.get("z", 0)
            dl_list.append({
                "source_file": source_file, "main_eq_name": main_eq_name, "process": main_process,
                "object_id": end.get("id"), "name": end.get("name"), "type": end.get("type"),
                "category": category, "level": "N/A",
                "utility": "N/A", "material": "N/A", "size": "N/A",
                "min_x": mnx, "min_y": mny, "min_z": mnz,
                "max_x": mxx, "max_y": mxy, "max_z": mxz,
                "length_x": abs(mxx - mnx), "length_y": abs(mxy - mny), "length_z": abs(mxz - mnz),
                "pos_x": npos[0] if isinstance(npos, list) else 0,
                "pos_y": npos[1] if isinstance(npos, list) else 0,
                "pos_z": npos[2] if isinstance(npos, list) else 0
            })

    # 4-2. Nodes/Edges에서 DUCT/LATERAL/BRANCH PIPE 부속품 추출
    def process_node_edge_list(items, is_edge=False):
        for item in items:
            iid = str(item.get("id", "")).upper()
            itmp_type = str(item.get("type", "")).upper()
            iname = str(item.get("name", "")).upper()
            combined = f"{iid} {itmp_type} {iname}"

            category = None
            if "BRANCH" in combined and "PIPE" in combined:
                category = "BRANCH PIPE"
            elif "LATERAL" in combined:
                category = "LATERAL"
            elif "DUCT" in combined:
                category = "DUCT"

            if category:
                pos = item.get("position", [0,0,0])
                bbox = item.get("boundaryBox", item.get("boundary", {}))
                mn = bbox.get("min", {})
                mx = bbox.get("max", {})

                mnx = mn.get("x", pos[0]); mny = mn.get("y", pos[1]); mnz = mn.get("z", pos[2])
                mxx = mx.get("x", pos[0]); mxy = mx.get("y", pos[1]); mxz = mx.get("z", pos[2])
                dl_list.append({
                    "source_file": source_file, "main_eq_name": main_eq_name, "process": main_process,
                    "object_id": item.get("id"), "name": item.get("name"), "type": item.get("type"),
                    "category": category, "level": item.get("level", "N/A"),
                    "utility": item.get("utility", "N/A"), "material": item.get("material", "N/A"), "size": item.get("size", "N/A"),
                    "min_x": mnx, "min_y": mny, "min_z": mnz,
                    "max_x": mxx, "max_y": mxy, "max_z": mxz,
                    "length_x": abs(mxx - mnx), "length_y": abs(mxy - mny), "length_z": abs(mxz - mnz),
                    "pos_x": pos[0], "pos_y": pos[1], "pos_z": pos[2]
                })

    process_node_edge_list(data.get("Nodes", []))
    process_node_edge_list(data.get("Edges", []), is_edge=True)

    return {"equipment": eq_list, "obstacles": obs_list, "spaces": space_list, "duct_lateral": dl_list}

def save_to_csv(data_list, filename):
    if not data_list: return
    keys = data_list[0].keys()
    with open(filename, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(data_list)
    print(f"[CSV Sink] 파일 저장 완료: {filename}")

def run_integration_import(json_directory, db_params, clean=False):
    try:
        json_files = sorted([f for f in os.listdir(json_directory) if f.endswith(".json")])
        all_eq, all_obs, all_space, all_dl = [], [], [], []
        
        for fname in json_files:
            res = extract_all_data_from_json(os.path.join(json_directory, fname))
            if res:
                all_eq.extend(res["equipment"]); all_obs.extend(res["obstacles"])
                all_space.extend(res["spaces"]); all_dl.extend(res["duct_lateral"])

        # CSV 백업
        export_dir = "./export"
        if not os.path.exists(export_dir): os.makedirs(export_dir)
        save_to_csv(all_eq, os.path.join(export_dir, "eq_data.csv"))
        save_to_csv(all_obs, os.path.join(export_dir, "obs_data.csv"))
        save_to_csv(all_space, os.path.join(export_dir, "space_data.csv"))
        save_to_csv(all_dl, os.path.join(export_dir, "duct_lateral_data.csv"))

        # DB 처리
        conn = psycopg2.connect(**db_params); cur = conn.cursor()
        cur.execute("CREATE EXTENSION IF NOT EXISTS postgis;")
        
        # 테이블 검증
        validate_and_recreate_table(cur, EQUIPMENT_TABLE, {"EQ_ID", "POC_GEOM", "LENGTH_X", "IS_MAIN"})
        cur.execute(get_equipment_schema_sql())
        validate_and_recreate_table(cur, OBSTACLE_TABLE, {"OBJECT_ID", "BBOX", "LENGTH_X"})
        cur.execute(get_obstacle_schema_sql())
        validate_and_recreate_table(cur, SPACE_TABLE, {"LEVEL_NAME", "BBOX", "LENGTH_X"})
        cur.execute(get_space_schema_sql())
        validate_and_recreate_table(cur, DUCT_LATERAL_TABLE, {"CATEGORY", "BBOX", "LENGTH_X"})
        cur.execute(get_duct_lateral_schema_sql())
        conn.commit()

        # 데이터 대량 삽입
        if clean:
            print("[Clean] 기존 데이터 삭제 후 임포트합니다.")
            cur.execute(f"DELETE FROM \"{EQUIPMENT_TABLE}\"")
            cur.execute(f"DELETE FROM \"{OBSTACLE_TABLE}\"")
            cur.execute(f"DELETE FROM \"{SPACE_TABLE}\"")
            cur.execute(f"DELETE FROM \"{DUCT_LATERAL_TABLE}\"")
            conn.commit()
        else:
            print("[Append] 기존 데이터에 추가합니다.")

        print("Importing to Database...")
        # 1. 장비
        ins_eq = f"INSERT INTO \"{EQUIPMENT_TABLE}\" (\"SOURCE_FILE\", \"EQ_ID\", \"NAME\", \"TYPE\", \"PROCESS\", \"MAKER\", \"POC_TOTAL_COUNT\", \"UTILITY_POC_INFO\", \"POC_LIST\", \"MIN_X\", \"MIN_Y\", \"MIN_Z\", \"MAX_X\", \"MAX_Y\", \"MAX_Z\", \"LENGTH_X\", \"LENGTH_Y\", \"LENGTH_Z\", \"BBOX\", \"POC_GEOM\", \"IS_MAIN\") VALUES (%(source_file)s, %(eq_id)s, %(name)s, %(type)s, %(process)s, %(maker)s, %(poc_total_count)s, %(utility_poc_info)s, %(poc_list)s, %(min_x)s, %(min_y)s, %(min_z)s, %(max_x)s, %(max_y)s, %(max_z)s, %(length_x)s, %(length_y)s, %(length_z)s, ST_3DMakeBox(ST_MakePoint(%(min_x)s, %(min_y)s, %(min_z)s), ST_MakePoint(%(max_x)s, %(max_y)s, %(max_z)s))::box3d, ST_GeomFromText(%(poc_geom_wkt)s), %(is_main)s)"
        extras.execute_batch(cur, ins_eq, all_eq)

        # 2. 장애물
        ins_obs = f"INSERT INTO \"{OBSTACLE_TABLE}\" (\"SOURCE_FILE\", \"MAIN_EQ_NAME\", \"PROCESS\", \"MAKER\", \"OBJECT_ID\", \"NAME\", \"DDWORKS_TYPE\", \"OST_TYPE\", \"MIN_X\", \"MIN_Y\", \"MIN_Z\", \"MAX_X\", \"MAX_Y\", \"MAX_Z\", \"LENGTH_X\", \"LENGTH_Y\", \"LENGTH_Z\", \"POS_X\", \"POS_Y\", \"POS_Z\", \"BBOX\") VALUES (%(source_file)s, %(main_eq_name)s, %(process)s, %(maker)s, %(object_id)s, %(name)s, %(ddworks_type)s, %(ost_type)s, %(min_x)s, %(min_y)s, %(min_z)s, %(max_x)s, %(max_y)s, %(max_z)s, %(length_x)s, %(length_y)s, %(length_z)s, %(pos_x)s, %(pos_y)s, %(pos_z)s, ST_3DMakeBox(ST_MakePoint(%(min_x)s, %(min_y)s, %(min_z)s), ST_MakePoint(%(max_x)s, %(max_y)s, %(max_z)s))::box3d)"
        extras.execute_batch(cur, ins_obs, all_obs)

        # 3. 공간
        ins_sp = f"INSERT INTO \"{SPACE_TABLE}\" (\"SOURCE_FILE\", \"MAIN_EQ_NAME\", \"PROCESS\", \"MAKER\", \"LEVEL_NAME\", \"MIN_X\", \"MIN_Y\", \"MIN_Z\", \"MAX_X\", \"MAX_Y\", \"MAX_Z\", \"SIZE_X\", \"SIZE_Y\", \"SIZE_Z\", \"LENGTH_X\", \"LENGTH_Y\", \"LENGTH_Z\", \"BBOX\") VALUES (%(source_file)s, %(main_eq_name)s, %(process)s, %(maker)s, %(level_name)s, %(min_x)s, %(min_y)s, %(min_z)s, %(max_x)s, %(max_y)s, %(max_z)s, %(size_x)s, %(size_y)s, %(size_z)s, %(length_x)s, %(length_y)s, %(length_z)s, ST_3DMakeBox(ST_MakePoint(%(min_x)s, %(min_y)s, %(min_z)s), ST_MakePoint(%(max_x)s, %(max_y)s, %(max_z)s))::box3d)"
        extras.execute_batch(cur, ins_sp, all_space)

        # 4. 덕트/라테랄
        ins_dl = f"INSERT INTO \"{DUCT_LATERAL_TABLE}\" (\"SOURCE_FILE\", \"MAIN_EQ_NAME\", \"PROCESS\", \"OBJECT_ID\", \"NAME\", \"TYPE\", \"CATEGORY\", \"LEVEL\", \"UTILITY\", \"MATERIAL\", \"SIZE\", \"MIN_X\", \"MIN_Y\", \"MIN_Z\", \"MAX_X\", \"MAX_Y\", \"MAX_Z\", \"LENGTH_X\", \"LENGTH_Y\", \"LENGTH_Z\", \"POS_X\", \"POS_Y\", \"POS_Z\", \"BBOX\") VALUES (%(source_file)s, %(main_eq_name)s, %(process)s, %(object_id)s, %(name)s, %(type)s, %(category)s, %(level)s, %(utility)s, %(material)s, %(size)s, %(min_x)s, %(min_y)s, %(min_z)s, %(max_x)s, %(max_y)s, %(max_z)s, %(length_x)s, %(length_y)s, %(length_z)s, %(pos_x)s, %(pos_y)s, %(pos_z)s, ST_3DMakeBox(ST_MakePoint(%(min_x)s, %(min_y)s, %(min_z)s), ST_MakePoint(%(max_x)s, %(max_y)s, %(max_z)s))::box3d)"
        extras.execute_batch(cur, ins_dl, all_dl)

        conn.commit()
        print(f"\n[Import Success] EQ:{len(all_eq)}, OBS:{len(all_obs)}, SPACE:{len(all_space)}, DL:{len(all_dl)}")

    except Exception as e:
        print(f"[Error] {e}"); import traceback; traceback.print_exc()
    finally:
        if 'conn' in locals(): conn.close()

def main():
    parser = argparse.ArgumentParser(description="Integrated BIM Importer v3.0.")
    parser.add_argument("json_dir")
    parser.add_argument("--dbname", required=True); parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True); parser.add_argument("--host", default="localhost")
    parser.add_argument("-p", "--port", default="5432")
    parser.add_argument("--clean", action="store_true", help="기존 데이터를 삭제하고 새로 임포트합니다")
    args = parser.parse_args()
    db_params = {"host": args.host, "database": args.dbname, "user": args.user, "password": args.password, "port": args.port}
    run_integration_import(args.json_dir, db_params, clean=args.clean)

if __name__ == "__main__":
    main()
