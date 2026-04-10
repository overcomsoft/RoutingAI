# ==============================================================================
# 실행 방법 (Command Line)
# python import_duct_poc_cluster.py <CSV_FILE_PATH> --dbname <DB_NAME> --user <DB_USER> --password <DB_PW> [--host DB_HOST -p DB_PORT] [--clean]
# 예시: python import_duct_poc_cluster.py ./duct_poc_cluster_20260406_113528.csv --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432 --clean
# 원격 python import_duct_poc_cluster.py ./duct_poc_cluster_20260330_084611.csv --dbname AUTOROUTINGV7 --user dinno  --password dinno --host 192.168.0.41 -p 55432  
# 원격2 python import_duct_poc_cluster.py ./duct_poc_cluster_20260406_113528.csv --dbname AUTOROUTINGV7 --user dinno  --password dinno --host 192.168.0.35 -p 55432 --clean

#
# [전체적인 흐름도 및 알고리즘]
# 1. 인자로 전달받은 CSV 경로 및 PostgreSQL 접속 정보를 파싱하여 읽어들입니다.
# 2. PostgreSQL DB에 연결하고, TB_DUCT_POC_CLUSTER 테이블의 존재 및 스키마 구조(컬럼)를 비교하여 최신화합니다 (없거나 다르면 재생성).
# 3. PostGIS 확장을 로드하며, WKT 형상 데이터를 포함시킬 준비를 합니다.
# 4. CSV 파일을 읽고(parse_csv_data), 읽어온 PoC 좌표 문자열을 WKT(MULTIPOINT Z) 포맷으로 가공합니다.
# 5. 기존에 저장된 동일한 파일명의 데이터가 존재하면 중복 적재 방지를 위해 DELETE를 수행합니다.
# 6. executemany()를 사용해 파싱된 다수의 행(row) 데이터를 DB에 안전하고 통째로(INSERT) 적재합니다.
#
# [주요 함수 설명]
# - get_db_schema_sql: TB_DUCT_POC_CLUSTER 테이블의 CREATE 구문을 반환하는 함수
# - parse_csv_data(csv_path): 인코딩 문제 방지를 포함하여 CSV를 읽고 (data_rows, distinct_filenames) 튜플을 반환. 문자로 된 좌표들을 MULTIPOINT Z 문자열로 바꿈.
# - run_postgresql_import(csv_path, db_params): 파싱된 데이터를 DB에 연결 및 검증 후 삽입.
#
# [주요 변수 설명]
# - rows: CSV에서 읽어들여 튜플 형태로 변환된 최종 데이터 리스트
# - unique_filenames: 중복 삭제(DELETE) 쿼리에 쓰일 삽입 타겟들(파일명집합)
# - wkt_geom: MULTIPOINT Z 포맷으로 조립된 공간 객체의 Well-Known Text
# - db_params: 데이터베이스 접속 호스트, 포트, 이름, 권한 등의 Dictionary
# ==============================================================================

import csv
import argparse
import os
import sys

def get_db_schema_sql():

    """ 데이터베이스 테이블 생성을 위한 표준 SQL 쿼리를 반환합니다. """
    return """
    CREATE TABLE IF NOT EXISTS "TB_DUCT_POC_CLUSTER" (
        "ID" SERIAL PRIMARY KEY,
        "FILE_NAME" TEXT,
        "PROCESS" TEXT,
        "MAKER" TEXT,
        "BAY" TEXT,
        "LEVEL" TEXT,
        "EQUIP_NAME" TEXT,
        "DUCT_ID" TEXT,
        "UTILITY" TEXT,
        "TOTAL_POC_COUNT" INTEGER,
        "CLUSTER_COUNT" INTEGER,
        "CLUSTER_ID" TEXT,
        "POC_ID_LIST" TEXT,
        "POC_POS_LIST" TEXT,
        "DUCT_FACE" TEXT,
        "SPACE_MIN_X" FLOAT,
        "SPACE_MAX_X" FLOAT,
        "SPACE_MIN_Y" FLOAT,
        "SPACE_MAX_Y" FLOAT,
        "SPACE_MIN_Z" FLOAT,
        "SPACE_MAX_Z" FLOAT,
        "BOUND_MIN" TEXT,
        "BOUND_MAX" TEXT,
        "BOUND_WIDTH" FLOAT,
        "BOUND_DEPTH" FLOAT,
        "BOUND_HEIGHT" FLOAT,
        "POC_GEOMETRY" GEOMETRY(MULTIPOINTZ),
        "IMPORT_TIME" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

def parse_csv_data(csv_path):
    """ CSV 파일을 읽어서 데이터 리스트와 고유 파일명 세트를 추출합니다. """
    rows = []
    unique_filenames = set()
    
    if not os.path.exists(csv_path):
        print(f"Error: CSV file not found at {csv_path}")
        return None, None

    data_lines = []
    try:
        with open(csv_path, mode='r', encoding='utf-8-sig') as f:
            data_lines = f.readlines()
    except UnicodeDecodeError:
        with open(csv_path, mode='r', encoding='cp949') as f:
            data_lines = f.readlines()

    if not data_lines:
        return [], set()

    reader = csv.DictReader(data_lines)
    for row in reader:
            fname = row['파일명']
            unique_filenames.add(fname)
            
            def to_f(k):
                try: return float(row.get(k, 0))
                except: return 0.0
            def to_i(k):
                try: return int(float(row.get(k, 0)))
                except: return 0
                
            # WKT 3D 형상 데이터 생성 (MULTIPOINT Z)
            poc_pos_str = row['PoC좌표']
            pos_str_list = poc_pos_str.split(' | ')
            points = []
            for pos_str in pos_str_list:
                clean_str = pos_str.replace('(', '').replace(')', '')
                points.append(clean_str.replace(', ', ' '))
            wkt_geom = "MULTIPOINT Z(" + ", ".join(points) + ")"
            
            rows.append((
                row['파일명'], row['PROCESS'], row.get('MAKER') if row.get('MAKER') else "N/A", row['BAY'], row['LEVEL'],
                row['장비명'], row['덕트ID'], row['유틸리티'], 
                to_i('전체PoC수량'), to_i('클러스터링수량'), row['클러스터링ID'], 
                row['PoCID리스트'], row['PoC좌표'], row['duct_face'],
                to_f('space_min_x'), to_f('space_max_x'),
                to_f('space_min_y'), to_f('space_max_y'),
                to_f('space_min_z'), to_f('space_max_z'),
                row['BoundBox최소좌표'], row['BoundBox최대좌표'],
                to_f('Bound너비(X)'), to_f('Bound깊이(Y)'), to_f('Bound높이(Z)'),
                wkt_geom
            ))
            
    return rows, unique_filenames

def run_postgresql_import(csv_path, db_params, clean_mode=False):
    try:
        import psycopg2
    except ImportError:
        print("Error: 'psycopg2' library is required.")
        return

    import os
    os.environ['PGCLIENTENCODING'] = 'UTF8'
    try:
        conn = psycopg2.connect(**db_params)
        conn.set_client_encoding('UTF8')
        cur = conn.cursor()
        
        # 1. 스키마 및 명칭 일치 여부 검사 (소문자/대문자 모두 조회)
        cur.execute("""
            SELECT table_name, column_name 
            FROM information_schema.columns 
            WHERE lower(table_name) = 'tb_duct_poc_cluster' 
            AND table_schema = 'public'
        """)
        fetched = cur.fetchall()
        
        has_table = len(fetched) > 0
        actual_table_name = fetched[0][0] if has_table else None
        # DB에 저장된 실제 컬럼명들을 수집
        existing_cols = {row[1] for row in fetched}
        
        expected_cols = {
            "ID", "FILE_NAME", "PROCESS", "MAKER", "BAY", "LEVEL", "EQUIP_NAME", "DUCT_ID", "UTILITY", 
            "TOTAL_POC_COUNT", "CLUSTER_COUNT", "CLUSTER_ID", 
            "POC_ID_LIST", "POC_POS_LIST", "DUCT_FACE",
            "SPACE_MIN_X", "SPACE_MAX_X", "SPACE_MIN_Y", "SPACE_MAX_Y", "SPACE_MIN_Z", "SPACE_MAX_Z",
            "BOUND_MIN", "BOUND_MAX", "BOUND_WIDTH", "BOUND_DEPTH", "BOUND_HEIGHT", "POC_GEOMETRY", "IMPORT_TIME"
        }
        
        # 2. 강제 재생성 조건 확인
        #   - 테이블명이 대문자 "TB_DUCT_POC_CLUSTER"가 아니거나 (소문자로 존재)
        #   - 컬럼명이 대문자가 아니거나
        #   - 컬럼 수/구성이 다르거나
        needs_recreate = False
        if has_table:
            if actual_table_name != 'TB_DUCT_POC_CLUSTER':
                needs_recreate = True
            elif existing_cols != expected_cols:
                needs_recreate = True
                
        if needs_recreate:
            print(f"[알림] 테이블 대소문자 또는 스키마 변경 감지. 재생성합니다. (기존: {actual_table_name})")
            cur.execute('DROP TABLE IF EXISTS "TB_DUCT_POC_CLUSTER" CASCADE;')
            cur.execute('DROP TABLE IF EXISTS tb_duct_poc_cluster CASCADE;')
            conn.commit()
        else:
            status_tag = "[확인]" if has_table else "[신규]"
            print(f"{status_tag} 테이블 스키마 검증 완료.")
            
        try:
            cur.execute("CREATE EXTENSION IF NOT EXISTS postgis;")
            conn.commit()
        except:
            conn.rollback()
            
        cur.execute(get_db_schema_sql())
        cur.execute('CREATE INDEX IF NOT EXISTS idx_duct_poc_filename ON "TB_DUCT_POC_CLUSTER"("FILE_NAME");')
        cur.execute('CREATE INDEX IF NOT EXISTS idx_duct_poc_geom ON "TB_DUCT_POC_CLUSTER" USING GIST ("POC_GEOMETRY");')
        
        rows, filenames = parse_csv_data(csv_path)
        if not rows:
            print("[오류] CSV 데이터가 없거나 파일을 읽지 못했습니다.")
            return
            
        print(f"[1/3] CSV 파싱 완료: {len(rows)}개 레코드 읽음")
        
        if clean_mode:
            print("[알림] --clean 옵션이 설정되었습니다. 기존 데이터를 모두 삭제합니다.")
            cur.execute('DELETE FROM "TB_DUCT_POC_CLUSTER"')
            print(f"      -> {cur.rowcount}개 레코드 삭제 완료.")
        else:
            placeholders = ', '.join(['%s'] * len(filenames))
            cur.execute(f"DELETE FROM \"TB_DUCT_POC_CLUSTER\" WHERE \"FILE_NAME\" IN ({placeholders})", list(filenames))
            deleted_count = cur.rowcount
            if deleted_count > 0:
                print(f"[2/3] 기존 데이터 삭제: {deleted_count}개 레코드 삭제됨 (중복 방지)")
            else:
                print(f"[2/3] 기존 데이터 없음 (신규 입력)")
        
        insert_sql = """
        INSERT INTO "TB_DUCT_POC_CLUSTER" (
            "FILE_NAME", "PROCESS", "MAKER", "BAY", "LEVEL", "EQUIP_NAME", "DUCT_ID", "UTILITY", 
            "TOTAL_POC_COUNT", "CLUSTER_COUNT", "CLUSTER_ID", 
            "POC_ID_LIST", "POC_POS_LIST", "DUCT_FACE",
            "SPACE_MIN_X", "SPACE_MAX_X", "SPACE_MIN_Y", "SPACE_MAX_Y", "SPACE_MIN_Z", "SPACE_MAX_Z",
            "BOUND_MIN", "BOUND_MAX", "BOUND_WIDTH", "BOUND_DEPTH", "BOUND_HEIGHT", "POC_GEOMETRY"
        ) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s, ST_GeomFromText(%s, 0))
        """
        cur.executemany(insert_sql, rows)
        conn.commit()
        print(f"[3/3] DB 입력 완료: {len(rows)}개 레코드가 TB_DUCT_POC_CLUSTER 테이블에 저장되었습니다.")
        print(f"      -> 테이블: TB_DUCT_POC_CLUSTER | 입력파일: {', '.join(filenames)}")
    except Exception as e:
        import traceback
        print(f"[오류] PostgreSQL Error: {e}")
        traceback.print_exc()
    finally:
        if 'conn' in locals(): conn.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    parser.add_argument("--dbname", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("-p", "--port", default="5432")
    parser.add_argument("--clean", action="store_true", help="Delete all existing data before import")
    
    args = parser.parse_args()
    db_params = {
        "host": args.host, "database": args.dbname,
        "user": args.user, "password": args.password, "port": args.port
    }
    run_postgresql_import(args.csv_file, db_params, clean_mode=args.clean)

if __name__ == "__main__":
    main()
