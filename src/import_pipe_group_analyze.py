# ==============================================================================
# 실행 방법 (Command Line)
# python import_pipe_group_analyze.py <CSV_FILE_PATH> --dbname <DB_NAME> --user <DB_USER> --password <DB_PW> [--host DB_HOST -p DB_PORT] [--clean]
# 예시: python import_pipe_group_analyze.py ./group_rule_data-v11_20260406113523_equipment+util.csv --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432 --clean
# 원격  python import_pipe_group_analyze.py ./group_rule_data-v11_20260406113523_equipment+util.csv --dbname AUTOROUTINGV7 --user dinno  --password dinno --host 192.168.0.35 -p 55432 --clean
# [전체적인 흐름도 및 알고리즘]
# 1. 인자로 전달받은 CSV 경로 및 PostgreSQL 접속 정보를 파싱하여 읽어들입니다.
# 2. PostgreSQL DB에 연결하고, TB_PIPE_GROUP_ANALYZE 테이블의 존재 및 스키마 구조(컬럼)를 비교하여 최신화합니다 (다를 경우 자동 재생성).
# 3. PostGIS 확장을 로드하며, WKT 형상 데이터를 포함시킬 준비를 합니다. (POLYGON Z)
# 4. CSV 파일을 읽고(parse_csv_data), 읽어온 정보를 기반으로 BBOX_GEOMETRY 생성을 위한 3D 바운딩 박스를 구성합니다.
# 5. 기존에 저장된 동일한 파일명의 데이터가 존재하면 중복 적재 방지를 위해 DELETE를 수행합니다.
# 6. executemany()를 사용해 파싱된 다수의 행(row) 데이터를 DB에 한 번에 (INSERT) 적재합니다.
#
# [주요 함수 설명]
# - get_db_schema_sql: TB_PIPE_GROUP_ANALYZE 테이블의 CREATE 구문을 반환하는 함수
# - parse_csv_data(csv_path): 인코딩 문제(utf-8/cp949) 방지를 포함하여 CSV를 읽고 Python 데이터 튜플 리스트로 변환.
# - run_postgresql_import(csv_path, db_params): 파싱된 데이터를 기반으로, PostGIS에서 인식 가능한 다각형 좌표(WKT)로 변환하고, DB에 적재를 수행.
#
# [주요 변수 설명]
# - rows: CSV에서 읽어들여 튜플 형태로 변환된 기본 메타 정보 리스트
# - unique_filenames: 중복 스캔 시 이전 파일 레코드들을 찾아 삭제(DELETE) 쿼리를 날릴 타겟들
# - wkt_str: 3차원 공간에서 그룹의 면적을 나타내기 위해 조합된 POLYGON Z WKT 문자열
# - db_params: 데이터베이스 연결 주소(IP), 포트, 유저명, 비밀번호 등의 정보를 담고 있는 Dictionary
# ==============================================================================

import csv
import argparse
import os
import sys
from datetime import datetime

def get_db_schema_sql():

    """ 데이터베이스 테이블 생성을 위한 표준 SQL 쿼리를 반환합니다. """
    return """
    CREATE TABLE IF NOT EXISTS "TB_PIPE_GROUP_ANALYZE" (
        "ID" SERIAL PRIMARY KEY,
        "FILE_NAME" TEXT,
        "EQUIPMENT_NAME" TEXT,
        "EQUIPMENT_ID" TEXT,
        "PROCESS" TEXT,
        "MAKER" TEXT,
        "EQUIPMENT_TYPE" TEXT,
        "UTILITY" TEXT,
        "BAY" TEXT,
        "LEVEL" TEXT,
        "GROUP_TYPE" TEXT,
        "PIPE_COUNT" INTEGER,
        "SPACING" FLOAT,
        "ELEVATION" FLOAT,
        "BOP" FLOAT,
        "LEFT_X" FLOAT,
        "LEFT_Y" FLOAT,
        "LEFT_Z" FLOAT,
        "RIGHT_X" FLOAT,
        "RIGHT_Y" FLOAT,
        "RIGHT_Z" FLOAT,
        "BOUND_WIDTH" FLOAT,
        "BOUND_DEPTH" FLOAT,
        "BOUND_HEIGHT" FLOAT,
        "BBOX_GEOMETRY" GEOMETRY(POLYGONZ),
        "IMPORT_TIME" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

def parse_csv_data(csv_path):
    rows = []
    unique_filenames = set()
    
    if not os.path.exists(csv_path):
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

    reader = csv.reader(data_lines)
    header = next(reader)
    for row in reader:
            if not row or len(row) < 23: continue
            
            def to_f(val):
                try: return float(val)
                except: return 0.0
            def to_i(val):
                try: return int(float(val))
                except: return 0
                
            unique_filenames.add(row[0])
            rows.append((
                row[0], row[1], row[2], row[3], row[4] if row[4] else "N/A", row[5], row[6], row[7], row[8], row[9],
                to_i(row[10]), to_f(row[11]), to_f(row[12]), to_f(row[13]),
                to_f(row[14]), to_f(row[15]), to_f(row[16]),
                to_f(row[17]), to_f(row[18]), to_f(row[19]),
                to_f(row[20]), to_f(row[21]), to_f(row[22])
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
            WHERE lower(table_name) = 'tb_pipe_group_analyze' 
            AND table_schema = 'public'
        """)
        fetched = cur.fetchall()
        
        has_table = len(fetched) > 0
        actual_table_name = fetched[0][0] if has_table else None
        # DB에 저장된 실제 컬럼명들을 수집
        existing_cols = {row[1] for row in fetched}
        
        expected_cols = {
            "ID", "FILE_NAME", "EQUIPMENT_NAME", "EQUIPMENT_ID", "PROCESS", "MAKER", "EQUIPMENT_TYPE", "UTILITY", "BAY", "LEVEL", 
            "GROUP_TYPE", "PIPE_COUNT", "SPACING", "ELEVATION", "BOP", 
            "LEFT_X", "LEFT_Y", "LEFT_Z", "RIGHT_X", "RIGHT_Y", "RIGHT_Z", 
            "BOUND_WIDTH", "BOUND_DEPTH", "BOUND_HEIGHT", "BBOX_GEOMETRY", "IMPORT_TIME"
        }
        
        # 2. 강제 재생성 조건 확인
        #   - 테이블명이 대문자 "TB_PIPE_GROUP_ANALYZE"가 아니거나 (소문자로 존재)
        #   - 컬럼구성이 다르거나
        needs_recreate = False
        if has_table:
            if actual_table_name != 'TB_PIPE_GROUP_ANALYZE':
                needs_recreate = True
            elif existing_cols != expected_cols:
                needs_recreate = True
                
        if needs_recreate:
            print(f"[알림] 테이블 대소문자 또는 스키마 변경 감지. 재생성합니다. (기존: {actual_table_name})")
            cur.execute('DROP TABLE IF EXISTS "TB_PIPE_GROUP_ANALYZE" CASCADE;')
            cur.execute('DROP TABLE IF EXISTS tb_pipe_group_analyze CASCADE;')
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
        cur.execute('CREATE INDEX IF NOT EXISTS idx_pipe_group_filename ON "TB_PIPE_GROUP_ANALYZE"("FILE_NAME");')
        cur.execute('CREATE INDEX IF NOT EXISTS idx_pipe_group_geom ON "TB_PIPE_GROUP_ANALYZE" USING GIST ("BBOX_GEOMETRY");')
        
        rows, filenames = parse_csv_data(csv_path)
        if not rows:
            print("[오류] CSV 데이터가 없거나 파일을 읽지 못했습니다.")
            return
        
        print(f"[1/3] CSV 파싱 완료: {len(rows)}개 레코드 읽음 (파일명: {', '.join(filenames)})")
        
        if clean_mode:
            print("[알림] --clean 옵션이 설정되었습니다. 기존 데이터를 모두 삭제합니다.")
            cur.execute('DELETE FROM "TB_PIPE_GROUP_ANALYZE"')
            print(f"      -> {cur.rowcount}개 레코드 삭제 완료.")
        else:
            placeholders = ', '.join(['%s'] * len(filenames))
            cur.execute(f"DELETE FROM \"TB_PIPE_GROUP_ANALYZE\" WHERE \"FILE_NAME\" IN ({placeholders})", list(filenames))
            deleted_count = cur.rowcount
            if deleted_count > 0:
                print(f"[2/3] 기존 데이터 삭제: {deleted_count}개 레코드 삭제됨 (중복 방지)")
            else:
                print(f"[2/3] 기존 데이터 없음 (신규 입력)")
        
        postgres_rows = []
        for r in rows:
            minx, miny, minz = r[14], r[15], r[16]
            maxx, maxy, maxz = r[17], r[18], r[19]
            if minx == maxx: maxx += 0.01
            if miny == maxy: maxy += 0.01
            wkt_str = f"POLYGON Z (({minx} {miny} {minz}, {maxx} {miny} {minz}, {maxx} {maxy} {minz}, {minx} {maxy} {minz}, {minx} {miny} {minz}))"
            postgres_rows.append(r + (wkt_str,))
        
        insert_sql = """
        INSERT INTO "TB_PIPE_GROUP_ANALYZE" (
            "FILE_NAME", "EQUIPMENT_NAME", "EQUIPMENT_ID", "PROCESS", "MAKER", "EQUIPMENT_TYPE", "UTILITY", "BAY", "LEVEL", 
            "GROUP_TYPE", "PIPE_COUNT", "SPACING", "ELEVATION", "BOP", 
            "LEFT_X", "LEFT_Y", "LEFT_Z", "RIGHT_X", "RIGHT_Y", "RIGHT_Z", 
            "BOUND_WIDTH", "BOUND_DEPTH", "BOUND_HEIGHT", "BBOX_GEOMETRY"
        ) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s, ST_GeomFromText(%s))
        """
        cur.executemany(insert_sql, postgres_rows)
        conn.commit()
        print(f"[3/3] DB 입력 완료: {len(postgres_rows)}개 레코드가 TB_PIPE_GROUP_ANALYZE 테이블에 저장되었습니다.")
        print(f"      -> 테이블: TB_PIPE_GROUP_ANALYZE | 입력파일: {', '.join(filenames)}")
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
