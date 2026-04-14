# =====================================================================================
# [실행 명령어]
#   python import_pipe_group_analyze.py ./group_rule_data-v11_20260406113523_equipment+util.csv \
#       --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432 --clean
#   (원격) python import_pipe_group_analyze.py ./group_rule_data-v11_20260406113523_equipment+util.csv \
#       --dbname AUTOROUTINGV7 --user dinno --password dinno --host 192.168.0.35 -p 55432 --clean
# =====================================================================================
#
# import_pipe_group_analyze.py  — 배관 그룹 분석 결과 DB 적재
# =============================================================
#
# [프로그램 개요]
#   analyze_group_pipes.py에서 생성한 배관 그룹 분석 CSV를 PostgreSQL(PostGIS)에 적재합니다.
#   그룹의 3D 바운딩 박스를 POLYGON Z WKT로 변환하여 공간 쿼리를 지원합니다.
#
# [전체 흐름도]
#   ┌──────────────────────────────────────────────────────────────┐
#   │  1. CSV 경로 + DB 접속 정보 파싱 (argparse)                  │
#   │  2. PostgreSQL 연결 + TB_PIPE_GROUP_ANALYZE 스키마 검증/생성 │
#   │  3. PostGIS 확장 로드 (POLYGON Z 지원)                       │
#   │  4. CSV 파싱 (parse_csv_data):                               │
#   │     └─ 그룹 BBox → POLYGON Z WKT 변환                       │
#   │  5. 동일 파일명 기존 데이터 DELETE (중복 방지)                │
#   │  6. executemany()로 일괄 INSERT                              │
#   └──────────────────────────────────────────────────────────────┘
#
# [주요 함수]
#   - get_db_schema_sql()                   : TB_PIPE_GROUP_ANALYZE CREATE 쿼리
#   - parse_csv_data(csv_path)              : CSV → 데이터 튜플 리스트 변환
#   - run_postgresql_import(csv_path, db)   : POLYGON Z WKT 변환 + DB 적재
#
# [주요 변수]
#   - rows             : CSV에서 변환된 메타 정보 리스트
#   - unique_filenames : 중복 DELETE 대상 파일명 집합
#   - wkt_str          : POLYGON Z WKT 문자열 (그룹 면적 표현)
#   - db_params        : DB 접속 정보 딕셔너리
# =====================================================================================

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
