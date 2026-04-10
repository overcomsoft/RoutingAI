import csv
import argparse
import os
import sys
from datetime import datetime

# ==============================================================================
# 실행 명령어 (Command Line)
# python import_objects.py <CSV_FILE_PATH> --dbname <DB_NAME> --user <DB_USER> --password <DB_PW> [--host DB_HOST -p DB_PORT]
# 예시: python import_objects.py ./all_objects_summary_en_20260401_121428.csv --dbname AUTOROUTINGV7 --user postgres --password dinno --host localhost -p 5432
# 예시 : python import_objects.py ./all_objects_summary_en_20260401_121428.csv --dbname AUTOROUTINGV7 --user dinno  --password dinno --host 192.168.0.41  -p 55432
# ==============================================================================

def get_db_schema_sql():
    return """
    CREATE TABLE IF NOT EXISTS "TB_OBJECTS" (
        "ID" SERIAL PRIMARY KEY,
        "SOURCE_FILE" TEXT,
        "CATEGORY" TEXT,
        "OBJECT_ID" TEXT,
        "NAME" TEXT,
        "LEVEL_NAME" TEXT,
        "MIN_X" FLOAT,
        "MIN_Y" FLOAT,
        "MIN_Z" FLOAT,
        "MAX_X" FLOAT,
        "MAX_Y" FLOAT,
        "MAX_Z" FLOAT,
        "BBOX" box3d,
        "IMPORT_TIME" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

def validate_and_ensure_schema(cur):
    """ 기존 테이블의 스키마를 검사하고, 구조가 다르면 삭제 후 재생성합니다. """
    table_name = "TB_OBJECTS"
    
    # 1. 테이블 존재 여부 확인
    cur.execute(f"SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = '{table_name}');")
    if not cur.fetchone()[0]:
        return # 테이블이 없으면 생성 로직으로 진행

    # 2. 컬럼 정보 및 타입 확인
    # 특히 BBOX 컬럼의 타입이 box3d인지 확인
    cur.execute(f"""
        SELECT column_name, udt_name 
        FROM information_schema.columns 
        WHERE table_name = '{table_name}';
    """)
    cols = {row[0]: row[1] for row in cur.fetchall()}
    
    # 필수 컬럼 존재 확인
    expected_cols = ["ID", "SOURCE_FILE", "CATEGORY", "OBJECT_ID", "NAME", "LEVEL_NAME", 
                     "MIN_X", "MIN_Y", "MIN_Z", "MAX_X", "MAX_Y", "MAX_Z", "BBOX"]
    
    recreate = False
    for col in expected_cols:
        if col not in cols:
            recreate = True
            break
            
    # BBOX 타입 상세 확인 (box3d 인지)
    if not recreate:
        if cols.get('BBOX') != 'box3d':
            recreate = True

    if recreate:
        print(f"[Schema] Table '{table_name}' structure is inconsistent. Recreating...")
        cur.execute(f'DROP TABLE IF EXISTS "{table_name}" CASCADE;')

def parse_csv_data(csv_path):
    """ CSV 파일을 읽어서 데이터 리스트와 고유 파일명 세트를 추출합니다. """
    rows = []
    unique_filenames = set()
    
    if not os.path.exists(csv_path):
        print(f"Error: CSV file not found at {csv_path}")
        return None, None

    data_lines = []
    try:
        # English headers expected: SourceFile, Category, ID, Name, Level, MinX, MinY, MinZ, MaxX, MaxY, MaxZ
        with open(csv_path, mode='r', encoding='utf-8-sig') as f:
            data_lines = f.readlines()
    except UnicodeDecodeError:
        with open(csv_path, mode='r', encoding='cp949') as f:
            data_lines = f.readlines()

    if not data_lines:
        return [], set()

    reader = csv.DictReader(data_lines)
    for row in reader:
        fname = row.get('SourceFile', 'Unknown')
        unique_filenames.add(fname)
        
        def to_f(k):
            try: return float(row.get(k, 0))
            except: return 0.0
            
        mn_x, mn_y, mn_z = to_f('MinX'), to_f('MinY'), to_f('MinZ')
        mx_x, mx_y, mx_z = to_f('MaxX'), to_f('MaxY'), to_f('MaxZ')
        
        # WKT string for ST_GeomFromText (PostGIS handles Box3D parsing)
        # ST_3DMakeBox(ST_PointZ(minx, miny, minz), ST_PointZ(maxx, maxy, maxz)) is more robust in SQL level
        
        rows.append((
            fname,
            row.get('Category', 'Unknown'),
            row.get('ID', 'N/A'),
            row.get('Name', 'N/A'),
            row.get('Level', 'N/A'),
            mn_x, mn_y, mn_z,
            mx_x, mx_y, mx_z,
            # Pass min/max coordinates to SQL ST_3DMakeBox
            mn_x, mn_y, mn_z, mx_x, mx_y, mx_z
        ))
            
    return rows, unique_filenames

def run_postgresql_import(csv_path, db_params):
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
        
        # 1. PostGIS 체크
        try:
            cur.execute("CREATE EXTENSION IF NOT EXISTS postgis;")
            conn.commit()
        except:
            conn.rollback()
            
        # 2. 스키마 검증 및 테이블 생성
        validate_and_ensure_schema(cur)
        cur.execute(get_db_schema_sql())
        cur.execute('CREATE INDEX IF NOT EXISTS idx_objects_source_file ON "TB_OBJECTS"("SOURCE_FILE");')
        # 3D 검색을 위해 GiST 인덱스 생성
        # cur.execute('CREATE INDEX IF NOT EXISTS idx_tb_objects_bbox_3d ON "TB_OBJECTS" USING GIST ("BBOX");')
        
        # 3. CSV 데이터 로드
        rows, filenames = parse_csv_data(csv_path)
        if not rows:
            print("[Error] No data found in CSV.")
            return
            
        print(f"[1/3] CSV Parsing complete: {len(rows)} records found.")
        
        # 4. 중복 방지를 위한 기존 데이터 삭제
        if filenames:
            placeholders = ', '.join(['%s'] * len(filenames))
            cur.execute(f"DELETE FROM \"TB_OBJECTS\" WHERE \"SOURCE_FILE\" IN ({placeholders})", list(filenames))
            deleted_count = cur.rowcount
            if deleted_count > 0:
                print(f"[2/3] Deleted {deleted_count} existing records for duplicate prevention.")
            else:
                print(f"[2/3] No existing records found. Starting fresh.")
        
        # 5. 데이터 삽입 (box3d 형식 - 3D Bounding Box)
        insert_sql = """
        INSERT INTO "TB_OBJECTS" (
            "SOURCE_FILE", "CATEGORY", "OBJECT_ID", "NAME", "LEVEL_NAME", 
            "MIN_X", "MIN_Y", "MIN_Z", "MAX_X", "MAX_Y", "MAX_Z",
            "BBOX"
        ) VALUES (
            %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, 
            ST_3DMakeBox(ST_MakePoint(%s, %s, %s), ST_MakePoint(%s, %s, %s))::box3d
        )
        """
        
        # rows list transformation for box3d (6 coords: min_x, min_y, min_z, max_x, max_y, max_z)
        box_rows = []
        for r in rows:
            # Source coordinates from indices 5 to 10
            fn, cat, oid, nm, lvl, mnx, mny, mnz, mxx, mxy, mxz = r[:11]
            box_rows.append((
                fn, cat, oid, nm, lvl, mnx, mny, mnz, mxx, mxy, mxz,
                # Box coords
                mnx, mny, mnz, mxx, mxy, mxz
            ))

        cur.executemany(insert_sql, box_rows)
        conn.commit()
        print(f"[3/3] DB Import complete: {len(rows)} records saved to TB_OBJECTS.")
        
    except Exception as e:
        import traceback
        print(f"[Error] PostgreSQL Error: {e}")
        traceback.print_exc()
    finally:
        if 'conn' in locals(): conn.close()

def main():
    parser = argparse.ArgumentParser(description="Import objects summary CSV to PostgreSQL.")
    parser.add_argument("csv_file", help="Path to the CSV file")
    parser.add_argument("--dbname", required=True, help="Database name")
    parser.add_argument("--user", required=True, help="Database user")
    parser.add_argument("--password", required=True, help="Database password")
    parser.add_argument("--host", default="localhost", help="Database host (default: localhost)")
    parser.add_argument("-p", "--port", default="5432", help="Database port (default: 5432)")
    
    args = parser.parse_args()
    db_params = {
        "host": args.host, "database": args.dbname,
        "user": args.user, "password": args.password, "port": args.port
    }
    run_postgresql_import(args.csv_file, db_params)

if __name__ == "__main__":
    main()
