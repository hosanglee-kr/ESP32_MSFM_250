
### 1. codeGraph 인덱스 생성

- codegraph의 기존 인덱싱을 삭제/초기화 해줘
- codegraph 아래 폴더와 파일 대상으로 인덱스 생성해줘

[lib](file:///e:/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/lib) [T2_MSFM_250](file:///e:/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/src/T2_MSFM_250) [main.cpp](file:///e:/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/src/main.cpp)

#### 진행 과정 및 관련 명령어

##### 1) 기존 인덱스 초기화 (SQLite DB 테이블 비우기)
CodeGraph의 MCP 서버가 활성화된 상태에서 프로세스가 데이터베이스 파일(`.codegraph/codegraph.db`)을 점유하고 있을 때, `codegraph uninit -f` 명령어 실행 시 `EPERM` 오류가 발생할 수 있습니다. 이 경우 SQLite DB에 연결하여 테이블 데이터를 삭제(TRUNCATE 대체)해 초기화합니다.

```python
import sqlite3

db_path = r'.codegraph\codegraph.db'
conn = sqlite3.connect(db_path)
cursor = conn.cursor()
cursor.execute("PRAGMA foreign_keys = OFF;")
for table in ['nodes', 'edges', 'files', 'unresolved_refs', 'project_metadata']:
    cursor.execute(f"DELETE FROM {table};")
conn.commit()
conn.isolation_level = None
conn.execute("VACUUM;")
conn.close()
```

##### 2) 전체 재생성 및 특정 경로만 필터링
CodeGraph는 내부적으로 `git ls-files`를 이용하여 파일을 스캔하므로 Git 추적 대상인 모든 파일을 인덱싱합니다. 특정 경로만 인덱스에 보관하기 위해 전체 인덱싱 후 SQLite 쿼리로 지정 경로 이외의 데이터를 정리해 줍니다.

* **인덱싱 명령 실행**:
  ```powershell
  codegraph index --force
  ```

* **지정한 경로 이외의 데이터 정리 (Python / SQL)**:
  ```python
  import sqlite3

  db_path = r'.codegraph\codegraph.db'
  conn = sqlite3.connect(db_path)
  cursor = conn.cursor()
  cursor.execute("PRAGMA foreign_keys = OFF;")

  # 지정 경로가 아닌 노드 삭제
  cursor.execute("""
      DELETE FROM nodes
      WHERE file_path NOT LIKE 'lib/%'
        AND file_path NOT LIKE 'src/T2_MSFM_250/%'
        AND file_path <> 'src/main.cpp';
  """)

  # 고립된 연결 정보(Edges) 삭제
  cursor.execute("""
      DELETE FROM edges
      WHERE source NOT IN (SELECT id FROM nodes)
         OR target NOT IN (SELECT id FROM nodes);
  """)

  # 지정 경로가 아닌 파일 정보 삭제
  cursor.execute("""
      DELETE FROM files
      WHERE path NOT LIKE 'lib/%'
        AND path NOT LIKE 'src/T2_MSFM_250/%'
        AND path <> 'src/main.cpp';
  """)

  conn.commit()
  conn.isolation_level = None
  conn.execute("VACUUM;")
  conn.close()
  ```



---

### 2. Graphify LLM Wiki 생성

#### 2.1 제미나이 API 키 등록

*   **임시 설정 (PowerShell)**:
    ```powershell
    $env:GEMINI_API_KEY="xxx"
    ```
*   **영구 설정 (Windows 시스템 변수)**:
    ```powershell
    [System.Environment]::SetEnvironmentVariable("GEMINI_API_KEY", "your_api_key_here", "User")
    ```

#### 2.2. graphify LLM Wiki 생성

- 요청 프롬프트 : graphify skill을 사용해서

-  아래 폴더들을 Raw소스로 하여 LLM Wiki 만들어줘
    - [T2_MSFM_250](file;file:///e%3A/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/src/T2_MSFM_250), [esp-dsp](file;file:///e%3A/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/lib/esp-dsp)[SensorFusion](file;file:///e%3A/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/lib/SensorFusion)[SparkFun%20BMI270%20Arduino%20Library](file;file:///e%3A/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/lib/SparkFun%20BMI270%20Arduino%20Library)[VectorQuaternionMatrix](file;file:///e%3A/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/lib/VectorQuaternionMatrix)
- gemini API 사용 예정 api 키 설정 방법 알려줘

- graphify output은 [T2_MSFM_250_wiki_graphify_out](file;file:///e%3A/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/src/T2_MSFM_250_wiki/T2_MSFM_250_wiki_graphify_out)폴더에 만들어줘
- 추가로 옵시디언용 Valt도 만들어주고 Volt 폴더는 [T2_MSFM_250_wiki_Obsidian](file;file:///e%3A/241224_Sub/50_2540/Platformio7/ESP32_MSFM_110/src/T2_MSFM_250_wiki/T2_MSFM_250_wiki_Obsidian)여기에 저장해줘


여러 개의 로컬 라이브러리 및 소스 폴더를 하나의 통합 지식 그래프로 빌드하고, 이를 바탕으로 LLM Wiki 보고서 및 Obsidian Vault를 생성하는 명령어 구성입니다.

##### 1) 각 폴더별 개별 코드/문서 특징 추출 (extract)
```powershell
graphify extract "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\src\T2_MSFM_250" --backend gemini --no-cluster --out "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\src\T2_MSFM_250_wiki\T2_MSFM_250_wiki_graphify_out\T2_MSFM_250"
graphify extract "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\lib\esp-dsp" --backend gemini --no-cluster --out "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\src\T2_MSFM_250_wiki\T2_MSFM_250_wiki_graphify_out\esp-dsp"
graphify extract "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\lib\SensorFusion" --backend gemini --no-cluster --out "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\src\T2_MSFM_250_wiki\T2_MSFM_250_wiki_graphify_out\SensorFusion"
graphify extract "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\lib\SparkFun BMI270 Arduino Library" --backend gemini --no-cluster --out "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\src\T2_MSFM_250_wiki\T2_MSFM_250_wiki_graphify_out\SparkFun_BMI270"
graphify extract "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\lib\VectorQuaternionMatrix" --backend gemini --no-cluster --out "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\src\T2_MSFM_250_wiki\T2_MSFM_250_wiki_graphify_out\VectorQuaternionMatrix"
```

##### 2) 수동 병합 및 클러스터링/Wiki 분석 실행
`graphify merge-graphs`가 raw extraction json을 병합할 때 발생할 수 있는 NetworkX 형변환 스키마 제약을 우회하기 위해, 직접 JSON을 머지하고 CLI를 통해 갱신합니다.

* **병합 및 룰 가동 스크립트 실행**:
  ```powershell
  python C:\Users\A0122016023\.gemini\antigravity-ide\brain\8d478b0f-ca2d-4224-aaeb-8ec0317e1983\scratch\build_merged_graph.py
  ```

##### 3) Obsidian Vault 내보내기 (export)
```powershell
graphify export obsidian --dir "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\src\T2_MSFM_250_wiki\T2_MSFM_250_wiki_Obsidian" --graph "e:\241224_Sub\50_2540\Platformio7\ESP32_MSFM_110\src\T2_MSFM_250_wiki\T2_MSFM_250_wiki_graphify_out\graphify-out\graph.json"
```



