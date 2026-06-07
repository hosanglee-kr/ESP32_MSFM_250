# Graphify Wiki & Obsidian Update Tool

이 폴더는 차후에 소스코드 변경 등으로 지식 그래프 및 Wiki를 갱신할 때 사용하는 자동 업데이트 스크립트를 포함하고 있습니다.

## 스크립트 파일
* **[update_wiki.py](./update_wiki.py)**: 필터링 및 갱신용 Python 스크립트
* **[update_codegraph.py](./update_codegraph.py)**: 코드그래프 동기화 및 DB 클리닝용 Python 스크립트

## 사용 방법
소스코드가 변경되어 갱신이 필요할 경우, 터미널(PowerShell)에서 다음 명령어를 실행하십시오.

*참고: 스크립트 실행 시 필요한 파이썬 가상환경(`.venv`)과 패키지(`graphifyy`, `networkx`)는 첫 실행 시 자동으로 구축되므로 별도로 사전 세팅할 필요가 없습니다.*

1. **제미나이 API 키 설정 (필요 시)**
   ```powershell
   $env:GEMINI_API_KEY="your_api_key_here"
   ```
   *(이미 윈도우 환경 변수에 등록되어 있는 경우 생략 가능)*

2. **스크립트 실행**
   * 코드그래프 정제 및 동기화:
     ```powershell
     python .agent/skills/graphify/scripts/update_codegraph.py
     ```
   * 위키 및 옵시디언 갱신:
     ```powershell
     python .agent/skills/graphify/scripts/update_wiki.py
     ```

## 스크립트가 처리하는 작업
* `T2_MSFM_250_wiki` 내 기존 통합 추출 데이터를 기준으로 하여 `SensorFusion` 및 `VectorQuaternionMatrix` 모듈 노드를 자동 필터링 및 제거합니다.
* `esp-dsp` 및 `SparkFun BMI270` 라이브러리에 대해 `.h`, `.hpp`, `.md` 파일 이외의 소스 코드를 무시하고 필터링합니다.
* 기존 한글 커뮤니티 사전(`labels`)을 바탕으로 다수결 매핑을 수행해 한글 군집 명칭을 상속하고 Obsidian 및 Wiki로 내보냅니다.
* 각 개별 디렉토리 하위의 불필요한 `graphify-out` 폴더들을 자동으로 영구 삭제 청소합니다.
* 최종 정제 그래프 데이터를 `src/T2_MSFM_250_wiki/T2_MSFM_250_wiki_graphify_out/graphify-out/` 경로로 최종 보존합니다.
