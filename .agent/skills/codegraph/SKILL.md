---
name: CodeGraph
description: "코드베이스의 심볼, 호출 관계(Call Graph), 구조를 분석하여 효율적인 코드 탐색과 변경 영향도 분석(Impact Analysis)을 수행하는 스킬입니다."
trigger: /codegraph
---

# CodeGraph

CodeGraph는 코드베이스의 의미적 구조(Semantic Structure), 심볼 관계, 호출 그래프를 미리 인덱싱하여, 대규모 코드베이스를 탐색할 때 토큰 소모와 도구 호출 횟수를 획기적으로 줄여주는 로컬 코드 지능화 스킬입니다.

## 주요 도구 (MCP Tools)

이 스킬은 다음과 같은 MCP 도구들을 제공합니다:

- `codegraph_explore`: 특정 쿼리나 심볼을 기준으로 관련된 심볼, 소스 코드 조각 및 관계를 한번에 탐색하여 가져옵니다. (코드 탐색 시 최우선 권장)
- `codegraph_search`: full-text 검색(FTS5)을 수행하여 코드베이스 전체에서 심볼을 즉시 검색합니다.
- `codegraph_callers` / `codegraph_callees`: 특정 함수의 Caller(호출자) 및 Callee(피호출자) 관계를 보여줍니다.
- `codegraph_impact`: 특정 심볼의 변경 시 영향을 받는 범위(Impact Radius)를 분석합니다.
- `codegraph_files`: 전체 파일 구조 및 파일 트리 정보를 조회합니다.
- `codegraph_status`: 현재 인덱싱 상태 및 프로젝트 정보를 조회합니다.

## 사용 규칙 (Rules)

1. **코드베이스 탐색 최우선 도구**: 코드 구조 분석이나 특정 로직 탐색 시 `grep`, `glob`, `read_file` 대신 `codegraph_explore`를 가장 먼저 호출합니다.
2. **에이전트 활용**: `codegraph_explore`를 통해 컨텍스트를 과도하게 차지하는 것을 방지하기 위해, 탐색 시에는 **Explore 에이전트(subagent)**를 생성하여 실행하는 것이 좋습니다.
3. **소스 파일 중복 조회 방지**: CodeGraph 도구가 관련 코드 조각이나 정의를 완전히 반환한 경우, 해당 소스 파일을 불필요하게 `read_file`로 다시 읽지 않습니다.
4. **변경 영향도 파악**: 주요 리팩토링이나 코드 변경 전에 `codegraph_impact`를 활용하여 영향 범위를 사전에 파악합니다.

## 설치 및 구성 (Installation)

### 1. CLI 설치
```powershell
# Windows (PowerShell)
irm https://raw.githubusercontent.com/colbymchenry/codegraph/main/install.ps1 | iex
```

### 2. 에이전트 연동
```bash
codegraph install
```

### 3. 프로젝트 인덱싱 초기화
```bash
codegraph init -i
```
