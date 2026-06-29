---
name: version-updater
description: "Use to update, replace, and generalize project version numbers or target strings in filenames, directory names, and source file contents securely without corrupting binary files."
trigger: /update-version
---

# version-updater

프로젝트 내의 파일 내용, 파일 이름, 그리고 폴더 이름에 있는 버전 번호나 특정 대상 문자열을 일괄 검색하여 새 문자열로 안전하게 치환합니다.
이 스킬은 빌드 아티팩트나 이미지 파일 등 바이너리 파일이 검색 대상으로 인해 파괴되지 않도록 안전 장치를 갖추고 있습니다.

## 주요 기능

1. **안전한 바이너리 파일 처리**:
   - 파일 확장자 필터와 파일 헤더 널 바이트(Null Byte) 검출기를 결합하여 바이너리 파일을 감지합니다.
   - 바이너리 파일의 경우 내용 치환을 생략하고 디스크의 파일 이름 변경(`os.rename`)만 수행하여 파일 깨짐을 방지합니다.

2. **하위 경로 우선 이름 변경 (Bottom-up Renaming)**:
   - 파일 및 폴더의 경로 길이를 기준으로 내림차순(Desc) 정렬하여 처리합니다.
   - 부모 폴더가 변경되기 전에 가장 하위의 파일 및 서브 폴더 이름이 먼저 변경되므로 경로 변경 유실 현상이 발생하지 않습니다.

3. **속도 최적화 및 폴더 제외**:
   - `.git`, `.pio`, `build`, `node_modules` 등 대용량 혹은 불필요한 빌드 부산물 디렉토리는 검색 시 제외(Pruning)하여 매우 빠르게 탐색을 마칩니다.

4. **다양한 버전 동시 치환**:
   - 여러 개의 `-f/--from-version` 및 `-t/--to-version` 인수를 넘겨 여러 버전 패턴(예: `249` -> `250`, `v2.4.9` -> `v2.5.0`)을 한 번에 업데이트할 수 있습니다.

5. **인코딩 자동 보존**:
   - 파일 읽기 시 UTF-8 및 CP949 코덱을 순차 테스트하고 원래 파일이 가지고 있던 인코딩으로 다시 작성하여 문자 깨짐 현상을 피합니다.

---

## 사용법 (Usage)

슬래시 명령 `/update-version` 또는 직접 파이썬 스크립트를 실행하여 도구를 호출할 수 있습니다.

```powershell
# 1. 도움말 확인
python .agent/skills/version-updater/scripts/update_version.py --help

# 2. 드라이런 (실제 변경 없이 결과 확인)
python .agent/skills/version-updater/scripts/update_version.py -d "." -f "249" -t "250" --dry-run

# 3. 단일 버전 업데이트 실제 반영
python .agent/skills/version-updater/scripts/update_version.py -d "." -f "249" -t "250"

# 4. 복수 문자열 쌍 업데이트
python .agent/skills/version-updater/scripts/update_version.py -d "." -f "249" -t "250" -f "v2.4.9" -t "v2.5.0"

# 5. 특정 제외 경로 및 상세 로깅 적용
python .agent/skills/version-updater/scripts/update_version.py -d "." -f "249" -t "250" --ignore-dirs ".git,.pio,build,temp" --verbose
```

---

## CLI 옵션 목록

| 옵션명 | 단축명 | 필수 여부 | 설명 |
| :--- | :--- | :--- | :--- |
| `--dir` | `-d` | **필수** | 대상 디렉토리의 절대 경로 또는 상대 경로 |
| `--from-version` | `-f` | **필수** | 치환 대상이 될 기존 버전/문자열 (복수 개 지정 가능) |
| `--to-version` | `-t` | **필수** | 새로 바꿀 버전/문자열 (복수 개 지정 가능, `-f` 개수와 일치 필수) |
| `--dry-run` | | 선택 | 실제 파일 시스템을 변경하지 않고 치환 예정 사항 로그만 화면에 미리 보여줍니다. |
| `--ignore-dirs` | | 선택 | 쉼표(,)로 구분된 제외할 폴더 이름 리스트 (예: `.git,build`) |
| `--ignore-exts` | | 선택 | 쉼표(,)로 구분된 내용 검사를 스킵할 바이너리 확장자 리스트 (예: `.bin,.png`) |
| `--verbose` | | 선택 | 스캔 대상 모든 파일에 대한 결과 세부 내역을 터미널에 로깅합니다. |

---

## 에이전트 행동 가이드 (Agent Guidelines)

1. **상대경로 활용**: 대상 디렉토리는 기본적으로 프로젝트 루트 `.`를 사용하거나 사용자가 지정한 디렉토리로 안전하게 설정해야 합니다.
2. **드라이런 선행**: 버전 업데이트 작업 시 예기치 못한 치환 오류나 대량 손상을 미연에 방지하기 위해, **실제 쓰기 전 반드시 `--dry-run`으로 변경 사항 목록을 검출한 뒤 사용자에게 검토 및 승인을 받으십시오.**
3. **바이너리 주의**: 이미지 파일이나 컴파일된 라이브러리(`*.a`, `*.lib`, `*.bin`)는 본문 내용 검색 대상에서 명백히 제외되도록 스크립트 확장자 정의 상태를 점검하십시오.
