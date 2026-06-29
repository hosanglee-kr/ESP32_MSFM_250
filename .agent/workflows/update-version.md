---
name: update-version
description: General version update in files & contents (both names and source code)
---

# Workflow: update-version

기존 코드 및 파일 경로의 버전을 일괄적으로 안전하게 교체하는 워크플로우입니다. `.agent/skills/version-updater/SKILL.md`에 설명된 스킬 지침에 따라 동작합니다.

## Commands & Workflows

### 1. 버전 치환 드라이런 (변경 사항 미리보기)
실제 파일과 디렉토리를 변경하지 않고 수정/이름 변경 예정인 파일 목록을 터미널로 미리 점검합니다.
```powershell
python .agent/skills/version-updater/scripts/update_version.py -d . -f "[이전버전]" -t "[신규버전]" --dry-run
```

### 2. 버전 치환 실행 (실제 적용)
프로젝트 내의 파일 이름, 폴더 이름 및 소스 코드의 문자열 치환을 실제로 실행하고 영구 보존합니다.
```powershell
python .agent/skills/version-updater/scripts/update_version.py -d . -f "[이전버전]" -t "[신규버전]"
```

### 3. 다중 치환 패턴 실행
버전 번호 외에 파일 및 코드 전체에 걸쳐 복수의 상호 연관 문자열들을 일괄 수정할 때 사용합니다.
```powershell
python .agent/skills/version-updater/scripts/update_version.py -d . -f "[이전버전]" -t "[신규버전]" -f "[이전태그]" -t "[신규태그]"
```
