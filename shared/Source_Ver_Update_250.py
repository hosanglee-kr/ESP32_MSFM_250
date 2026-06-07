#!/usr/bin/env python3
"""
================================================================================
General Version Updater (Files & Contents)
================================================================================
이 스크립트는 프로젝트 폴더 내 파일명 및 소스 코드(주석, include 구문, 상수 등)에 포함된
특정 버전 문자열(예: '249')을 검색하여 새로운 버전 문자열(예: '250')로 일괄 치환합니다.

[사용법 (Usage)]
1. 도움말 확인:
   python update_version_250.py --help

2. 드라이런 모드 (실제 변경 없이 미리보기):
   python update_version_250.py -d "C:\\path\\to\\dir" -f "249" -t "250" --dry-run

3. 실제 실행 (파일명 및 코드 내용 치환 적용):
   python update_version_250.py -d "C:\\path\\to\\dir" -f "249" -t "250"
================================================================================
"""

import os
import argparse
import sys

def update_version(target_dir, from_ver, to_ver, dry_run=False):
    # 대상 디렉토리 유효성 검사
    if not os.path.exists(target_dir):
        print(f"Error: Directory '{target_dir}' does not exist.")
        sys.exit(1)

    print(f"Target Directory: {target_dir}")
    print(f"Replacing '{from_ver}' with '{to_ver}'")
    if dry_run:
        print("--- DRY RUN MODE (No changes will be applied) ---")

    # 대상 디렉토리부터 하위 폴더까지 모든 파일을 순회 (os.walk 활용)
    for root, dirs, files in os.walk(target_dir):
        for filename in files:
            old_path = os.path.join(root, filename)

            # 1. 파일 내용 읽기 시도 (UTF-8 인코딩 우선 적용, 실패 시 CP949로 예외 처리)
            try:
                with open(old_path, 'r', encoding='utf-8') as f:
                    content = f.read()
            except UnicodeDecodeError:
                try:
                    with open(old_path, 'r', encoding='cp949', errors='ignore') as f:
                        content = f.read()
                except Exception as e:
                    print(f"Failed to read {old_path}: {e}")
                    continue
            except Exception as e:
                print(f"Failed to read {old_path}: {e}")
                continue

            # 2. 파일 내용 중 이전 버전 문자열이 있는지 체크하여 치환 데이터 생성
            has_changes = from_ver in content
            new_content = content.replace(from_ver, to_ver) if has_changes else content

            # 3. 파일 이름 자체에 버전 정보가 포함되어 있다면 새 파일 이름 생성
            new_filename = filename.replace(from_ver, to_ver)
            new_path = os.path.join(root, new_filename)
            filename_changed = new_filename != filename

            # 4. 파일 이름이 변경되었거나 내용 치환이 필요한 경우 처리 진행
            if has_changes or filename_changed:
                if dry_run:
                    # [드라이런] 변경 예정 사항 로그 출력
                    action = []
                    if filename_changed:
                        action.append(f"Rename: {filename} -> {new_filename}")
                    if has_changes:
                        action.append(f"Replace content occurrences of '{from_ver}'")
                    print(f"[Dry-Run] {old_path} -> {' & '.join(action)}")
                else:
                    # [실제 실행] 파일 업데이트 및 이름 변경 수행
                    try:
                        # 치환된 내용으로 신규 파일 저장
                        with open(new_path, 'w', encoding='utf-8') as f:
                            f.write(new_content)

                        # 파일 이름이 바뀌었다면 이전 파일 삭제 처리
                        if filename_changed:
                            os.remove(old_path)
                            print(f"Renamed & Updated: {filename} -> {new_filename}")
                        else:
                            print(f"Updated content: {filename}")
                    except Exception as e:
                        print(f"Error processing {old_path}: {e}")

    print("\nProcess finished.")

if __name__ == "__main__":
    # 실행 매개변수 파서 정의
    parser = argparse.ArgumentParser(description="General Version Updater (Files & Contents)")
    parser.add_argument("-d", "--dir", required=True, help="Target directory path")
    parser.add_argument("-f", "--from-version", dest="from_ver", required=True, help="Original version string (e.g. 249)")
    parser.add_argument("-t", "--to-version", dest="to_ver", required=True, help="New version string (e.g. 250)")
    parser.add_argument("--dry-run", action="store_true", help="Preview changes without executing")

    args = parser.parse_args()
    # 버전 치환 프로세스 시작
    update_version(args.dir, args.from_ver, args.to_ver, args.dry_run)

