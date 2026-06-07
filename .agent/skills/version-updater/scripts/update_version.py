#!/usr/bin/env python3
"""
================================================================================
General Version Updater (Files & Contents)
================================================================================
이 스크립트는 프로젝트 폴더 내 파일명, 폴더명 및 소스 코드(주석, include 구문, 상수 등)에 포함된
특정 버전 문자열(예: '249')을 검색하여 새로운 버전 문자열(예: '250')로 일괄 치환합니다.

[주요 기능]
1. 복수 치환 지원: 여러 개의 검색/치환 쌍 지정 가능 (-f 249 -t 250 -f v2.4.9 -t v2.5.0)
2. 디렉토리 제외 목록 지원: .git, .pio, build 등 빌드/외부 폴더 스캔 제외로 속도 대폭 향상
3. 바이너리 보호 기능: 확장자 매칭 및 널 바이트(Null Byte) 검사 기반으로 바이너리 파일 감지 시
   내용 변경을 스킵하고 파일 이름 변경(rename)만 안전하게 처리하여 파일 깨짐 방지
4. 안전한 경로 변경: 파일/폴더 이름 변경 시 하위 경로를 먼저 처리하도록 정렬(Path Length Descending)
5. 인코딩 자동 감지 및 유지: UTF-8 및 CP949 인코딩을 자동 분석하여 원래 인코딩을 보존하며 파일 저장

[사용법 (Usage)]
1. 도움말 확인:
   python update_version.py --help

2. 드라이런 모드 (실제 변경 없이 미리보기):
   python update_version.py -d "C:\\path\\to\\dir" -f "249" -t "250" --dry-run

3. 실제 실행 (파일명, 폴더명 및 코드 내용 치환 적용):
   python update_version.py -d "C:\\path\\to\\dir" -f "249" -t "250"
================================================================================
"""

import os
import argparse
import sys

# 기본 제외 디렉토리 및 바이너리 확장자 정의
DEFAULT_IGNORE_DIRS = {'.git', '.pio', '.vscode', '.idea', 'build', 'node_modules', 'venv', '.venv', '__pycache__'}
DEFAULT_BINARY_EXTS = {
    '.png', '.jpg', '.jpeg', '.gif', '.ico', '.pdf', '.bin', '.elf', '.o',
    '.a', '.hex', '.db', '.exe', '.zip', '.tar', '.gz', '.woff', '.woff2',
    '.ttf', '.eot', '.mp3', '.mp4', '.wav', '.dll', '.so', '.dylib', '.suo', '.sdf'
}
DEFAULT_TEXT_EXTS = {
    '.py', '.c', '.cpp', '.h', '.hpp', '.txt', '.md', '.json', '.xml',
    '.ini', '.cfg', '.csv', '.sh', '.bat', '.ps1', '.html', '.css', '.js',
    '.ts', '.jsonl', '.yml', '.yaml', '.gitattributes', '.gitignore',
    'makefile', 'sconstruct', 'sconscript'
}

def is_binary_file(filepath, binary_exts, text_exts):
    """파일이 바이너리 파일인지 확인합니다."""
    _, ext = os.path.splitext(filepath.lower())
    if ext in binary_exts:
        return True
    if ext in text_exts or os.path.basename(filepath).lower() in text_exts:
        return False
    
    # 널 바이트(Null Byte) 검사 기반 폴백
    try:
        with open(filepath, 'rb') as f:
            chunk = f.read(1024)
            if b'\x00' in chunk:
                return True
    except Exception:
        return True
    return False

def replace_all(text, replacement_pairs):
    """입력 텍스트 내의 모든 검색 대상을 치환값으로 바꿉니다."""
    has_changes = False
    new_text = text
    for from_str, to_str in replacement_pairs:
        if from_str in new_text:
            new_text = new_text.replace(from_str, to_str)
            has_changes = True
    return new_text, has_changes

def update_version(target_dir, replacement_pairs, dry_run=False, ignore_dirs=None, ignore_exts=None, verbose=False):
    if not os.path.exists(target_dir):
        print(f"Error: Target directory '{target_dir}' does not exist.")
        sys.exit(1)

    print(f"Target Directory: {os.path.abspath(target_dir)}")
    print("Replacements:")
    for f, t in replacement_pairs:
        print(f"  '{f}' -> '{t}'")
    if dry_run:
        print("--- DRY RUN MODE (No changes will be applied) ---")

    # 스캔에 사용할 필터값 설정
    ignore_dirs_set = set(ignore_dirs) if ignore_dirs is not None else DEFAULT_IGNORE_DIRS
    binary_exts_set = set(ignore_exts) if ignore_exts is not None else DEFAULT_BINARY_EXTS

    content_updates = []  # 리스트: (file_path, new_content, encoding)
    renames = []          # 리스트: (source_path, target_path)

    for root, dirs, files in os.walk(target_dir, topdown=True):
        # 제외 디렉토리 프루닝 (In-place 수정으로 하위 순회 방지)
        dirs[:] = [d for d in dirs if d not in ignore_dirs_set]

        # 디렉토리 이름 변경 수집
        for d in dirs:
            dir_path = os.path.join(root, d)
            new_name, changed = replace_all(d, replacement_pairs)
            if changed:
                new_dir_path = os.path.join(root, new_name)
                renames.append((dir_path, new_dir_path))

        # 파일 스캔 및 내용 치환/이름 변경 수집
        for filename in files:
            file_path = os.path.join(root, filename)
            
            # 파일명 변경 여부 검사
            new_filename, filename_changed = replace_all(filename, replacement_pairs)
            new_file_path = os.path.join(root, new_filename) if filename_changed else file_path

            # 바이너리 여부 감지
            is_bin = is_binary_file(file_path, binary_exts_set, DEFAULT_TEXT_EXTS)

            has_content_changes = False
            content = None
            detected_encoding = None

            if not is_bin:
                # UTF-8 시도
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        content = f.read()
                    detected_encoding = 'utf-8'
                except UnicodeDecodeError:
                    # CP949 시도
                    try:
                        with open(file_path, 'r', encoding='cp949') as f:
                            content = f.read()
                        detected_encoding = 'cp949'
                    except Exception:
                        # CP949 ignore 시도
                        try:
                            with open(file_path, 'r', encoding='cp949', errors='ignore') as f:
                                content = f.read()
                            detected_encoding = 'cp949'
                        except Exception as e:
                            if verbose:
                                print(f"Warning: Failed to read text file {file_path}: {e}")
                            continue
                except Exception as e:
                    if verbose:
                        print(f"Warning: Failed to read text file {file_path}: {e}")
                    continue

                if content is not None:
                    new_content, has_content_changes = replace_all(content, replacement_pairs)
                    if has_content_changes:
                        content_updates.append((file_path, new_content, detected_encoding))

            # 파일명 변경 필요 시 추가
            if filename_changed:
                renames.append((file_path, new_file_path))

            if verbose:
                status = []
                if has_content_changes:
                    status.append("content update")
                if filename_changed:
                    status.append("rename")
                if is_bin:
                    status.append("binary")
                if status:
                    print(f"Scanned: {file_path} ({', '.join(status)})")

    # 1. 파일 내용 업데이트 실행
    print("\n--- Applying Content Updates ---")
    if not content_updates:
        print("No content updates needed.")
    else:
        for file_path, new_content, encoding in content_updates:
            if dry_run:
                print(f"[Dry-Run] Would update contents of: {file_path} (encoding: {encoding})")
            else:
                try:
                    with open(file_path, 'w', encoding=encoding) as f:
                        f.write(new_content)
                    print(f"Updated contents: {file_path}")
                except Exception as e:
                    print(f"Error updating contents of {file_path}: {e}")

    # 2. 파일/폴더 이름 변경 실행
    # 소스 경로 길이가 긴 순서(내림차순)로 정렬하여 하위 파일/폴더를 부모 폴더보다 먼저 이름 변경 처리
    renames.sort(key=lambda x: len(x[0]), reverse=True)

    print("\n--- Applying Renames ---")
    if not renames:
        print("No file or folder renames needed.")
    else:
        for src, dst in renames:
            if dry_run:
                print(f"[Dry-Run] Would rename: {src} -> {dst}")
            else:
                try:
                    if os.path.exists(src):
                        if os.path.exists(dst):
                            print(f"Warning: Rename target already exists, skipping: {src} -> {dst}")
                        else:
                            os.rename(src, dst)
                            print(f"Renamed: {src} -> {dst}")
                    else:
                        print(f"Warning: Rename source does not exist: {src}")
                except Exception as e:
                    print(f"Error renaming {src} to {dst}: {e}")

    print("\nVersion update process finished.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="General Version Updater (Files & Contents)")
    parser.add_argument("-d", "--dir", required=True, help="Target directory path")
    parser.add_argument("-f", "--from-version", dest="from_ver", action="append", required=True, help="Original version string(s) to replace")
    parser.add_argument("-t", "--to-version", dest="to_ver", action="append", required=True, help="New version string(s) to replace with")
    parser.add_argument("--dry-run", action="store_true", help="Preview changes without executing")
    parser.add_argument("--ignore-dirs", help="Comma-separated folder names to ignore (e.g., '.git,.pio,build')")
    parser.add_argument("--ignore-exts", help="Comma-separated extensions to ignore (e.g., '.bin,.elf,.png')")
    parser.add_argument("--verbose", action="store_true", help="Print verbose details of scanned files")

    args = parser.parse_args()

    # 치환 리스트 검사 및 쌍 확인
    if len(args.from_ver) != len(args.to_ver):
        print("Error: The number of from-version values (-f) must match the number of to-version values (-t).")
        sys.exit(1)

    replacement_pairs = list(zip(args.from_ver, args.to_ver))

    # 제외 디렉토리 커스텀 파싱
    ignore_dirs_list = None
    if args.ignore_dirs:
        ignore_dirs_list = {d.strip() for d in args.ignore_dirs.split(',') if d.strip()}

    # 제외 확장자 커스텀 파싱
    ignore_exts_list = None
    if args.ignore_exts:
        ignore_exts_list = {e.strip() if e.strip().startswith('.') else '.' + e.strip() for e in args.ignore_exts.split(',') if e.strip()}

    update_version(
        target_dir=args.dir,
        replacement_pairs=replacement_pairs,
        dry_run=args.dry_run,
        ignore_dirs=ignore_dirs_list,
        ignore_exts=ignore_exts_list,
        verbose=args.verbose
    )
