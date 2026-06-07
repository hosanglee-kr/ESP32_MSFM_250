import os
import shutil
from os.path import join, isdir, exists

Import("env")

def filter_esp_dsp(source, target, env):
    # 경로 변수 설정
    lib_deps_dir = env.subst("$PROJECT_LIBDEPS_DIR")
    board_env = env.subst("$PIOENV")
    project_dir = env.subst("$PROJECT_DIR")

    # 원본 위치와 대상 위치
    esp_dsp_src = join(lib_deps_dir, board_env, "esp-dsp")
    target_lib_dir = join(project_dir, "lib", "esp-dsp")

    # 1. 원본 소스가 있는지 확인 (PlatformIO가 다운로드 완료했는지)
    if not exists(esp_dsp_src):
        print(f"--> [esp-dsp] Source not found at {esp_dsp_src}. It will be processed on the next build.")
        return

    # 2. 이미 추출된 결과물이 있다면 스킵
    if exists(target_lib_dir):
        # 만약 내용물이 비어있지 않다면 이미 처리된 것으로 간주
        if os.listdir(target_lib_dir):
            return

    print(f"--> [esp-dsp] Extracting core modules to {target_lib_dir}...")

    # 3. 대상 폴더 생성
    os.makedirs(target_lib_dir, exist_ok=True)

    # 4. 'modules' 폴더 내부 내용물만 복사
    modules_path = join(esp_dsp_src, "modules")
    if exists(modules_path):
        try:
            for item in os.listdir(modules_path):
                s = join(modules_path, item)
                d = join(target_lib_dir, item)
                if isdir(s):
                    shutil.copytree(s, d, dirs_exist_ok=True)
                else:
                    shutil.copy2(s, d)

            # library.json이나 library.properties가 필요할 수 있으므로 메인 폴더에서도 복사 (선택 사항)
            for meta_file in ["library.json", "library.properties"]:
                meta_path = join(esp_dsp_src, meta_file)
                if exists(meta_path):
                    shutil.copy2(meta_path, join(target_lib_dir, meta_file))

            print("--> [esp-dsp] Successfully isolated modules and prepared library!")
        except Exception as e:
            print(f"--> [esp-dsp] Error during isolation: {e}")
    else:
        print("--> [esp-dsp] 'modules' folder not found in source!")

# 빌드 프로세스 시작 전(buildprog)에 실행되도록 훅 설정
env.AddPreAction("buildprog", filter_esp_dsp)
