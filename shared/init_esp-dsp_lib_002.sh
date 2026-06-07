#!/bin/bash

# 1. 경로 설정
PROJECT_ROOT="$(pwd)/.."
LIB_NAME="esp-dsp"
TARGET_DIR="${PROJECT_ROOT}/lib/${LIB_NAME}"
TEMP_DIR="${PROJECT_ROOT}/lib/${LIB_NAME}_temp"
REPO_URL="https://github.com/espressif/esp-dsp.git"

# 2. 컴파일러 및 SDK 경로 설정
PIO_PKG_PATH="$HOME/.platformio/packages"
TOOLCHAIN_PATH="${PIO_PKG_PATH}/toolchain-xtensa-esp32s3/bin"
SDK_PATH="${PIO_PKG_PATH}/framework-arduinoespressif32/tools/sdk/esp32s3/include"

CC="${TOOLCHAIN_PATH}/xtensa-esp32s3-elf-gcc.exe"
AR="${TOOLCHAIN_PATH}/xtensa-esp32s3-elf-ar.exe"

# 3. 환경 정리 및 다운로드
## rm -rf "$TEMP_DIR"
rm -rf "$TARGET_DIR"
mkdir -p "$TARGET_DIR/include"
mkdir -p "$TARGET_DIR/src"

# echo "### [1/4] esp-dsp 최신 소스 다운로드..."
# git clone --recursive --depth 1 "$REPO_URL" "$TEMP_DIR"

echo "### [2/4] 헤더 및 모든 모듈 경로 수집..."
find "$TEMP_DIR/modules" \( -name "*.h" -o -name "*.inc" \) -exec cp {} "$TARGET_DIR/include/" \;
ALL_MODULE_DIRS=$(find "$TEMP_DIR/modules" -type d | sed 's/^/-I/')

echo "### [3/4] C 및 어셈블리(.S) 컴파일..."

# 공통 기본 플래그
BASE_FLAGS="-O3 -mlongcalls -ffunction-sections -fdata-sections -fstrict-volatile-bitfields \
-DESP_PLATFORM -DCONFIG_IDF_TARGET_ESP32S3=1 -DCONFIG_DSP_OPTIMIZED_ASM=1 \
-DCONFIG_DSP_MAX_FFT_SIZE=4096 -DCONFIG_DSP_OPTIMIZATION_AES3=1 \
-I$TARGET_DIR/include \
-I$SDK_PATH/config \
-I$SDK_PATH/esp_common/include \
-I$SDK_PATH/esp_hw_support/include \
-I$SDK_PATH/esp_rom/include \
-I$SDK_PATH/esp_system/include \
-I$SDK_PATH/esp_log/include \
-I$SDK_PATH/hal/include \
-I$SDK_PATH/hal/esp32s3/include \
-I$SDK_PATH/soc/include \
-I$SDK_PATH/soc/esp32s3/include \
-I$SDK_PATH/xtensa/include \
-I$SDK_PATH/xtensa/esp32s3/include \
-I$SDK_PATH/newlib/platform_include \
$ALL_MODULE_DIRS"

# C 전용 플래그: IRAM_ATTR 에러 방지용 정의만 포함
C_FLAGS="$BASE_FLAGS -DIRAM_ATTR="

# 어셈블리 전용 플래그: __ASSEMBLER__ 명시 및 전처리기 활성화
ASM_FLAGS="$BASE_FLAGS -D__ASSEMBLER__ -x assembler-with-cpp"

# 블랙리스트 (정말 불필요한 파일들)
BLACKLIST="aes3_tie_log.c"

find "$TEMP_DIR/modules" \( -name "*.c" -o -name "*.S" \) | while read -r src_file; do
    file_name=$(basename "$src_file")

    if [[ "$src_file" == *"test"* ]] || [[ "$src_file" == *"sim"* ]]; then continue; fi
    if [[ "$BLACKLIST" == *"$file_name"* ]]; then continue; fi

    parent_dir=$(basename $(dirname "$src_file"))
    obj_name="${parent_dir}_${file_name%.*}.o"

    echo -n "Compiling: $file_name ... "

    if [[ "$file_name" == *.S ]]; then
        # 어셈블리 파일 컴파일
        "$CC" $ASM_FLAGS -c "$src_file" -o "$TARGET_DIR/src/$obj_name"
    else
        # C 파일 컴파일 (절대 -D__ASSEMBLER__를 넣지 않음)
        "$CC" $C_FLAGS -c "$src_file" -o "$TARGET_DIR/src/$obj_name"
    fi

    if [ $? -eq 0 ]; then
        echo "OK"
    else
        echo "FAILED"
    fi
done

echo "### [4/4] 정적 라이브러리 생성..."
cd "$TARGET_DIR/src" || exit
if [ "$(ls -A)" ]; then
    "$AR" rcs ../libesp_dsp.a *.o
    echo "### [완료] lib/esp-dsp/libesp_dsp.a 생성됨."
else
    echo "### [오류] 오브젝트 파일 생성 실패."
fi

# cd "$PROJECT_ROOT" || exit
# rm -rf "$TEMP_DIR"
