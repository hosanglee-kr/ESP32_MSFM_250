#!/bin/bash

# 1. 설정
LIB_NAME="esp-dsp_2"
TARGET_DIR="lib/${LIB_NAME}"
TEMP_DIR="temp_esp_dsp"
REPO_URL="https://github.com/espressif/esp-dsp.git"

# ESP32-S3 기준 컴파일러 설정 (PlatformIO 패키지 경로에 맞게 수정 필요할 수 있음)
# 일반적인 PlatformIO 도구 경로 예시: ~/.platformio/packages/toolchain-xtensa-esp32s3/bin/
CC="xtensa-esp32s3-elf-gcc"
AR="xtensa-esp32s3-elf-ar"

# 컴파일 플래그 (Arduino core 및 ESP32-S3 최적화)
CFLAGS="-O3 -mlongcalls -ffunction-sections -fdata-sections -fstrict-volatile-bitfields -Wno-error=unused-function -Wno-error=unused-variable -DESP_PLATFORM -DMBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\" -I$TARGET_DIR/include -I$TARGET_DIR/modules/common/include"

echo "### 1. 최신 esp-dsp 다운로드 중..."
git clone --recursive $REPO_URL $TEMP_DIR

# 2. 폴더 구조 생성
mkdir -p $TARGET_DIR/include
mkdir -p $TARGET_DIR/src

echo "### 2. 헤더 파일 및 모듈 복사 중..."
# 필요한 모듈 (common, dotprod, fft, filter, matrix 등)의 include 파일들을 복사
cp -r $TEMP_DIR/modules $TARGET_DIR/
find $TEMP_DIR/modules -name "*.h" -exec cp --parents {} $TARGET_DIR/include/ \;

echo "### 3. C 파일 컴파일 및 오브젝트 파일 생성 중..."
# 각 모듈의 .c 파일들을 찾아 컴파일
find $TEMP_DIR/modules -name "*.c" | while read -r src_file; do
    obj_name=$(basename "${src_file%.c}.o")
    echo "Compiling: $obj_name"
    $CC $CFLAGS -c "$src_file" -o "$TARGET_DIR/src/$obj_name"
done

# 4. 정적 라이브러리 생성 (선택 사항)
echo "### 4. 정적 라이브러리(libesp_dsp.a) 생성 중..."
cd $TARGET_DIR/src
$AR rcs ../libesp_dsp.a *.o
cd ../../../

# 5. 임시 파일 삭제
rm -rf $TEMP_DIR

echo "### 완료! 'lib/esp-dsp' 폴더를 확인하세요."