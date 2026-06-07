/* ============================================================================
 * File: T246_MelGen_249.hpp
 * Summary: 오디오 및 IMU(가속도/자이로) 센서 데이터를 위한 Mel Filterbank 
 * 가중치 행렬 생성 클래스입니다. 주파수 도메인(Hz) 데이터를 
 * 인간의 청각 및 물리적 특성을 반영하는 Mel 스케일로 변환합니다.
 * 주요기능 :
 * - 오디오 신호용 Mel Filterbank 가중치 계산 (주파수 범위 가변 설정)
 * - IMU 신호용 Mel Filterbank 가중치 계산 (1600Hz 샘플링율, 800Hz Nyquist 고정)
 * - 평탄화된 1차원 배열(Flat Array) 형태로 메모리에 가중치 행렬 저장
 ============================================================================ 
*/

#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include "esp_log.h"

class MelFilterbankGenerator {
private:
    static constexpr const char* TAG = "T246_MelGen";

public:
    // 오디오 신호 처리를 위한 Mel Filterbank 가중치 행렬을 생성합니다.
    static void generateAudioMelFilterbank(float* p_melBankFlat, float sampleRate, uint8_t activeMelBands, uint16_t binsMax, uint16_t melPadded, uint16_t melBandsMax) {
        if (!p_melBankFlat) {
            ESP_LOGE(TAG, "generateAudioMelFilterbank: p_melBankFlat is null");
            return;
        }

        // --- 1. 주파수 영역 및 Mel 스케일 경계값 설정 ---
        const float nyquist    = sampleRate / 2.0f;                             // 나이퀴스트 주파수 계산
        const float v_lowFreq  = 100.0f;                                        // 오디오 분석 최소 주파수 (100Hz)
        const float v_highFreq = fminf(8000.0f, nyquist * 0.95f);               // 최대 주파수를 8kHz와 나이퀴스트 95% 중 작은 값으로 제한
        const float v_lowMel   = 1127.0f * log1p(v_lowFreq / 700.0f);            // 최저 주파수의 Mel 변환
        const float v_highMel  = 1127.0f * log1p(v_highFreq / 700.0f);           // 최고 주파수의 Mel 변환

        // --- 2. 각 Mel 밴드별 삼각 필터 생성 ---
        for (int m = 0; m < activeMelBands; m++) {
            // Mel 스케일 영역에서 현재 밴드의 좌측, 중심, 우측 주파수 포인트 계산
            const float v_mCenter = v_lowMel + (v_highMel - v_lowMel) * (m + 1) / (melBandsMax + 1);
            const float v_mLeft   = v_lowMel + (v_highMel - v_lowMel) * m       / (melBandsMax + 1);
            const float v_mRight  = v_lowMel + (v_highMel - v_lowMel) * (m + 2) / (melBandsMax + 1);

            // Mel 스케일 포인트를 다시 일반 주파수(Hz) 영역으로 역변환
            const float v_hzC = 700.0f * (expm1(v_mCenter / 1127.0f));
            const float v_hzL = 700.0f * (expm1(v_mLeft   / 1127.0f));
            const float v_hzR = 700.0f * (expm1(v_mRight  / 1127.0f));

            // --- 3. 각 FFT 주파수 빈(Bin)에 대한 가중치 계산 ---
            for (int b = 0; b < binsMax; b++) {
                // 현재 FFT 빈 인덱스(b)에 대응하는 실제 주파수(Hz) 계산
                const float v_hzB = (float)b * (nyquist / (float)binsMax);
                float v_weight = 0.0f;
                
                // 삼각 필터의 경계 조건에 맞춰 선형 보간 가중치(Linear Interpolation) 계산
                if (v_hzB >= v_hzL && v_hzB <= v_hzC) {
                    v_weight = (v_hzB - v_hzL) / (v_hzC - v_hzL);               // 삼각 필터의 상승 구간 (좌측 -> 중심)
                } else if (v_hzB > v_hzC && v_hzB <= v_hzR) {
                    v_weight = (v_hzR - v_hzB) / (v_hzR - v_hzC);               // 삼각 필터의 하강 구간 (중심 -> 우측)
                }
                
                // 2차원 행렬 구조를 1차원 플랫 배열에 맵핑하여 저장 [Row: 빈, Col: 밴드]
                p_melBankFlat[b * melPadded + m] = v_weight;
            }
        }
    }

    // IMU(가속도/자이로) 센서 신호 처리를 위한 Mel Filterbank 가중치 행렬을 생성합니다.
    static void generateImuMelFilterbank(float* p_melBankIMU, uint16_t binsImuMax, uint16_t melPadded, uint16_t melBandsMax) {
        if (!p_melBankIMU) {
            ESP_LOGE(TAG, "generateImuMelFilterbank: p_melBankIMU is null");
            return;
        }

        // --- 1. IMU 전용 고정 주파수 영역 및 Mel 스케일 경계값 설정 ---
        const float v_lowFreq_imu  = 10.0f;                                     // IMU 분석 최소 주파수 (10Hz)
        const float v_highFreq_imu = 800.0f;                                    // IMU 샘플링 1600Hz 기준 나이퀴스트 주파수(800Hz) 고정 적용
        const float v_lowMel_imu   = 1127.0f * log1p(v_lowFreq_imu / 700.0f);      // IMU 최저 주파수의 Mel 변환
        const float v_highMel_imu  = 1127.0f * log1p(v_highFreq_imu / 700.0f);     // IMU 최고 주파수의 Mel 변환

        // --- 2. 각 Mel 밴드별 삼각 필터 생성 ---
        for (int m = 0; m < melBandsMax; m++) {
            // Mel 스케일 영역에서 현재 밴드의 좌측, 중심, 우측 주파수 포인트 계산
            const float v_mCenter = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * (m + 1) / (melBandsMax + 1);
            const float v_mLeft   = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * m       / (melBandsMax + 1);
            const float v_mRight  = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * (m + 2) / (melBandsMax + 1);

            // Mel 스케일 포인트를 다시 일반 주파수(Hz) 영역으로 역변환
            const float v_hzC = 700.0f * (expm1(v_mCenter / 1127.0f));
            const float v_hzL = 700.0f * (expm1(v_mLeft   / 1127.0f));
            const float v_hzR = 700.0f * (expm1(v_mRight  / 1127.0f));

            // --- 3. 각 IMU 주파수 빈(Bin)에 대한 가중치 계산 ---
            for (int b = 0; b < binsImuMax; b++) {
                // 현재 IMU FFT 빈 인덱스(b)에 대응하는 실제 주파수(Hz) 계산 (최대 800Hz 기준)
                const float v_hzB = (float)b * (800.0f / (float)binsImuMax);
                float v_weight = 0.0f;
                
                // 삼각 필터의 경계 조건에 맞춰 선형 보간 가중치(Linear Interpolation) 계산
                if (v_hzB >= v_hzL && v_hzB <= v_hzC) {
                    v_weight = (v_hzB - v_hzL) / (v_hzC - v_hzL);               // 삼각 필터의 상승 구간
                } else if (v_hzB > v_hzC && v_hzB <= v_hzR) {
                    v_weight = (v_hzR - v_hzB) / (v_hzR - v_hzC);               // 삼각 필터의 하강 구간
                }
                
                // 2차원 행렬 구조를 1차원 플랫 배열에 맵핑하여 저장 [Row: 빈, Col: 밴드]
                p_melBankIMU[b * melPadded + m] = v_weight;
            }
        }
    }
};




/*
#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include "esp_log.h"

class MelFilterbankGenerator {
private:
    static constexpr const char* TAG = "MelGen";

public:
    // 함수설명: xxx (반환값: xxx)
    static void generateAudioMelFilterbank(float* p_melBankFlat, float sampleRate, uint8_t activeMelBands, uint16_t binsMax, uint16_t melPadded, uint16_t melBandsMax) {
        if (!p_melBankFlat) {
            ESP_LOGE(TAG, "generateAudioMelFilterbank: p_melBankFlat is null");
            return;
        }

        const float nyquist    = sampleRate / 2.0f;
        const float v_lowFreq  = 100.0f;
        const float v_highFreq = fminf(8000.0f, nyquist * 0.95f);  // 나이퀴스트의 95%로 클램프
        const float v_lowMel   = 1127.0f * log1p(v_lowFreq / 700.0f);
        const float v_highMel  = 1127.0f * log1p(v_highFreq / 700.0f);

        for (int m = 0; m < activeMelBands; m++) {
            const float v_mCenter = v_lowMel + (v_highMel - v_lowMel) * (m + 1) / (melBandsMax + 1);
            const float v_mLeft   = v_lowMel + (v_highMel - v_lowMel) * m       / (melBandsMax + 1);
            const float v_mRight  = v_lowMel + (v_highMel - v_lowMel) * (m + 2) / (melBandsMax + 1);

            const float v_hzC = 700.0f * (expm1(v_mCenter / 1127.0f));
            const float v_hzL = 700.0f * (expm1(v_mLeft   / 1127.0f));
            const float v_hzR = 700.0f * (expm1(v_mRight  / 1127.0f));

            for (int b = 0; b < binsMax; b++) {
                const float v_hzB = (float)b * (nyquist / (float)binsMax);
                float v_weight = 0.0f;
                if (v_hzB >= v_hzL && v_hzB <= v_hzC)      v_weight = (v_hzB - v_hzL) / (v_hzC - v_hzL);
                else if (v_hzB > v_hzC && v_hzB <= v_hzR)  v_weight = (v_hzR - v_hzB) / (v_hzR - v_hzC);
                p_melBankFlat[b * melPadded + m] = v_weight;
            }
        }
    }

    static void generateImuMelFilterbank(float* p_melBankIMU, uint16_t binsImuMax, uint16_t melPadded, uint16_t melBandsMax) {
        if (!p_melBankIMU) {
            ESP_LOGE(TAG, "generateImuMelFilterbank: p_melBankIMU is null");
            return;
        }

        const float v_lowFreq_imu  = 10.0f;
        const float v_highFreq_imu = 800.0f;    // Accel/Gyro 1600Hz → 나이퀴스트 800Hz
        const float v_lowMel_imu   = 1127.0f * log1p(v_lowFreq_imu / 700.0f);
        const float v_highMel_imu  = 1127.0f * log1p(v_highFreq_imu / 700.0f);

        for (int m = 0; m < melBandsMax; m++) {
            const float v_mCenter = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * (m + 1) / (melBandsMax + 1);
            const float v_mLeft   = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * m       / (melBandsMax + 1);
            const float v_mRight  = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * (m + 2) / (melBandsMax + 1);

            const float v_hzC = 700.0f * (expm1(v_mCenter / 1127.0f));
            const float v_hzL = 700.0f * (expm1(v_mLeft   / 1127.0f));
            const float v_hzR = 700.0f * (expm1(v_mRight  / 1127.0f));

            for (int b = 0; b < binsImuMax; b++) {
                const float v_hzB = (float)b * (800.0f / (float)binsImuMax);
                float v_weight = 0.0f;
                if (v_hzB >= v_hzL && v_hzB <= v_hzC)      v_weight = (v_hzB - v_hzL) / (v_hzC - v_hzL);
                else if (v_hzB > v_hzC && v_hzB <= v_hzR)  v_weight = (v_hzR - v_hzB) / (v_hzR - v_hzC);
                p_melBankIMU[b * melPadded + m] = v_weight;
            }
        }
    }
};
*/