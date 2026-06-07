/* 
============================================================================
 * File: T410_Def_013.hpp
 * Summary: SMEA-100 Global Configuration & Constants (JSON Dynamic Config Ready)
 * * [AI 메모: 리팩토링 및 방어 원칙 적용]
 * 1. 런타임 변경 불가 메모리/하드웨어 규격 -> _CONST 접미사 (정적 상수)
 * 2. 웹/JSON을 통해 변경될 동적 설정의 초기값 -> _DEF 접미사 (기본값)
 * 3. [보완] 밴드 개수 동적 할당 방어: 구조체 크기 고정을 위해 MAX 상수를
 * 도입하고, 실제 연산 개수는 _DEF 변수로 분리하여 OOM 및 정렬 파괴 방지.
 * 4. [계산식 연동 방어]: 의존성을 가지는 파라미터 자동 계산 수식 묶음 처리.
 * 5. [v013 추가] OTA/유지보수 상태(MAINTENANCE) 및 비동기 스토리지 상수 추가.
 ========================================================================== 
*/

#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"

namespace SmeaConfig {

    // ========================================================================
    // [1] 정적 상수 (Static Constants - _CONST 접미사)
    // 런타임 변경 불가. 메모리 할당(Array Size, PSRAM Pool) 및 하드웨어 핀맵 규격.
    // ========================================================================

    namespace System {
        inline constexpr uint32_t   SAMPLING_RATE_CONST      = 42000;                    // I2S 마이크 데이터 수집 주파수 (Hz)
        inline constexpr uint32_t   CHANNELS_CONST           = 2;                        // 수집 채널 수 (L/R 스테레오 모드)
        inline constexpr uint32_t   BITS_PER_SAMPLE_CONST    = 32;                       // 샘플당 비트 수 (ICS43434 32bit PCM 데이터)
        inline constexpr uint32_t   ALIGNMENT_BYTE_CONST     = 16;                       // ESP-DSP 가속기 SIMD 최적화를 위한 강제 메모리 정렬 크기 (Bytes)

        inline constexpr uint32_t   FFT_SIZE_CONST           = 1024;                     // 고속 푸리에 변환(FFT) 윈도우 샘플 크기
        inline constexpr uint32_t   MFCC_COEFFS_CONST        = 13;                       // 추출할 기본 MFCC 계수(Static) 개수

        inline constexpr uint32_t   MFCC_TOTAL_DIM_CONST     = MFCC_COEFFS_CONST * 3;    // MFCC 1D 텐서의 최종 차원 수
        inline constexpr uint32_t   MEL_BANDS_CONST          = 26;                       // 멜 필터뱅크(Mel-filterbank) 밴드 개수

        inline constexpr uint32_t   FEATURE_POOL_SIZE_CONST  = 100;                      // FSM 이중 큐 통신용 구조체 풀(Pool) 개수 (프레임 유실 방지용 여유분)

        inline constexpr float      PCM_32BIT_SCALE_CONST    = 1.0f / (float)(1ULL << 31);
        inline constexpr float      MATH_EPSILON_CONST       = 1e-6f;                    // 일반 부동소수점 0 나누기 방어용 Epsilon
        inline constexpr float      MATH_EPSILON_12_CONST    = 1e-12f;                   // 로그(Log) 및 제곱근(Sqrt) 0 진입 방어용 고정밀 Epsilon
        inline constexpr float      MS_PER_SEC_CONST         = 1000.0f;                  // 초(Sec) 단위를 밀리초(ms)로 역산하기 위한 상수
    }

    namespace Hardware {
        inline constexpr uint32_t   SERIAL_BAUD_CONST        = 115200;                   // 디버깅용 PC 시리얼 통신 보드레이트
        
        inline constexpr uint8_t    PIN_TRIGGER_CONST        = 7;                       // 외부 DC 제어용 하드웨어 트리거 입력 핀
        
        inline constexpr uint8_t    PIN_I2S_BCLK_CONST       = 4;                        // I2S Bit Clock (BCLK) 핀
        inline constexpr uint8_t    PIN_I2S_WS_CONST         = 5;                        // I2S Word Select (LRCLK) 핀
        inline constexpr uint8_t    PIN_I2S_DIN_CONST        = 6;                        // I2S Data In (마이크 수신) 핀
        inline constexpr int        I2S_PORT_NUM_CONST       = 0;                        // ESP32 하드웨어 I2S 할당 포트 번호
        
        inline constexpr int        I2S_DMA_BUF_COUNT_CONST  = 16;                       // Wi-Fi 블로킹 대응 I2S DMA 버퍼 개수 증가 (8 -> 16)
    }
    
    
    namespace Task {
        inline constexpr uint32_t   CAPTURE_STACK_SIZE_CONST = 8192;                     // 오디오 수집 태스크(Core 0) 스택 크기
        inline constexpr uint32_t   PROCESS_STACK_SIZE_CONST = 16384;                    // AI 및 신호 처리 태스크(Core 1) 스택 크기
        inline constexpr uint32_t   STORAGE_STACK_SIZE_CONST = 8192;                     // 비동기 스토리지 태스크 스택 크기
        inline constexpr uint8_t    CAPTURE_PRIORITY_CONST   = 10;                       // 오디오 수집 태스크 우선순위
        inline constexpr uint8_t    PROCESS_PRIORITY_CONST   = 5;                        // 신호 처리 태스크 우선순위
        inline constexpr uint8_t    STORAGE_PRIORITY_CONST   = 2;                        // 비동기 스토리지 우선순위 (가장 낮음)
        inline constexpr BaseType_t CORE_CAPTURE_CONST       = 0;                        // 오디오 수집 코어 락업
        inline constexpr BaseType_t CORE_PROCESS_CONST       = 1;                        // 신호 처리 코어 락업

        inline constexpr uint32_t   ALIVE_CHECK_MS_CONST     = 5000;                     // 메인 루프 시스템 헬스체크 주기 (ms)
        inline constexpr uint32_t   QUEUE_BLOCK_MS_CONST     = 100;                      // FSM 큐(Queue) 대기 타임아웃 (ms)
        inline constexpr uint32_t   WDG_YIELD_MS_CONST       = 1;                        // 와치독 양보 시간 (ms)
        inline constexpr uint32_t   REBOOT_DELAY_MS_CONST    = 500;                      // 소프트웨어 재부팅 전 안전 대기 시간 (ms)
        inline constexpr uint32_t   BOOT_DELAY_MS_CONST      = 1000;                     // 부팅 직후 센서 안정화 대기 시간 (ms)
        inline constexpr uint32_t   MAIN_LOOP_DELAY_MS_CONST = 10;                       // 메인 루프 양보 시간 (ms)
        
        inline constexpr uint32_t   LAZY_WRITE_DELAY_MS_CONST = 3000;
    }

    namespace DspLimit {
        inline constexpr int        MAX_MEDIAN_WINDOW_CONST  = 15;                       // Median 필터 배열 최대 크기 상한
    }

    namespace StorageLimit {
        inline constexpr uint8_t    DMA_SLOT_COUNT_CONST     = 3;                        // SD카드 고속 저장을 위한 DMA 버퍼 덩어리 개수
        inline constexpr uint32_t   DMA_SLOT_BYTES_CONST     = 16384;                    // DMA 슬롯 1개당 크기 (16KB)
        inline constexpr uint16_t   MAX_ROTATE_LIST_CONST    = 16;                       // 파일 로테이션 기록 최대 개수
        inline constexpr uint16_t   ROTATE_KEEP_MAX_CONST    = 8;                        // SD카드 용량 관리를 위해 유지할 최대 개수
        inline constexpr uint16_t   WATERMARK_HIGH_CONST     = 8;                        // 플러시 유발 캐시 임계치

        inline constexpr uint16_t   MAX_PATH_LEN_CONST       = 128;                      // 절대 경로 문자열 최대 길이
        inline constexpr uint16_t   MAX_PREFIX_LEN_CONST     = 16;                       // 파일 접두어 버퍼 최대 길이
        inline constexpr uint32_t   BYTES_PER_MB_CONST       = 1024 * 1024;              // (MB -> Bytes 변환)
        inline constexpr uint32_t   MS_PER_MIN_CONST         = 60 * 1000;                // (Min -> ms 변환)
        
        inline constexpr uint16_t   MAX_DIR_FILES_CONST      = 100;                      // 단일 폴더 내 O(N) 지연 방지용 파일 개수 상한
        inline constexpr uint32_t   PREALLOC_BYTES_CONST     = 10 * BYTES_PER_MB_CONST;  // SD카드 웨어레벨링 멈춤 방어용 파일 선할당(f_expand) 크기 (10MB)
    }

    namespace MlLimit {
        inline constexpr uint16_t   MAX_SEQUENCE_FRAMES_CONST = 128;                     // 시퀀스 텐서 프레임 상한선
    }

    namespace FeatureLimit {
        inline constexpr uint8_t    MAX_BAND_RMS_COUNT_CONST   = 8;                      // 멀티 밴드 배열 상한
        inline constexpr uint16_t   FIR_TAPS_CONST             = 63;                     // FIR 필터 차수(Taps)
        inline constexpr uint16_t   DELTA_HISTORY_FRAMES_CONST = 5;                      // MFCC Delta 링버퍼 크기

        inline constexpr uint32_t   STA_SAMPLES_CONST          = System::SAMPLING_RATE_CONST / 1000; 
        inline constexpr uint32_t   LTA_SAMPLES_CONST          = System::SAMPLING_RATE_CONST / 100;  

        inline constexpr float      MEL_SCALE_2595_CONST       = 2595.0f;                
        inline constexpr float      MEL_SCALE_700_CONST        = 700.0f;                 
        inline constexpr uint16_t   MAX_PEAK_CANDIDATES_CONST  = 128;                    
        inline constexpr uint8_t    TOP_PEAKS_COUNT_CONST      = 5;                      
        inline constexpr uint8_t    CEPS_TARGET_COUNT_CONST    = 4;                      
        inline constexpr float      CEPS_TOLERANCE_CONST       = 0.0003f;                
    }
    
    namespace Calib {
        inline constexpr uint32_t AUTO_IDLE_MIN_DEF        = 60;       // 자동 보정 진입 유휴 시간 (분, 0=사용안함)
        inline constexpr float    REF_FREQ_HZ_DEF          = 1000.0f;  // 보정 기준 주파수 (Hz)
        inline constexpr float    FILTER_MIN_FREQ_HZ_DEF   = 100.0f;   // 평탄화 하한 주파수 (Hz)
        inline constexpr float    FILTER_MAX_FREQ_HZ_DEF   = 16000.0f; // 평탄화 상한 주파수 (Hz)
        inline constexpr float    TARGET_GAIN_MAX_DEF      = 4.0f;     // 최대 보상 게인 (+12dB)
        inline constexpr float    TARGET_GAIN_MIN_DEF      = 0.25f;    // 최소 보상 게인 (-12dB)
        inline constexpr float    NORM_SAFE_THRESH_DEF     = 2.0f;     // FIR 계수 정규화 안전 임계치
    }
    
    namespace CalibLimit {
        inline constexpr uint8_t  MANUAL_RECORD_SEC_CONST = 10; 
        inline constexpr uint32_t RAW_BUFFER_BYTES_CONST = 
            System::SAMPLING_RATE_CONST * MANUAL_RECORD_SEC_CONST * System::CHANNELS_CONST * sizeof(float);
        
        inline constexpr float    GAIN_RATIO_MIN_CONST = 0.707f; 
        inline constexpr float    GAIN_RATIO_MAX_CONST = 1.414f; 
        
        inline constexpr uint16_t EQ_FIR_TAPS_CONST = 63;
        inline constexpr uint32_t WELCH_CHUNK_SAMPLES_CONST = System::FFT_SIZE_CONST;
        inline constexpr uint8_t  AUTO_STABLE_SEC_CONST = 10;
        
    }
    
    namespace DecisionLimit {
        inline constexpr uint8_t    MAX_TRIAL_COUNT_CONST      = 3;                      // 1회 트리거 시 검사 반복 횟수
    }

    namespace NetworkLimit {
        inline constexpr uint16_t   MAX_SSID_LEN_CONST         = 32;                     
        inline constexpr uint16_t   MAX_PW_LEN_CONST           = 64;                     
        inline constexpr uint16_t   MAX_IP_LEN_CONST           = 16;                     
        inline constexpr uint8_t    MAX_MULTI_AP_CONST         = 3;                 
        inline constexpr uint16_t   MAX_BROKER_LEN_CONST       = 64;                
        inline constexpr uint32_t   WIFI_MODE_SWITCH_DELAY_MS_CONST = 50;                
        inline constexpr uint32_t   WIFI_DISCONNECT_DELAY_MS_CONST  = 100;               
    }

    // ========================================================================
    // [2] 동적 설정 기본값 (Dynamic Defaults - _DEF 접미사)
    // ========================================================================

    namespace Dsp {
        inline constexpr float      WINDOW_MS_DEF                  = 25.0f;              
        inline constexpr float      HOP_MS_DEF                     = 10.0f;              
        inline constexpr float      NOTCH_FREQ_HZ_DEF              = 60.0f;              
        inline constexpr float      NOTCH_FREQ_2_HZ_DEF            = 120.0f;             
        inline constexpr float      NOTCH_Q_FACTOR_DEF             = 30.0f;              
        inline constexpr float      PRE_EMPHASIS_ALPHA_DEF         = 0.97f;              
        inline constexpr float      BEAMFORMING_GAIN_DEF           = 0.5f;               
        inline constexpr float      FIR_LPF_CUTOFF_DEF             = 20000.0f;           
        inline constexpr float      FIR_HPF_CUTOFF_DEF             = 100.0f;             
        inline constexpr int        MEDIAN_WINDOW_DEF              = 5;                  
        inline constexpr float      NOISE_GATE_THRESH_DEF          = 0.001f;             
        inline constexpr int        NOISE_LEARN_FRAMES_DEF         = 100;                
        inline constexpr float      SPECTRAL_SUB_GAIN_DEF          = 1.2f;               
    }

    namespace Feature {
        inline constexpr uint8_t   BAND_RMS_COUNT_DEF = 4;                               
        inline constexpr float     BAND_RANGES_DEF[FeatureLimit::MAX_BAND_RMS_COUNT_CONST][2] = {
            {10.0f, 150.0f}, 
            {150.0f, 1000.0f}, 
            {1000.0f, 5000.0f}, 
            {5000.0f, 20000.0f},
            {0.0f, 0.0f}, 
            {0.0f, 0.0f}, 
            {0.0f, 0.0f}, 
            {0.0f, 0.0f}
        };
		inline constexpr float      PEAK_AMPLITUDE_MIN_DEF 	   = 0.5f;                   
	    inline constexpr float      PEAK_FREQ_GAP_HZ_MIN_DEF   = 10.0f;                  
    }

    namespace Decision {
        inline constexpr float RULE_ENRG_THRESHOLD_DEF   = 0.00003f;                     
        inline constexpr float RULE_STDDEV_THRESHOLD_DEF = 0.12f;                        
        inline constexpr float TEST_NG_MIN_ENERGY_DEF    = 1e-8f;                        
        inline constexpr float STA_LTA_THRESHOLD_DEF     = 3.0f;						
        inline constexpr int   MIN_TRIGGER_COUNT_DEF     = 1;                            
        inline constexpr float NOISE_PROFILE_SEC_DEF     = 0.15f;                        
        inline constexpr float VALID_START_SEC_DEF       = 0.30f;                        
        inline constexpr float VALID_END_SEC_DEF         = 0.50f;                        
    }

    namespace Storage {
        inline constexpr uint8_t  PRE_TRIGGER_SEC_DEF   = 3;                               
        inline constexpr uint32_t ROTATE_MB_DEF         = 100;                             
        inline constexpr uint32_t ROTATE_MIN_DEF        = 60;                              
        inline constexpr uint32_t IDLE_FLUSH_MS_DEF     = 250;                             
    }

    namespace Path {
        inline constexpr char DIR_DATA_DEF[]            = "/t20_data";                       
        inline constexpr char DIR_RAW_DEF[]             = "/t20_data/raw";                   
        inline constexpr char SYS_CFG_JSON_DEF[]        = "/sys/config_013_01.json";         
        inline constexpr char FILE_INDEX_JSON_DEF[]     = "/sys/recorder_index.json";        
        inline constexpr char FILE_INDEX_TMP_DEF[]      = "/sys/recorder_index.tmp";         
        inline constexpr char WEB_INDEX_DEF[]           = "T4_013_001.html";                 
        inline constexpr char WWW_ROOT_DEF[]            = "/www";              
        
        inline constexpr char DIR_CALIB_DEF[]           = "/t20_data/calib";       
        inline constexpr char FILE_CALIB_JSON[]         = "/sys/calibration_013_01.json"; 

    }

    namespace Network {
        inline constexpr uint16_t HTTP_PORT_DEF         = 80;                            
        inline constexpr char     WS_URI_DEF[]          = "/ws";                         
        inline constexpr uint32_t WIFI_CONN_TIMEOUT_DEF = 4000;                          
        inline constexpr uint32_t WIFI_RETRY_MS_DEF     = 10000;                         
        inline constexpr uint32_t LARGE_BUF_SIZE_DEF    = 8192;                          

        inline constexpr char     NTP_SERVER_1_DEF[]    = "pool.ntp.org";                
        inline constexpr char     NTP_SERVER_2_DEF[]    = "time.nist.gov";               
        inline constexpr char     TZ_INFO_DEF[]         = "KST-9";                       
        inline constexpr uint32_t NTP_TIMEOUT_MS_DEF    = 5000;                          

        inline constexpr uint8_t  WIFI_MODE_DEF         = 3;                             
        inline constexpr const char* AP_SSID_DEF        = "SMEA_100_AP";                 
        inline constexpr const char* AP_PW_DEF          = "12345678";                    
        inline constexpr const char* AP_IP_DEF          = "192.168.4.1";                 
    }
    
    namespace Mqtt {
        inline constexpr uint32_t RETRY_INTERVAL_MS_DEF = 5000;                          
        inline constexpr uint16_t DEFAULT_PORT_DEF      = 1883;                          
        inline constexpr char     TOPIC_RESULT_DEF[]    = "smea100/inspection/result";   
        // 네이티브 MQTT 고급 기능 기본값
        inline constexpr uint8_t  PROTOCOL_VER_DEF      = 4; // 5: MQTT 5.0, 4: MQTT 3.1.1
        inline constexpr uint8_t  QOS_DEF               = 1; // QoS 1 (At least once)
        inline constexpr char     TOPIC_LWT_DEF[]       = "smea100/status/lwt";
    }


}

// ========================================================================
// [3] 시스템 열거형 (Enum Classes)
// ========================================================================

enum class SystemState : uint8_t {
    INIT = 0,       
    READY,          
    MONITORING,     
    RECORDING,      
    CALIBRATING,    // 캘리브레이션 전용 격리 상태
    MAINTENANCE,    // OTA 업데이트 및 시스템 버스 락 해제를 위한 안전 격리 상태 (이 상태에선 I2S/SD 접근 전면 금지)
    ERROR           
};

enum class DetectionResult : uint8_t {
    PASS = 0,       
    RULE_NG = 1,    
    TEST_NG = 2,    
    ML_NG = 3       
};

enum class SystemCommand : uint8_t {
    CMD_MANUAL_RECORD_START, 
    CMD_MANUAL_RECORD_STOP,  
    
    CMD_LEARN_NOISE,         
    CMD_REBOOT,
    CMD_OTA_START,             // OTA 진입 이벤트 트리거
    CMD_OTA_END,               // OTA 종료 이벤트 트리거
    CMD_CALIB_MANUAL_START, 
    CMD_CALIB_AUTO_START,   
    CMD_CALIB_STOP,
    // [v013 추가] Web UI/UX 튜닝 상태 동기화용 커맨드
    CMD_TUNING_PREVIEW,        // 메모리에만 설정 반영 (핫스왑)
    CMD_TUNING_SAVE,           // 플래시에 영구 저장
    CMD_TUNING_CANCEL          // 이전 저장 상태로 롤백
};


