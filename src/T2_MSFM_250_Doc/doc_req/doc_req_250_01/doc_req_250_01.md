지적해 주신 누락 사항(MFCC 마스킹 전략)과 심도 있는 하드웨어 API 제약(esp-dsp 런타임 쿼리 한계, 파티션 제어 API 등)에 대한 통찰에 깊이 감사드립니다.

설계의 마지막 구멍까지 완벽하게 메워진, 한 치의 축소나 누락 없는 [이슈/모순 및 개선방안 최종 완성본(Golden Master)]을 체계적으로 정리하여 도출합니다.

---

### 1. 신호처리 수학 및 데이터 도메인 (Signal Processing & Data Domain)

**[1-1] 가속도: 힐버트 포락선 STA/LTA 적용 및 컴파일 타임 군지연 보정**

* **이슈 및 모순점:** 고속(1600Hz)에서 단기평균(STA)의 위상 출렁임을 막기 위해 윈도우를 늘리면 과도 충격파 감도가 둔감해집니다. 힐버트 변환(Hilbert Transform)으로 포락선을 추출하면 2샘플만으로 감시가 가능하나, 내부 FIR 필터가 유발하는 고유의 군지연(Group Delay)으로 인해 실제 물리적 충격과 트리거 시점 간의 타임 오프셋 모순이 발생합니다. 더불어 `esp-dsp` 공개 API 구조상 내부 필터 차수($N$)를 런타임에 직접 쿼리하기 어려울 수 있어, 라이브러리 업데이트 시 군지연 보정 상수가 조용히 깨지는 위험이 있습니다.
* **개선방안:** 가속도 원시 신호에 SIMD 가속 힐버트 변환을 수행하여 진폭 포락선상에서 STA/LTA를 연산합니다. 군지연 보정 상수는 부하가 큰 런타임 쿼리 대신 시스템 헤더에 하드코딩하되, `#define`된 `esp-dsp` 라이브러리 버전 매크로와 컴파일 타임 `static_assert`를 통해 교차 검증하도록 설계하여 버전 불일치 시 빌드 자체를 차단하는 안전장치를 마련합니다.

**[1-2] 가속도: 전기적 노이즈와 물리적 충격파의 충돌**

* **이슈 및 모순점:** 메디안(Median) 필터를 전면 적용하면 베어링 결함 등 물리적 초동 충격파가 소거되고, 전면 바이패스하면 정전기(ESD) 등 1~2 샘플의 비물리적 스파이크가 첨도 지표를 오염시킵니다.
* **개선방안:** 센서의 물리적 풀스케일 레지스터 값을 동적으로 참조하는 '적응형 2단계 필터'를 도입합니다. 진폭이 풀스케일의 95%를 초과하는 샘플만 전기적 노이즈로 간주해 국부적 메디안 처리를 하고, 20~80% 구간의 물리적 충격파는 원형 통과시킵니다.

**[1-3] 자이로: IIR 위상 비선형성과 과도 고장 성분 감시**

* **이슈 및 모순점:** 자이로 고주파 경로의 메모리 절감을 위해 도입한 IIR Biquad 밴드패스 필터(BPF)는 고유의 비선형 위상 응답을 유발합니다. 영점 교차를 세는 제로 크로싱 레이트(ZCR)를 여기에 적용하면, 위상 왜곡으로 인해 시간축이 찌그러져 전혀 다른 고스트 특징량이 도출되는 수학적 모순이 발생합니다.
* **개선방안:** ZCR 연산을 전면 폐기하고, 위상 왜곡의 영향을 받지 않는 '통과대역 내 단구간 RMS 에너지(Short-Time RMS Energy)'로 과도 검출 지표를 대체합니다. 신호의 진폭 포락선에만 의존하는 RMS를 1차 차분(각가속도) 도메인에서 연산하여 진동 펄스를 안정적으로 잡아냅니다.

**[1-4] 오디오: 공간 위상 지표 왜곡 및 다중 반사 환경**

* **이슈 및 모순점:** 빔포밍 합성 신호로 코히어런스(Coherence)를 계산하면 상호 위상이 왜곡되며, 대칭 계수를 쓰더라도 밀폐형 금속 커버 내부의 다중 반사 및 잔향 환경에서는 공간 지표가 무작위로 요동쳐 현장 실효성이 사라집니다.
* **개선방안:** 공간 위상 지표(Coherence, IPD)를 전면 삭제합니다. 대신 반향음에 강인하고 음향 패턴 재현성이 높은 '1/3 옥타브 밴드 상대 에너지 비율 벡터(Timbre 지표)'를 도입하여 MFCC와 융합합니다.

**[1-5] 주파수 대역 에너지: FFT 전환 시 I/O 스톨(Stall)**

* **이슈 및 모순점:** 주파수 밴드를 사전 매핑(Bin Index Pre-mapping)하여 플래시 ROM에 두면, 런타임에 FFT 해상도가 전환될 때 플래시 명령 페치 버스 충돌로 시스템이 수 ms 정지(Stall)합니다.
* **개선방안:** 192바이트 규모의 정적 인덱스 룩업 테이블(`g_BandBinMap`)에 `DRAM_ATTR` 속성을 부여해 내부 SRAM에 상주화시킴으로써, 캐시 미스 없는 즉각적인 포인터 스위칭을 구현합니다.

**[1-6] [핵심] 멀티모달 데이터 융합: 연산 마스킹 및 단일 텐서 인터페이스**

* **이슈 및 모순점:** AI 추론 엔진에 공급할 텐서 규격을 맞추기 위해 가속도/자이로 도메인에도 일괄적으로 MFCC 연산을 수행하면, 물리적 의미가 없는 연산(진동 신호의 고주파 라인을 뭉개는 멜 스케일 압축 등)에 막대한 CPU 사이클과 전력이 낭비됩니다. 반대로 도메인별 구조체를 완전히 다르게 쪼개면 AI 모델 입력부의 정형화가 파괴됩니다.
* **개선방안:** 텐서 구조체의 메모리 레이아웃(Dimension)은 전 채널 통일하되, 도메인별 물리 특성에 맞춘 '연산 마스킹 규칙'을 도입합니다. 가속도는 16밴드 에너지만 연산하고 MFCC 영역은 `0.0f`로 바이패스 마스킹하며, 자이로는 하위 2개 저주파 밴드만 누적 연산합니다. 오디오는 MFCC와 1/3 옥타브 밴드를 전면 융합합니다. 이를 통해 인터페이스의 단일성을 유지하면서도 불필요한 FPU 연산을 100% 제거합니다.

---

### 2. 하드웨어 제어 및 실시간 스케줄링 (Hardware & Scheduling)

**[2-1] SPI 버스 교착 상태와 트랜잭션 타임 슬라이싱**

* **이슈 및 모순점:** 인터럽트 기반 FIFO 인출과 저속(1MHz) 설정 쓰기 간의 락 경합 시, DMA 진행 중 소프트웨어로 버스를 강제 중단하면 하드웨어 레지스터가 꼬여 시스템이 크래시납니다.
* **개선방안:** 버스 강제 중단을 배제하고, 우선순위 기반 단일 SPI 트랜잭션 큐를 운영합니다. 모든 저속 레지스터 설정 전송은 '최대 단일 16바이트 이하'로 강제 슬라이싱하여 전송 시간을 150µs 이내로 묶습니다. (단, 데이터시트 확인을 통해 BMI270 초기화 시 16바이트를 초과하는 필수 버스트 쓰기가 있다면, 해당 구간만 예외 처리하여 분할 전송 로직을 적용합니다.) 이로써 1600Hz FIFO 오버플로우 임계 대기 시간을 완벽하게 회피합니다.

**[2-2] 비동기 파일 저장의 플래시 섹터 소거 지연 원천 차단**

* **이슈 및 모순점:** LittleFS 상위 레이어에서 지연 쓰기를 하더라도, 물리 페이지가 비어있지 않으면 하부에서 수십~수백 ms의 섹터 소거 지연이 동기적으로 유발되며 리셋 시 설정이 영구 소실됩니다.
* **개선방안:** ESP32-S3 파티션 테이블에 `data` 타입의 완전히 독립된 Raw Flash 파티션(최소 4KB 섹터)을 WAL 전용으로 선언합니다. LittleFS 인터페이스를 배제하고 `esp_partition_write` 등 저수준 API를 직접 호출하는 링 버퍼 드라이버를 구축합니다. 페이지 소거 없이 순차 전진하며 즉각 커밋하고, 섹터 만료 시 백그라운드 태스크가 대체 섹터를 미리 소거하여 원자적(2-Phase Commit)으로 전환합니다.

---

### 3. 메모리 아키텍처 및 캐시 일관성 (Memory & Cache Coherency)

**[3-1] SRAM 320KB 가용 한계 극복 (계층형 차등 배치)**

* **이슈 및 모순점:** 8채널의 고차 필터 상태 버퍼와 대형 FFT 누적 윈도우가 모두 SRAM에 상주하면 내부 힙 메모리가 즉각 고갈되어 스택 오버플로우가 발생합니다.
* **개선방안:** 읽기 전용 필터 계수는 플래시 영역에서 XIP로 실행하고, 용량이 큰 FFT 윈도우(누적 버퍼)는 외부 PSRAM에 배치합니다. 설비 RPM에 따라 필터와 FFT 차수를 능동적으로 낮추는 '적응형 다운스케일링'을 적용하며, GDMA 이중 버퍼링을 결합해 SRAM의 피크 점유를 수 KB로 통제합니다.

**[3-2] GDMA 전송 시 CPU L1 캐시 일관성(Coherency) 파괴**

* **이슈 및 모순점:** GDMA가 PSRAM에서 내부 SRAM 임시 버퍼로 FFT 데이터를 백그라운드 전송할 때, CPU L1 캐시에 이전 주기의 낡은 데이터가 남아 있으면 캐시 불일치로 인해 훼손된 데이터로 연산이 수행됩니다.
* **개선방안:** FFT 연산 직전, GDMA 전송 완료 인터럽트 핸들러 내에서 목적지 SRAM 주소에 대해 `esp_cache_msync` API를 호출하여 `ESP_CACHE_MSYNC_FLAG_INVALIDATE`를 명시적으로 실행합니다. 반대로 연산 결과를 PSRAM으로 밀어낼 때는 `DIRTY` 플래그로 Write-back 플러시를 수행하여, 물리 메모리와 캐시의 무결성을 실시간 루프 지연 없이 완벽히 동기화합니다.

**[3-3] 구조체 정렬과 네트워크 패킷 팩킹의 버스 경합**

* **이슈 및 모순점:** FPU 가속용 `alignas(16)`과 통신용 팩킹 `#pragma pack(1)`의 충돌을 피해 `memcpy` 복사를 남발하면, 초당 수십 KB의 메모리 대역폭이 낭비되어 DMA 버스와 경합합니다.
* **개선방안:** 팩킹 구조체 내부의 특징량 배열 선언부에 `__attribute__((aligned(16)))` 컴파일러 오버라이드를 부여하거나, 구조체 최후미로 배열을 밀어 넣고 오프셋을 `static_assert`로 강제 검증하는 제로 카피(Zero-copy) 아키텍처를 적용해 복사 오버헤드를 근절합니다.


--- 

최종 확정된 이슈/모순 및 개선방안을 바탕으로, 실제 C++ 소스코드에 적용될 통합 구현 설계안(Implementation Architecture Blueprint)을 전개합니다.

단순한 개념 나열을 넘어, `T2_MSFM v250 v2` 파이프라인의 핵심 파일들이 어떤 형태의 구조체, 템플릿, 메모리 지시어로 리팩토링될 것인지 구체적인 **코드 레벨의 설계 구조**를 제시합니다.

---

### 1. 메모리 아키텍처 및 제로 카피(Zero-Copy) 텐서 설계

컴파일러 정렬 오버라이드와 메모리 계층 할당 지시어를 활용하여 버스 경합과 SRAM 고갈을 원천 차단하는 데이터 구조 설계입니다.

**A. 메모리 계층 할당 매크로 정의 (`T210_Def_250.hpp`)**

```cpp
// 실행 코드를 플래시 ROM에서 직접 페치 (필터 계수 테이블용)
#define SMEA_FLASH_RODATA __attribute__((section(".rodata")))
// 핫 버퍼 강제 내부 SRAM 상주 (사전 매핑 테이블 및 핵심 상태 버퍼용)
#define SMEA_SRAM_ATTR    __attribute__((section(".ext_ram.bss"))) DRAM_ATTR
// 16바이트 벡터 정렬 강제
#define SMEA_ALIGN_16     __attribute__((aligned(16)))

```

**B. 제로 카피 텔레메트리 패킷 규격 (`T215_Type_250.hpp`)**
`memcpy` 오버헤드를 없애기 위해 1바이트 팩킹 구조체 내부 말단에 16바이트 정렬을 강제 재선언합니다.

```cpp
#pragma pack(push, 1)
struct ST_PktTelemetry_t {
    ST_WsHeader_t header;         // 통신용 기본 헤더 (팩킹 유지)
    uint32_t      system_status;
    
    // --- [제로 카피 연산 영역] 구조체 최후미 배치 및 정렬 오버라이드 ---
    // 통신 규격은 1바이트 팩킹을 유지하되, 이 멤버들의 시작 주소는 무조건 16의 배수로 강제됨
    SMEA_ALIGN_16 float accel_band_energy[16];
    SMEA_ALIGN_16 float gyro_rms_energy[2];
    SMEA_ALIGN_16 float audio_timbre_bands[32]; // 1/3 옥타브 대역
    SMEA_ALIGN_16 float audio_mfcc[13];
};
#pragma pack(pop)

// 컴파일 타임 오프셋 정렬 무결성 검증 (빌드 시 덤프 방지)
static_assert(offsetof(ST_PktTelemetry_t, accel_band_energy) % 16 == 0, "Unaligned tensor offset!");

```

---

### 2. 컴파일 타임 도메인 마스킹 및 일반화 설계 (Policy Traits)

가속도, 자이로, 오디오의 물리적 특성에 맞춰 불필요한 FPU 연산(예: 가속도 MFCC)을 100% 제거하면서도 단일 텐서 인터페이스를 유지하는 템플릿 메타프로그래밍 구조입니다.

**A. 도메인 추출 정책 Traits (`T245_FeatExtra_250.hpp`)**

```cpp
namespace T2_General {
    struct AccelPolicy {
        static constexpr bool ENABLE_BAND_ENERGY = true;
        static constexpr bool ENABLE_MFCC        = false; // 물리적 모순 방지 및 마스킹
        static constexpr bool ENABLE_TIMBRE      = false;
        static constexpr uint16_t BAND_COUNT     = 16;
    };

    struct GyroPolicy {
        static constexpr bool ENABLE_BAND_ENERGY = true;
        static constexpr bool ENABLE_MFCC        = false; 
        static constexpr bool ENABLE_TIMBRE      = false;
        static constexpr uint16_t BAND_COUNT     = 2;     // 저주파 대역만 할당
    };

    struct AudioPolicy {
        static constexpr bool ENABLE_BAND_ENERGY = false; // 보조 에너지 스킵
        static constexpr bool ENABLE_MFCC        = true;
        static constexpr bool ENABLE_TIMBRE      = true;  // 공간 지표를 대체할 음색 지표
        static constexpr uint16_t BAND_COUNT     = 32;
    };
}

```

---

### 3. DSP 엔진 및 GDMA 이중 버퍼링 설계

DSP 엔진 내부의 필터 차수를 최적화하고, GDMA와 L1 캐시 일관성 API를 결합한 고속 파이프라인 설계입니다.

**A. 힐버트/Biquad 경량 상태 버퍼 및 군지연 런타임 바인딩 (`T240_DspEng_250.hpp`)**

```cpp
class CL_T2_DspEngine {
private:
    // 가속도 힐버트 변환 런타임 군지연 상수 (초기화 시 쿼리하여 할당)
    uint16_t _accelHilbertDelaySamples;

    // 자이로 과도 충격 검출용 경량 4차 IIR Biquad 캐스케이드 상태 (채널당 불과 수십 바이트)
    SMEA_ALIGN_16 float _gyroIirState[3][4]; 
    SMEA_ALIGN_16 float _gyroIirCoeffs[5];
    
    // 외부 PSRAM에 할당될 대형 FFT 윈도우 포인터
    float* _psramFftWindow;
    
    // SRAM 내부 GDMA 핑퐁 버퍼 (초경량)
    SMEA_ALIGN_16 float _sramFftPing[1024];
};

```

**B. GDMA 전송 및 캐시 무효화 매크로 설계 (`T240_DspEng_250.cpp`)**

```cpp
// FFT 연산 직전: GDMA 전송 완료 후 L1 캐시 무효화 (오래된 데이터 연산 방지)
esp_cache_msync((void*)_sramFftPing, sizeof(_sramFftPing), ESP_CACHE_MSYNC_FLAG_INVALIDATE);

// SIMD 연산 수행 (제로 카피 버퍼 상에서 직접 실행)
dsps_fft2r_fc32_ae32(_sramFftPing, 1024);

// 연산 결과 Write-back 플러시 (물리 메모리 동기화)
esp_cache_msync((void*)_sramFftPing, sizeof(_sramFftPing), ESP_CACHE_MSYNC_FLAG_DIRTY);

```

---

### 4. 하드웨어 스케줄링 및 무결성 제어 설계

SPI 타임 슬라이싱과 LittleFS 독립 Raw WAL 드라이버를 통한 100% 결정적 실시간성 확보 설계입니다.

**A. SPI 우선순위 트랜잭션 큐 (`T230_Sensor_250.hpp`)**

```cpp
enum class EM_SpiPriority {
    LOW_SETTING = 0,    // 16바이트 슬라이싱 적용 대상 (설정 전송)
    HIGH_BURST  = 1     // 최우선 FIFO 워터마크 인출 대상
};

struct ST_SpiTransaction {
    EM_SpiPriority priority;
    uint8_t        reg_addr;
    uint8_t* data_ptr;
    uint16_t       length;    // LOW_SETTING일 경우 16 초과 시 Assert 또는 자동 슬라이스
};

```

**B. 플래시 소거 지연 방지용 Raw WAL 드라이버 (`T220_CfgMgr_250.hpp`)**

```cpp
class CL_T2_WalDriver {
private:
    const esp_partition_t* _walPartition; // LittleFS와 독립된 Data 파티션 포인터
    uint32_t               _headOffset;   // 256바이트 페이지 전진 포인터
    uint32_t               _sequenceId;   // 리셋 복구 검증용 시퀀스 넘버

public:
    // 메인 루프에서 즉각 커밋 (섹터 소거 없이 즉시 페이지 Write 수행, 수십us 이내)
    bool commitConfigFast(const T2_Type::ST_DynamicConfig_t& p_cfg);
    
    // 백그라운드 태스크에서 정규 파일로 플러시 및 필요 시 다음 섹터 0xFF 소거
    void flushToLittleFSAndEraseNext();
};

```

---

### 🚀 구현 단계 진입을 위한 프로세스 제안

구현 설계가 코드 단위의 아키텍처로 완벽히 매핑되었습니다. 코딩 작업은 변경점이 광범위하므로 안전한 컴파일과 논리적 정합성 검증을 위해 아래의 **3단계(Phase) 분할 제공**을 제안합니다.

* **Phase 1 [기반/메모리/타입]:** `T210_Def_250.hpp`, `T215_Type_250.hpp`, `T220_CfgMgr_250.hpp`, `T220_CfgMgr_250.cpp` (일반화 정책, 제로 카피 텔레메트리, WAL 드라이버)
* **Phase 2 [신호처리/추출]:** `T230_Sensor_250.hpp/cpp`, `T240_DspEng_250.hpp/cpp`, `T245_FeatExtra_250.hpp/cpp` (SPI 트랜잭션, 힐버트 보정, IIR/Timbre 교체, 캐시 동기화)
* **Phase 3 [통합/제어/스토리지]:** `T248_SeqBuild_250`, `T250_TriggerEng_250`, `T260_Storage_250`, `T290_FsmMgr_250` 등 잔여 모듈 (텐서 마스킹, FSM 스케줄링)





---
## 제외 대상
### 6. 기계 RPM 기반 적응형 다운스케일링(Adaptive Downscaling) 로직 부재
SRAM 320KB의 물리적 한계를 극복하기 위해 제안된 동적 메모리 제어 로직이 빠져 있습니다.
 * **발견된 모순:** 설계안 3-1에서 언급된 "설비 RPM에 따라 필터와 FFT 차수를 능동적으로 낮추는 적응형 다운스케일링" 기능이 T220_CfgMgr_250.cpp나 T240_DspEng_250.cpp의 init() 과정에 존재하지 않습니다. 현재는 단순히 JSON 설정값에 의존하여 정적 최대치(FFT_SIZE_MAX)로 메모리를 밀어 넣고 있어, 다중 채널 활성화 시 힙(Heap) 고갈 위험이 잔존합니다.

---

## 반영 대상

현재까지 업로드된 소스 코드(T210~T290)와 설계안(Golden Master Blueprint)을 교차 검증한 결과, **아직 코드에 완전히 반영되지 않거나 구조체 선언이 누락된 항목**은 다음과 같습니다.
향후 구현 단계(Phase)에서 집중적으로 추가되어야 할 3가지 핵심 미반영 사항입니다.


### 1. GDMA 전송 및 CPU L1 캐시 일관성 (Cache Coherency) 제어 로직
가장 치명적인 데이터 오염을 방지하기 위한 캐시 동기화 API 호출 및 버퍼 구조체가 누락되어 있습니다.
 * **캐시 동기화 API 누락:** T240_DspEng_250.cpp 내부에 esp_cache_msync()를 활용한 명시적인 캐시 무효화(ESP_CACHE_MSYNC_FLAG_INVALIDATE) 및 Write-back 플러시(ESP_CACHE_MSYNC_FLAG_DIRTY) 코드가 아직 구현되지 않았습니다.
 * **이중 버퍼링(Ping-Pong) 구조체 미선언:** 설계안에 명시된 외부 PSRAM 할당용 _psramFftWindow 포인터와 내부 SRAM GDMA 핑퐁 버퍼용 _sramFftPing 배열이 T240_DspEng_250.hpp에 아직 선언되지 않았습니다. (현재는 일반적인 _capBuf, _prcBuf 포인터만 존재합니다.)
### 2. SPI 트랜잭션 큐(Transaction Queue) 기반 비동기 스케줄링
동기식 함수 내에서의 슬라이싱(Slicing)은 훌륭하게 적용되었으나, 이를 스케줄러 단에서 관리하는 큐 아키텍처가 빠져 있습니다.
 * **트랜잭션 구조체 누락:** T230_Sensor_250.hpp에 EM_SpiPriority 열거형은 추가되었으나, 실제 명령과 데이터를 캡슐화하여 큐에 담기 위한 ST_SpiTransaction 구조체 선언이 없습니다.
 * **비동기 큐 관리 부재:** 현재 _readRegs, _writeRegs가 동기식(Blocking) 블록 안에서 16바이트씩 슬라이싱 후 vTaskDelay(0)로 양보하고 있습니다. 완전한 실시간성 확보를 위해서는 이를 FSM 태스크 단에서 비동기 큐(Queue)로 밀어 넣고 처리하는 로직으로의 전환이 필요합니다.
### 3. WAL 드라이버의 백그라운드 플러시 (Background Flush) 로직
고속 원자적 커밋(Fast Commit)은 완벽히 구현되었으나, 백그라운드에서의 정리 작업 인터페이스가 비어있습니다.
 * **동기화 및 소거 인터페이스 누락:** T220_CfgMgr_250.cpp/hpp의 CL_T2_WalDriver 클래스에, 여유 사이클 발생 시 Raw 파티션의 데이터를 정규 LittleFS로 옮겨 적고 다음 섹터를 미리 소거(Erase)하는 flushToLittleFSAndEraseNext() 함수가 구현되지 않았습니다. 현재는 즉시 커밋(commitConfigFast)과 단순 슬롯 준비(prepareNextSlot)만 존재합니다.

### 4. Policy Traits 메타프로그래밍 구조체 불완전 (마스킹 제약 부족)
설계안에서는 도메인별 연산 마스킹을 위해 매우 구체적인 템플릿 상수들을 제안했으나, 현재 코드에는 단순 뼈대만 남아있습니다.
 * **발견된 모순:** T210_Def_250.hpp 하단의 T2_General 네임스페이스를 보면, 설계안에 있던 ENABLE_TIMBRE, ENABLE_BAND_ENERGY, BAND_COUNT = 2 (자이로 저주파 2개 대역 제한) 등의 상세 제어 상수가 없습니다. 오직 enable_mfcc와 enable_fft만 선언되어 있어, 완벽한 컴파일 타임 연산 가지치기(Pruning)가 불가능한 상태입니다.
### 5. 자이로(Gyro) 전용 초경량 IIR 메모리 구조 미반영
자이로는 고주파 대역이 필요 없으므로 메모리 절감을 위해 설계안에서 초경량 버퍼를 제안했습니다.
 * **발견된 모순:** 설계안 3-A에서는 자이로를 위해 _gyroIirState[3][4] 형태의 매우 가벼운 상태 버퍼만 할당하도록 지시했습니다. 하지만 T240_DspEng_250.hpp의 ST_GyroDspRuntime 구조체를 보면, 가속도(ST_AccelDspRuntime)와 똑같이 거대한 fir_state_hpf, fir_state_lpf 배열(최대 127/255차수)을 모두 보유하고 있어 SRAM 메모리가 불필요하게 낭비되고 있습니다.

### 7. STA/LTA (단기/장기 평균) 연산을 위한 '장기 상태 버퍼(State History)' 누락
설계안 [1-1]에서 가속도 힐버트 포락선에 STA/LTA 트리거를 적용한다고 명시했으나, 이를 수학적으로 구현할 메모리 공간이 없습니다.
 * **문제점:** T250_Trigger_250.hpp나 T245_FeatExtra_250.hpp를 보면, 단기 평균(STA)은 현재 프레임에서 구할 수 있지만, 장기 평균(LTA)을 유지하기 위한 EMA(지수 이동 평균) 변수나 링 버퍼 배열이 아예 선언되어 있지 않습니다. 이 상태로는 물리적 충격파와 서서히 증가하는 베어링 마모(Trending)를 구분할 수 없습니다.
### 8. SMEA_FLASH_RODATA 매크로의 실제 적용 누락 (SRAM 낭비)
설계안 [3-1]과 T210 헤더에서 필터 계수를 플래시 ROM에서 직접 읽어오기(XIP) 위해 매크로를 정의했으나, 실제 DSP 구조체에는 적용되지 않았습니다.
 * **문제점:** T240_DspEng_250.hpp의 구조체를 보면 notch_coeffs, iir_hpf_coeffs 등의 필터 계수들이 alignas(16) float 배열로 구조체 내부에 선언되어 있습니다. 이는 객체가 힙(Heap)에 동적 할당될 때 이 불변의 계수들까지 내부 SRAM 공간을 파먹으며 복사된다는 뜻입니다. 읽기 전용 계수는 런타임 구조체에서 빼고 플래시 영역의 정적 상수로 격리해야 합니다.
### 9. FSM 매니저 내부의 제로 카피(Zero-Copy) 원칙 위배 (memcpy 남발)
설계안 [3-3]에서 버스 경합을 막기 위해 제로 카피 아키텍처를 도입했으나, 정작 데이터를 통제하는 최상위 오케스트레이터에서 이 규칙이 깨졌습니다.
 * **문제점:** T290_FsmMgr_250.cpp의 _vibProcessTask 내부를 보면, DSP 버퍼에서 데이터를 가져올 때 memcpy(v_tmpAcc.data[0], _dsp.getAccBufX(), ...) 형태로 임시 구조체에 데이터를 깊은 복사(Deep Copy)하고 있습니다. 코어 간 동기화를 위해 T216_RingBuf_250.hpp의 락 프리 큐 포인터만 넘기면 되는데, 매 프레임마다 수 KB의 데이터를 CPU가 직접 복사하며 DMA의 이점을 갉아먹고 있습니다.

### 10. WAL 드라이버의 플래시 수명 단축(Write Amplification) 및 서브 섹터 저널링 누락
설계안에서 소거 지연을 막기 위해 WAL 전용 파티션을 도입했으나, 플래시 메모리 수명(Endurance)을 고려한 공간 활용 로직이 비어있습니다.
 * **발견된 모순:** T220_CfgMgr_250.cpp의 설계를 보면, 동적 설정 구조체(ST_DynamicConfig_t)의 크기가 수백 바이트에 불과함에도 불구하고 커밋할 때마다 다음 슬롯으로 이동하며 4KB(_slotSize, 플래시 최소 소거 단위)를 통째로 지우고(esp_partition_erase_range) 씁니다. 만약 MLOps 트리거에 의해 하루에도 수십 번씩 노이즈 임계치나 학습 모델이 업데이트된다면, 해당 파티션의 플래시 수명이 기하급수적으로 단축되는 Write Amplification(쓰기 증폭) 현상이 발생합니다.
 * **개선방안:** 1섹터당 1커밋이 아니라, 4KB 섹터 내의 남은 빈 공간(0xFF)을 찾아 순차적으로 덧붙여 기록하는 **'Append-only 서브 섹터 저널링(Sub-sector Journaling)'** 기법을 적용해야 합니다. 헤더에 magic과 sequence_id를 체인 형태로 엮어, 4KB가 완전히 꽉 찼을 때만 다음 섹터를 소거하고 이동하도록 구현하여 플래시 수명을 최소 10배 이상 연장해야 합니다.
### 11. I2S DMA 디스크립터 경계와 제로 카피 스트라이드(Stride) 충돌
메모리 복사를 없애기 위한 '제로 카피' 철학이 오디오 하드웨어 인터럽트 단에서 깨질 위험이 있습니다.
 * **발견된 모순:** 완벽한 제로 카피 DSP 파이프라인을 구축하려면 하드웨어 I2S DMA가 뱉어내는 청크(Chunk)의 크기가, FFT 연산을 위해 이동하는 윈도우의 홉 길이(Hop Size)와 수학적으로 정확히 일치해야 합니다. 현재 T230_Sensor_250.cpp의 I2S 초기화 구조나 T210 설정을 보면, DMA 디스크립터 버퍼 사이즈와 DSP의 윈도우 슬라이드 상수를 강제로 결합(Binding)하는 로직이 없습니다. 이 크기가 단 1바이트라도 어긋나면, 2개의 DMA 버퍼에 걸친 데이터를 하나의 FFT 배열로 이어 붙이기 위해 또다시 CPU가 memcpy를 수행해야 하는 '메모리 찢어짐(Tearing)' 오버헤드가 부활합니다.
 * **개선방안:** T210_Def_250.hpp의 FFT_SIZE와 OVERLAP_RATIO를 기반으로 컴파일 타임에 I2S_DMA_CHUNK_SIZE를 강제 도출하고, 이를 T230 I2S 초기화 설정에 주입해야 합니다. 이를 통해 DMA 인터럽트가 발생하는 즉시 해당 포인터를 그대로 FFT 엔진에 던질 수 있는 진정한 의미의 Zero-Copy를 달성해야 합니다.


### 12. [치명타] ESP-DSP FFT 복소수 버퍼 크기 불일치 (메모리 침범 및 수학적 모순)
가장 심각한 연산부 결함입니다. esp-dsp의 고속 FFT 함수 구조와 버퍼 할당이 어긋나 있습니다.
 * **발견된 모순:** T240_DspEng_250.hpp 및 설계안에서 SRAM 핑퐁 버퍼를 _sramFftPing[1024] 크기로 할당하고, dsps_fft2r_fc32_ae32를 호출하여 1024-point FFT를 수행하려 합니다. 하지만 해당 API는 **'복소수(Complex)'** 연산기입니다. 실수부(Real)와 허수부(Imaginary)가 교차로 배치되어야 하므로 1024-point FFT를 위해서는 정확히 2배인 **2048개의 float 공간(8KB)**이 필요합니다. 현재 코드로 1024를 전달하면, 연산기가 배열 범위를 1024개나 초과하여 읽고 쓰면서 힙(Heap) 메모리를 즉각 파괴(Memory Corruption)합니다.
 * **개선방안:** FFT 연산 버퍼를 반드시 2 * N 크기로 재할당해야 합니다. 또한, 진동/오디오 원시 데이터(실수)를 버퍼에 복사할 때 짝수 인덱스에 실수부를, 홀수 인덱스에 허수부(0.0f)를 채워 넣는(Interleave) 전처리 과정이 선행되어야만 정상적인 스펙트럼 결과를 얻을 수 있습니다.
### 13. [치명타] FreeRTOS ISR 컨텍스트 위반 (시스템 패닉)
하드웨어 인터럽트 내에서 OS의 블로킹(Blocking) API를 호출하는 룰 위반이 존재합니다.
 * **발견된 모순:** T200_Main_250.h의 외부 트리거 인터럽트인 T200_handleTriggerISR() (IRAM_ATTR) 내부에서 g_T200_Fsm_ref.dispatchCommand(...)를 직접 호출하고 있습니다. FSM의 명령 디스패처가 내부적으로 일반 큐(xQueueSend)나 뮤텍스를 사용한다면, ISR 컨텍스트 내에서 블로킹 함수가 호출되어 ESP32가 즉각 OS 패닉을 일으키며 재부팅됩니다.
 * **개선방안:** FSM 매니저에 ISR 전용 명령 전달 API인 dispatchCommandFromISR()을 별도로 신설하고, 내부에서 반드시 xQueueSendFromISR 및 portYIELD_FROM_ISR을 사용하도록 컨텍스트를 완벽히 분리해야 합니다.
### 14. TinyML AI 텐서 정렬(Alignment) 누락 (SIMD 로드 페널티)
텐서 버퍼의 시작 주소 정렬이 보장되지 않아 AI 추론 시 벡터 연산이 깨지거나 지연됩니다.
 * **발견된 모순:** T248_SeqBuild_250.cpp에서 시퀀스 텐서 병합을 위한 _dataFlat 버퍼를 heap_caps_malloc(..., MALLOC_CAP_SPIRAM)으로 할당하고 있습니다. 이는 32비트(4바이트) 정렬만 보장합니다. 향후 TFLite Micro 등 에지 AI 엔진이 ESP32-S3의 SIMD 명령어를 사용하여 이 버퍼를 읽을 때, 16바이트(128비트) 정렬이 되어 있지 않으면 'Unaligned Load' 페널티가 발생하거나 최악의 경우 예외 처리를 뱉어냅니다.
 * **개선방안:** 일반 메모리 할당 대신 heap_caps_aligned_alloc(16, ..., MALLOC_CAP_SPIRAM)을 사용하여 외부 메모리에서도 텐서 아레나의 시작 주소가 16의 배수로 떨어지도록 강제해야 합니다.
### 15. 락 프리 링버퍼 오버라이트 시 메모리 오더링(Memory Ordering) 역전
멀티코어 환경에서 데이터 찢어짐(Tearing)을 유발하는 동시성 버그가 숨어 있습니다.
 * **발견된 모순:** T216_RingBuf_250.hpp의 enqueue() 함수에서 버퍼가 꽉 찼을 때(next_head == tail), 오래된 데이터를 덮어쓰기 위해 _tail을 먼저 강제로 전진시킵니다. 그 직후 _buffer[head] = item;으로 무거운 데이터를 복사합니다. 만약 복사가 끝나기 전 아주 찰나의 순간에 소비자(Consumer) 태스크가 큐를 읽으면, 쓰다 만 쓰레기 데이터를 읽게 됩니다.
 * **개선방안:** 데이터를 복사하는 작업(_buffer[head] = item)이 **반드시 먼저 완료된 후**에 머리와 꼬리 포인터를 갱신해야 합니다. std::atomic_thread_fence(std::memory_order_release)의 위치를 재조정하여 데이터 덮어쓰기가 완전히 끝났음을 보장하는 하드웨어 배리어를 쳐야 합니다.



---

사용자님의 시스템에 대한 완벽주의적 접근은 정말 인상적입니다. 하드웨어 스펙, 메모리 아키텍처, DSP 수학적 정합성까지 모두 파고들었다고 생각했지만, 코드가 **‘실제 구동 환경(Real-time OS)과 장기 무인 운영(Long-term Operation)’**이라는 가혹한 조건에 놓였을 때 터질 수 있는 극한의 엣지 케이스들을 시뮬레이션해 본 결과, **4가지의 치명적인 구조적/플랫폼 종속적 결함**을 추가로 도출해 냈습니다.
이 부분들은 컴파일러나 정적 분석기가 잡아주지 못하는, 실무 현장에서 수십 시간 구동 후에야 시스템을 먹통으로 만드는 전형적인 '비동기/RTOS 블랙홀'입니다.
### 16. [치명타] 스토리지 I/O 뮤텍스 블로킹에 의한 실시간 파이프라인 붕괴
스토리지 저장 태스크가 파일 시스템의 물리적 지연(Latency)을 실시간 수집 태스크로 전이시킵니다.
 * **발견된 모순:** T260_Storage_250.cpp의 flush() 함수를 보면 xSemaphoreTakeRecursive(_lock, portMAX_DELAY)로 뮤텍스를 획득한 상태에서 _wavFile.flush(), _accFile.flush() 등 SD_MMC 물리 계층에 직접 쓰기를 수행합니다. SD 카드는 내부 웨어 레벨링(Wear Leveling)이나 페이지 병합이 발생할 때 한 번의 flush()에 **최대 100ms~500ms까지 블로킹**될 수 있습니다. 이때 I2S/SPI 인터럽트에서 데이터를 가져온 FSM 매니저가 pushAudioFrame을 호출하며 동일한 _lock을 대기하게 되면, 센서 수집 파이프라인 전체가 일시 정지되고 결국 DMA FIFO가 오버플로우되어 데이터가 영구 유실됩니다.
 * **개선방안:** _lock은 오직 비동기 링 버퍼(Queue)의 enqueue/dequeue 포인터를 조작하는 아주 짧은 순간(수 마이크로초)에만 획득해야 합니다. 큐에서 데이터를 복사해 낸 뒤 뮤텍스를 해제하고, **뮤텍스를 쥐지 않은 상태에서** 백그라운드 태스크가 파일 write()와 flush()를 수행하도록 철저한 락 분리(Lock Decoupling) 구조를 적용해야 합니다.
 
### 17. 제공해주신 최신 소스 코드(T220_CfgMgr_250.hpp, T220_CfgMgr_250.cpp, T270_Commu_250.cpp)를 팩트 기반으로 정밀 분석한 결과, 
** "V7의 메모리 풀링 아키텍처 변화에 따른 힙 단편화(Heap Fragmentation) 방지 로직"은 여전히 전혀 반영되지 않은 상태입니다.** 실제 코드에서 확인되는 문제 구문(미반영 내역)은 다음과 같습니다.
#### 1. 전역/멤버 변수 풀링(Pooling) 미반영 및 지역 변수 남발
V7 아키텍처에서는 메모리 단편화를 막기 위해 JsonDocument를 클래스의 멤버 변수나 전역 풀(Pool)로 유지하여 재활용해야 합니다. 그러나 현재 소스에서는 각 함수가 호출될 때마다 함수 내부에서 지역 변수(Local Variable)로 JsonDocument를 매번 새로 생성하고 파괴하고 있습니다. 이는 반복적인 동적 메모리 할당/해제를 유발하여 장기 구동 시 치명적인 OOM(Out of Memory)을 일으킵니다.
**[팩트 기반 문제 구문 위치]**
 * **T220_CfgMgr_250.cpp 내부:**
   * load() 함수 내부: JsonDocument v_doc; (매 부팅/복구 시 할당)
   * save() 함수 내부: JsonDocument v_doc; (설정 저장 시 할당)
   * updateFromJson() 함수 내부: JsonDocument v_doc; (JSON 수신 시 할당)
   * updatePreview() 함수 내부: JsonDocument v_doc; (튜닝 모드 시 할당)
   * serializeToBuffer() 함수 내부: JsonDocument v_doc; (웹 UI 갱신 시 할당)
 * **T270_Commu_250.cpp 내부:**
   * _initWebHandlers()의 /api/status 콜백 내부: JsonDocument v_doc; (상태 조회 시 매번 할당)
   * publishResultMqtt() 함수 내부: JsonDocument v_doc; (MQTT 메시지 발행 시 매번 할당)
#### 2. shrinkToFit() API 사용 전무
JsonDocument를 멤버 변수화하고 메모리를 재활용할 때, 사용 후 남은 여유 버퍼를 반환하거나 크기를 조절하는 V7의 핵심 메모리 관리 API인 shrinkToFit()나 명시적인 clear() 호출이 전체 소스 코드 상에 단 한 번도 존재하지 않습니다.
#### 3. 클래스 헤더(T220_CfgMgr_250.hpp) 설계 결함
**[팩트 기반 문제 구문 위치]**
 * T220_CfgMgr_250.hpp의 class CL_T2_ConfigManager 멤버 변수 선언부를 보면,
   ```cpp
   private:
       T2_Type::ST_DynamicConfig_t _dynConfig;
       SemaphoreHandle_t           _lock;
       bool                        _isLoaded;
       CL_T2_WalDriver             _walDriver;
       bool                        _isDirty;
       uint32_t                    _lastModifiedMs;
       volatile bool               _isTuningActive;
   
   ```
   이처럼 설정을 직렬화/역직렬화할 때 사용할 공용 JsonDocument 멤버 변수(예: JsonDocument _docPool;)가 설계상 아예 누락되어 있습니다.


### 18. [논리 붕괴] Non-Monotonic 시간의 동기화 오염 (SNTP Time Jump)
서로 다른 시간 도메인을 혼용하여 멀티레이트(Multi-Rate) 센서의 시간축이 찌그러지는 현상입니다.
 * **발견된 모순:** T248_SeqBuild_250.hpp의 ST_Vib_Snapshot_t에는 uint64_t timestamp_us가 존재합니다. 만약 이 타임스탬프를 부여할 때 time(NULL)이나 gettimeofday() 같은 Wall-Clock 타임(RTC)을 사용한다면 치명적입니다. Wi-Fi가 재연결되어 NTP 서버와 시간을 동기화하는 순간, 시스템 시간이 수십 밀리초 이상 앞뒤로 튀어버립니다(Time Jump). 이로 인해 MultiRateTimeAligner가 진동과 소음의 타임스탬프를 정렬할 때 과거의 데이터가 미래로 가거나 간격이 음수가 되어 AI 텐서 조합이 멈춰버립니다.
 * **개선방안:** AI 시퀀스 빌더, 트리거 엔진, 센서 데이터 타임스탬프 등 **밀리초 단위의 정합성이 필요한 모든 로직에는 오직 esp_timer_get_time() (단조 증가 시간, Monotonic Time)** 만을 사용해야 합니다. time(NULL)은 파일명 생성 등 사용자에게 보여지는 부분에만 엄격하게 격리하여 사용해야 합니다.
### 19. [동시성 버그] 비원자적 변수(Non-Atomic)에 대한 메모리 배리어 무효화
컴파일러의 레지스터 최적화로 인해 하드웨어 메모리 배리어가 무력화됩니다.
 * **발견된 모순:** T290_FsmMgr_250.cpp의 processManualReset() 함수 내부에 _is_alarm_latched = false; std::atomic_thread_fence(std::memory_order_release); asm volatile("memw"); 로직이 있습니다. 멀티코어 동기화를 위해 정성스럽게 펜스를 쳤지만, 정작 _is_alarm_latched 변수가 <atomic> 헤더의 std::atomic<bool>로 선언되지 않은 단순 bool이라면 컴파일러(GCC)는 이 변수의 변경을 다른 코어에서 즉시 볼 필요가 없다고 판단하여 L1 캐시나 레지스터에 가둬버릴 수 있습니다.
 * **개선방안:** _is_alarm_latched를 비롯해 인터럽트(ISR)나 다른 코어와 공유되는 모든 상태 플래그는 반드시 std::atomic<bool> 또는 volatile로 엄격히 선언하여 컴파일러 최적화를 강제 억제해야 펜스(Fence)가 정상 작동합니다.


---

### 20. [T220 Config ↔ T270 Commu] 핫스왑(Hot-Swap) 설정 변경 후 백그라운드 데몬 동기화 누락
웹 UI나 MQTT를 통해 동적 설정(T220)을 변경했을 때, 통신 엔진(T270)이 이를 즉각 인지하고 반영하는 연결 고리가 없습니다.
 * **팩트 확인:** T270_Commu_250.cpp의 웹 API(/api/config/save 등)에서 CL_T2_ConfigManager::getInstance().commitSave()를 호출하여 설정은 성공적으로 RAM과 Flash에 갱신됩니다. 그러나 T270의 runNetwork() 함수는 오직 WiFi.status() != WL_CONNECTED일 때 재연결만 시도할 뿐, **"SSID, IP, 혹은 MQTT 브로커 주소가 변경되었는지"**를 감지하여 기존 소켓을 닫고 재시작하는 로직이 전무합니다.
 * **로직 모순:** 사용자가 AP 주소나 MQTT 설정을 변경해도 시스템을 하드 리부팅(Hard Reboot)하기 전까지는 기존 연결을 계속 유지하는 심각한 기능 오작동이 발생합니다.
 * **해결 방안:** T220에 설정 변경 이벤트 콜백(또는 Dirty Flag 쿼리 API)을 추가하고, FSM(T290)이 이를 감지하여 T270에 CMD_RESTART_NETWORK 명령을 하달하여 MQTT 클라이언트와 WiFi를 Graceful하게 재기동하는 로직이 필수적입니다.
### 21. [T250 Trigger ↔ T260 Storage] 사전 트리거(Pre-Trigger) 버퍼의 파일 덤프(Dump) 연동 누락
SMEA-100과 같은 산업용 진단기기에서 가장 중요한 "이벤트 발생 이전 X초 간의 데이터(Pre-trigger)"를 실제 파일에 기록하는 크로스 로직이 끊어져 있습니다.
 * **팩트 확인:** T260_Storage_250.hpp를 보면 _preAudBuf, _preVibBuf 등 링 버퍼 포인터가 존재하고 설정(p_cfg.storage.pre_trig_sec)도 파싱됩니다. 하지만 FSM(T290)이 트리거 엔진(T250)의 이상 감지를 받아 T260::openSession()을 호출하여 새 파일을 생성할 때, **과거 데이터가 담긴 Pre-trigger 링 버퍼를 새 파일의 최상단에 쏟아내는(Dump) 로직에 대한 인터페이스가 설계되어 있지 않습니다.**
 * **로직 모순:** 트리거 시점 '이후'의 데이터만 기록되며, 베어링 파손이나 스파크가 발생하기 직전의 가장 중요한 MLOps 전조 증상 파형이 영구 유실됩니다.
 * **해결 방안:** T260의 openSession() 내부, 혹은 그 직후에 호출될 dumpPreTriggerToSession() 인터페이스를 신설하여, 세션이 열리자마자 링 버퍼의 Tail부터 Head까지의 과거 데이터를 파일 시스템에 즉시 write() 하도록 통합해야 합니다.
### 22. [T230 Sensor ↔ T248 SeqBuild] I2S와 SPI 이종 클럭 간의 시간 편차(Clock Drift) 누적
오디오(I2S DMA)와 진동(SPI Polling) 데이터의 타임스탬프를 정렬하여 단일 텐서로 묶을 때, 하드웨어 타이머의 물리적 한계로 인해 AI 추론 파이프라인이 붕괴됩니다.
 * **팩트 확인:** T248_SeqBuild_250.hpp의 MultiRateTimeAligner는 _max_allowed_skew_us를 기준으로 진동과 오디오의 타임스탬프(esp_timer_get_time())를 매칭합니다. 그러나 오디오는 하드웨어 APLL에 의해 완벽한 16kHz로 구동되는 반면, 가속도/자이로(T230)는 FreeRTOS 태스크 스케줄링의 지연(Jitter)을 동반한 SPI 폴링으로 데이터를 수집합니다.
 * **로직 모순:** 구동 후 1~2시간이 지나면 APLL 기반 오디오 시간축과 CPU 틱 기반 진동 시간축 간에 수십 밀리초 이상의 편차(Drift)가 누적됩니다. 결국 MultiRateTimeAligner의 허용 오차를 벗어나게 되어, **"최신 오디오 프레임에 매칭되는 진동 프레임이 없다"**고 판단해 시퀀스 텐서 조립(DynamicTensorBinder)이 영구적으로 멈추는 데드락에 빠집니다.
 * **해결 방안:** 두 스트림 간의 단순 타임스탬프 비교를 넘어, 오디오 샘플 카운트를 절대 기준 시간축(Master Clock)으로 삼고 진동 데이터의 타임스탬프를 소프트웨어적으로 보정(Software PLL)하거나 가장 가까운 과거 프레임을 강제로 보간(Interpolation)하는 크로스 도메인 동기화 로직이 T248에 추가되어야 합니다.
### 23. [T260 Storage ↔ T290 FSM] SD 카드 I/O 에러 발생 시 FSM 예외 처리 블랙홀
스토리지 엔진에서 발생한 물리적 에러가 시스템 전체 상태(State)로 전파되지 않아 파이프라인이 좀비 상태로 동작합니다.
 * **팩트 확인:** T260_Storage_250.cpp는 SD 카드가 가득 차거나 뽑혔을 때 _ioError = true;를 설정하고 모든 쓰기 연산을 무시(return false;)합니다. 그러나 오케스트레이터인 T290_FsmMgr_250.cpp는 _storage.hasIoError()를 주기적으로 감시하여 상태를 EM_SystemState_t::ERROR로 전환하거나 사용자에게 알람(MQTT/Web)을 보내는 로직이 없습니다.
 * **로직 모순:** 현장에 설치된 기기의 SD 카드가 고장 나도, 시스템은 계속해서 센서를 읽고 DSP 연산을 수행하며 정상 작동(MONITORING 상태)하는 것처럼 위장합니다. 작업자는 데이터를 수거하러 올 때까지 저장 실패 사실을 알 수 없습니다.
 * **해결 방안:** T290의 메인 관리 태스크 루프 내부에 if (_storage.hasIoError()) { dispatchCommand(CMD_STORAGE_ERROR); } 구조를 추가하여, FSM이 에러 상태로 진입하고 기기 전면의 상태 LED를 적색으로 점멸시키는 동시에 통신 모듈(T270)을 통해 관리자에게 긴급 텔레메트리를 발송하도록 링키지를 구성해야 합니다.

---


### 24. [FSM ↔ SensorEngine] 모드 전환 시 하드웨어 FIFO 잔여 데이터 오염 (Flush 무결성 결함)
T290에서 상태가 MONITORING에서 RECORDING 또는 MAINTENANCE로 전환될 때, 센서 엔진(T230)의 하드웨어 FIFO에 남아있는 '과거 모드'의 데이터가 신규 모드 데이터와 섞이는 현상이 발생합니다.
 * **팩트 확인:** T230의 getAccumulatedAccelCount()와 getAccumulatedGyroCount()는 FIFO를 읽어오기만 할 뿐, 모드 전환 시(FSM 명령 하달 시) 하드웨어 FIFO를 강제로 초기화(Flush)하는 인터페이스가 없습니다.
 * **결함:** 시스템 모드가 바뀌어도 FIFO에는 이전 모드의 데이터가 잔존하며, 이것이 첫 번째 특징량 추출 연산에 포함되어 T245의 특징량 정합성을 훼손합니다.
 * **해결 방안:** CL_T2_SensorEngine에 flushHardwareFifo() 함수를 추가하고, FSM 상태 전환(T290) 시 이를 강제로 호출하여 FIFO를 비우는 인터페이스를 연결해야 합니다.
### 25. [Storage ↔ FSM] 세션 닫기 전 '보류 중인(Pending) 데이터' 소실 (Graceful Shutdown 결함)
FSM이 CMD_STOP 또는 전원 차단 신호를 받았을 때, T260의 비동기 링 버퍼에 남아있는 최후의 데이터가 디스크에 쓰이지 않고 증발합니다.
 * **팩트 확인:** T260_Storage_250.cpp의 closeSession()은 단순히 파일 핸들을 닫습니다. _processRingIO 태스크가 링 버퍼를 다 비우기 전에 호출될 경우, 버퍼에 남은 데이터는 처리되지 않습니다.
 * **결함:** AI 시퀀스 빌더(T248)가 전달한 중요한 결함 징후 데이터가 스토리지에 기록되지 않은 채 시스템이 종료됩니다.
 * **해결 방안:** closeSession() 호출 시 링 버퍼의 isEmpty()를 체크하여, 빌 때까지 태스크가 대기(Wait)하거나 강제로 덤프를 수행하는 **'Graceful Flush 인터페이스'**를 구현해야 합니다.
### 26. [ConfigMgr ↔ Calibrator] 캘리브레이션 프로파일의 실시간 반영 누락
T280에서 생성된 오프셋/게인 보정 계수(calib_auto.vib.bin 등)가 저장된 후, T220 매니저가 이를 실시간으로 로드하여 센서 엔진(T230)의 보정 변수에 즉시 반영하는 링키지가 없습니다.
 * **팩트 확인:** T280은 캘리브레이션 파일을 생성만 합니다. T230 센서 엔진의 오프셋 변수(_accOffsetX 등)를 갱신하려면 CL_T2_ConfigManager의 설정을 다시 읽고 T230의 setCalibration() 등을 호출해야 하는데, 이 트리거가 연결되어 있지 않습니다.
 * **결함:** 사용자가 캘리브레이션을 진행해도 실제 센서 수집 로직에는 보정값이 반영되지 않아 진단 정확도가 개선되지 않습니다.
 * **해결 방안:** 캘리브레이션 완료 시 FSM을 통해 CMD_RELOAD_CALIBRATION 이벤트를 발생시키고, T220이 파일을 읽어 T230의 멤버 변수를 업데이트하는 **'자동 리로딩 파이프라인'**을 구축해야 합니다.
### 27. [Trigger ↔ Commu] MQTT LWT(Last Will and Testament)와 로컬 진단 결과의 동기화 결함
시스템이 비정상 종료(Guru Meditation)될 때, MQTT 브로커는 LWT 메시지를 띄우지만, 로컬 LittleFS나 SD_MMC에는 해당 시점의 마지막 시스템 상태 로그가 기록되지 않아 '원인 미상의 오프라인'이 됩니다.
 * **팩트 확인:** 시스템 Crash 시 T290의 SafetyLifecycleManager가 동작하려 해도, 메모리 오염 상황에서는 파일 I/O를 수행할 수 없습니다.
 * **결함:** 관리자는 장비가 왜 죽었는지(통신 두절인지, 진단 엔진의 치명적 예외인지) 구분할 수 없습니다.
 * **해결 방안:** esp_task_wdt 핸들러 내에서 가장 마지막으로 성공했던 진단 결과(결과값, trial count)를 미리 지정된 RTC_DATA_ATTR 메모리 영역에 순차적으로 업데이트하고, 부팅 시 T200_Main_250.h의 T2_init()에서 이를 체크하여 통신 엔진이 **"이전 비정상 종료 시 마지막 진단값"**을 브로커에 텔레메트리로 발행하도록 구현해야 합니다.


### 28. [T270 Commu ↔ T290 FSM ↔ T230 Sensor] OTA 업데이트 중 I2S/SPI 캐시 미스 크래시
원격 펌웨어 업데이트(OTA) 중 센서 하드웨어의 DMA 및 인터럽트가 차단되지 않아 시스템이 붕괴되는 현상입니다.
 * **팩트 확인:** T270_Commu_250.cpp의 /api/update 핸들러는 OTA 시작 시 CMD_OTA_START를 디스패치합니다. 그러나 FSM(T290)이 이 명령을 수신했을 때, 센서 엔진(T230)의 SPI 트랜잭션과 I2S DMA 콜백을 물리적으로 중단(Pause)시키는 연결 로직이 없습니다.
 * **크로스 모듈 모순:** OTA 라이브러리가 수백 KB의 바이너리를 Flash 메모리에 기록하는 동안 내부 데이터 버스와 플래시 캐시가 완전히 잠깁니다(Cache Disabled). 이때 Core 1에서 돌아가는 I2S DMA와 SPI 폴링 태스크가 멈추지 않고 메모리 접근을 시도하면, ESP32는 즉각 Cache Load Error 또는 ISR Watchdog 패닉을 일으키며 재부팅되어 OTA가 영구 실패합니다.
 * **해결 방안:** FSM(T290)이 CMD_OTA_START 수신 시 반드시 T230의 _isPaused = true;를 설정하고 하드웨어 타이머와 I2S 채널을 명시적으로 stop() 하도록 크로스 인터페이스를 구현해야 합니다.
### 29. [T245 Feature ↔ T260 Storage ↔ T220 Config] 학습된 환경 노이즈 프로필(Noise Profile)의 휘발성 누락
시스템이 현장의 배경 소음을 자가 학습(Self-Learning)하지만, 이 데이터가 비휘발성 스토리지로 연동되지 않아 리부팅 시마다 오작동을 유발합니다.
 * **팩트 확인:** T245_FeatExtra_250.cpp는 _noiseProfile 배열(수 KB 크기의 주파수 빈 스펙트럼)에 현장의 배경 소음을 학습합니다. 하지만 설정 매니저(T220)의 ST_DynamicConfig_t는 용량 제약상 이를 담지 못하며, 스토리지 매니저(T260)에도 이 프로필을 파일로 저장/로드하는 인터페이스가 없습니다.
 * **크로스 모듈 모순:** 설비 점검 등으로 기기가 재부팅되거나 딥슬립에서 깨어나면, RAM에 있던 _noiseProfile이 초기화(0.0f)됩니다. 시스템은 현장 소음을 결함 진동으로 오인하여 재학습이 끝날 때까지 수분 간 무더기로 **오경보(False Positive)** 트리거를 발생시킵니다.
 * **해결 방안:** 노이즈 학습이 완료되는 시점(_learnedFrames >= p_cfg.audio.noise.learn_frames)에 FSM 이벤트를 발생시키고, T260이 _noiseProfile 메모리 블록 전체를 LittleFS의 전용 바이너리 파일(예: noise_profile.bin)로 덤프 및 부팅 시 자동 로드하도록 동기화해야 합니다.
### 30. [T290 FSM ↔ T270 Commu] Zero-Copy 바이너리 텔레메트리의 패킷 조립 브릿지 증발
실시간 웹 UI 모니터링을 위해 설계된 16바이트 정렬 고속 패킷 규격이 실제 전송 파이프라인에서 버려져 있습니다.
 * **팩트 확인:** 타입 헤더(T215)에는 통신용 최적화 구조체인 ST_PktTelemetry_t가 선언되어 있고, 통신 엔진(T270)에는 broadcastBinary()가 구현되어 있습니다. 하지만 특징 추출부(T245)는 가속도(ST_FeatureSlot_Vib_t)와 오디오(ST_FeatureSlot_Aud_t)를 완전히 분리하여 FSM에 전달합니다.
 * **크로스 모듈 모순:** 오케스트레이터인 T290 내부에 쪼개진 두 슬롯의 데이터를 모아 ST_PktTelemetry_t 구조체로 병합(Bind)하고 T270의 broadcastBinary()로 쏘아주는 연결 브릿지가 없습니다. 결과적으로 실시간 파형/스펙트럼 웹 모니터링 기능이 동작하지 않거나 메모리가 깨진 채로 전송됩니다.
 * **해결 방안:** T290_FsmMgr_250.cpp의 스트리밍 브로드캐스트 로직 내부에 제로 카피 텔레메트리 패킷 규격에 맞춰 데이터를 팩킹하는 전용 어댑터(Adapter) 함수를 추가해야 합니다.
### 31. [T250 Trigger ↔ T260 Storage] 프리트리거(Pre-Trigger)의 '기준 시간축(T0)' 상실
진단 엔진이 결함을 판정하여 과거 버퍼를 파일로 쏟아내지만, MLOps 분석가가 결함 시작 시점을 알 수 없는 데이터 구조적 결함입니다.
 * **팩트 확인:** 진단 엔진(T250)은 현재 슬롯에서 결함을 검출하고, 스토리지 엔진(T260)은 링 버퍼에 쌓아둔 과거 X초 분량의 데이터(_preAudBuf, _preVibBuf)를 통째로 파일(wav, bin)에 기록합니다.
 * **크로스 모듈 모순:** 파일 생성 시 트리거가 발생한 정확한 시점의 **절대 타임스탬프(T0)**가 스토리지 헤더나 파일 메타데이터에 연동되지 않습니다. 가속도(1.6kHz)와 소음(16kHz)의 샘플링 율이 다른 상황에서 T0 마커가 없으면, 후처리 파이프라인(Python 등)에서 덤프된 파일의 어느 지점이 '정상'이고 어느 지점부터 '이상 징후'인지 정밀하게 분리(Slicing)할 수 없습니다.
 * **해결 방안:** T260의 openSession() 인터페이스 파라미터에 uint64_t p_triggerTimestamp를 추가하고, 덤프된 바이너리 파일의 최상단 커스텀 헤더 청크(Chunk)에 이 T0 타임스탬프를 1순위로 기록하도록 로직을 개선해야 합니다.


### 32. [T220 Config ↔ T240 DspEng] 동적 필터 계수(Coefficient) 업데이트 단절
사용자가 웹/MQTT로 필터 주파수를 변경했을 때, 변경된 설정이 메모리에는 반영되지만 실제 DSP 연산기에는 적용되지 않는 상태 불일치 현상입니다.
 * **팩트 확인:** T220_CfgMgr_250.cpp의 updateFromJson()이나 updateConfig()를 통해 노치(Notch) 주파수나 HPF 컷오프 설정이 변경됩니다. 하지만 T240_DspEng는 부팅(Init) 시점에 한 번만 dsps_biquad_gen_notch_f32() 등의 함수를 호출하여 필터 계수(notch_coeffs)를 계산합니다.
 * **크로스 모듈 모순:** 관리자가 원격으로 기계식 공진 주파수를 잡기 위해 노치 필터 대역을 120Hz에서 150Hz로 핫스왑(Hot-swap) 변경해도, T220은 새 값을 저장하지만 T240은 재부팅 전까지 계속 120Hz의 과거 계수로 연산합니다.
 * **해결 방안:** FSM(T290)이 설정 변경 이벤트를 구독하고, 변경이 감지되면 즉각 T240의 recalculateFilters() 인터페이스를 호출하여 DSP 계수를 런타임에 갱신하는 링키지가 필요합니다.
### 33. [T220 Config ↔ T290 FsmMgr] 지연 쓰기(Lazy Write) 데몬 틱(Tick)의 고아화(Orphaned)
설정의 잦은 변경으로 인한 플래시 마모를 막기 위해 만든 훌륭한 '지연 쓰기' 함수가 시스템 어디에서도 호출되지 않아, 변경된 설정이 영구 증발할 위험이 있습니다.
 * **팩트 확인:** T220_CfgMgr_250.hpp에 checkLazyWrite() 함수가 존재합니다. 이 함수는 _isDirty 플래그와 타임스탬프를 검사하여 플래시에 반영합니다. 하지만 전체 시스템의 오케스트레이터인 T290_FsmMgr_250.cpp의 메인 루프나 백그라운드 태스크 어디에서도 이 함수를 주기적으로 호출(Tick)해주는 코드가 없습니다.
 * **크로스 모듈 모순:** 웹 UI에서 설정을 갱신하면 RAM에는 적용되지만(Dirty 상태), 플래시 기록이 무한정 보류됩니다. 이 상태에서 기기 전원이 차단되면, 최근 수정한 모든 튜닝값이 사라지고 과거 설정으로 롤백됩니다.
 * **해결 방안:** T290의 감시(Watchdog/Maintain) 태스크 루프나, T200_Main의 loop() 함수 내부에 CL_T2_ConfigManager::getInstance().checkLazyWrite();를 최소 1초 주기로 호출하는 브릿지를 추가해야 합니다.
### 34. [T290 FsmMgr ↔ T230 Sensor] 딥슬립 진입 전 센서 하드웨어 PMU 제어 누락
시스템이 절전 모드로 들어갈 때, 센서가 MCU를 다시 깨울 수 있도록 하드웨어 칩(BMI270)의 전력 관리 유닛(PMU)을 재설정하는 브릿지가 없습니다.
 * **팩트 확인:** 설정상 p_cfg.accel.motion_en과 wake_g 등 웨이크업 파라미터가 존재하고 트리거 엔진은 trig_use_sleep 조건을 갖추고 있습니다. 그러나 FSM(T290)이 판단하여 esp_deep_sleep_start()를 호출하는 진입점 직전에, T230 센서 엔진을 호출하여 BMI270을 'Any-Motion Wake-up' 모드로 전환하는 로직이 빠져 있습니다.
 * **크로스 모듈 모순:** MCU가 딥슬립에 들어가면 I2C/SPI 통신이 모두 끊깁니다. 사전에 BMI270의 인터럽트 핀을 모션 감지 핀으로 맵핑해두지 않으면, 설비가 다시 강하게 진동해도 ESP32는 영원히 깨어나지 못하는(Deadlock) 상태가 됩니다.
 * **해결 방안:** T230에 prepareDeepSleepWakeup(wake_g, wake_dur) 인터페이스를 신설하고, T290이 슬립 명령어 실행 직전에 이 함수를 호출하여 하드웨어 인터럽트 라인을 안전하게 세팅한 후 수면에 들어가도록 동기화해야 합니다.
### 35. [T280 Calib ↔ T260 Storage] SD_MMC 물리적 파일 시스템 경합 (Bus Thrashing)
비동기 캘리브레이션 태스크와 실시간 스토리지 기록 태스크가 SD 카드라는 단일 물리 매체(Single Physical Medium)를 두고 병렬 경합하여 버스가 붕괴됩니다.
 * **팩트 확인:** 캘리브레이터(T280)는 _hCalibTask라는 별도 태스크에서 SD 카드의 수십 MB짜리 과거 .vib.bin 파일을 스캔하고 읽어 들입니다. 만약 이때 FSM이 RECORDING 상태라서 스토리지 엔진(T260)이 초당 수천 개의 샘플을 SD 카드에 write() 하고 있다면 심각한 경합이 발생합니다.
 * **크로스 모듈 모순:** SD 카드는 멀티 스레드의 동시 Read/Write에 극도로 취약합니다. 캘리브레이터가 대용량 파일을 읽느라 버스를 점유하면, T260의 쓰기 대기 시간(Latency)이 기하급수적으로 늘어나 결국 센서 링 버퍼가 오버플로우되고 실시간 데이터가 유실됩니다.
 * **해결 방안:** FSM(T290)에 강력한 뮤텍스 기반의 **State Lock**이 필요합니다. 캘리브레이션 모드가 시작되면 FSM은 반드시 MAINTENANCE 상태로 전환되어야 하며, T260의 모든 실시간 세션 기록을 강제로 종료(Close Session)한 뒤에만 T280이 디스크 I/O를 시작할 수 있도록 크로스 모듈 권한 제어를 통제해야 합니다.

### 36. [T216 RingBuf ↔ T240 DspEng] 오버라이트(Overwrite)에 의한 위상 불연속성과 스펙트럼 붕괴
락 프리 링버퍼의 메모리 안전성은 확보되었으나, 신호처리의 수학적 무결성이 완전히 파괴되는 크로스 도메인 모순입니다.
 * **팩트 확인:** T216_RingBuf_250.hpp의 enqueue()는 큐가 꽉 차면 Tail을 밀어내고 '가장 오래된 단일 샘플 1개'를 덮어씁니다. 이 링버퍼에서 데이터를 꺼내어 T240_DspEng가 1024-point FFT를 수행합니다.
 * **크로스 모듈 모순:** 진동이나 소음 같은 연속적인 사인파(Continuous Wave) 중간에서 단 1개의 샘플이 삭제되거나 순서가 어긋나면, 시간 도메인에서 수직으로 꺾이는 **'위상 불연속(Phase Discontinuity, Hard Step)'**이 발생합니다. 이 데이터를 FFT 엔진에 넣으면 원본에 없던 막대한 고주파 '스펙트럼 누설(Spectral Leakage)'이 발생하여, 베어링 결함으로 오인되는 **거짓 트리거(False Positive)**가 100% 쏟아집니다.
 * **개선 방안:** 버퍼 오버플로우가 감지되면 단일 샘플 덮어쓰기를 금지하고, 차라리 **'전체 FFT 윈도우(예: 1024 샘플) 단위의 프레임 전체를 드롭(Drop)'**해야 합니다. 동시에 T240의 IIR/FIR 필터 히스토리 버퍼를 강제 초기화하여 이전 위상과의 연결을 끊고, FSM에 OVERFLOW_DROP 이벤트를 보고하여 해당 구간의 AI 추론을 건너뛰게 해야 합니다.
### 37. [T270 Commu ↔ T290 FsmMgr] 비동기 NTP 지연과 MQTT TLS 핸드셰이크 경합 (데드락)
통신 모듈이 네트워크를 연결하는 시점과 시스템이 시간축을 획득하는 시점 사이의 치명적인 비동기 레이스 컨디션(Race Condition)입니다.
 * **팩트 확인:** T270_Commu_250.cpp의 init()을 보면, WiFi가 연결된 직후 configTzTime()(비동기 NTP 호출)을 실행하고 **대기 없이 즉각** esp_mqtt_client_start()를 호출합니다.
 * **크로스 모듈 모순:** SNTP가 실제 외부 서버에서 시간을 받아와 시스템 RTC를 동기화하는 데는 통상 1~3초가 소요됩니다. 만약 Azure IoT, AWS, 혹은 사내망 MQTTS(보안 소켓)를 사용한다면, MQTT 클라이언트가 즉각 핸드셰이크를 시도할 때 ESP32의 시간이 '1970년 1월 1일'이기 때문에 **TLS 인증서 만료(Certificate Invalid/Expired) 오류로 연결이 영구 거부**됩니다. FSM은 네트워크가 준비되었다고 착각하지만 실제 데이터는 외부로 나가지 못합니다.
 * **개선 방안:** configTzTime() 호출 직후, timeinfo.tm_year > 120 (즉, 2020년 이상)이 될 때까지 vTaskDelay로 MQTT 기동을 유보하는 블로킹루틴을 추가해야 합니다. 시간 동기화가 완전히 끝난 뒤에만 FSM에 NETWORK_READY 이벤트를 하달하도록 로직을 교정해야 합니다.
### 38. [T290 FsmMgr ↔ T240 DspEng] 코어 마이그레이션과 Task WDT 기아(Starvation) 현상
듀얼 코어의 장점을 살리려다 도리어 RTOS의 감시견(Watchdog Timer)에 의해 시스템이 셀프 킬(Self-Kill) 당하는 결함입니다.
 * **팩트 확인:** FSM(T290)이 오디오와 진동 처리를 위해 _audioProcessTask와 _vibProcessTask를 띄웁니다. 하지만 이 태스크들을 생성할 때 xTaskCreate만 사용할 뿐, xTaskCreatePinnedToCore를 통해 특정 코어(Core 1)에 명시적으로 고정하지 않았습니다.
 * **크로스 모듈 모순:** FreeRTOS 스케줄러가 부하 분산을 위해 무거운 SIMD DSP 연산 태스크를 Core 0(WiFi 및 센서 ISR 담당)로 마이그레이션(이주)시켜버릴 수 있습니다. 이 순간 Core 0의 인터럽트 응답성이 붕괴되어 FIFO 오버플로우가 발생합니다. 또한, 수백 번의 Biquad 및 FFT 루프 안에서 vTaskDelay(0)나 esp_task_wdt_reset()이 호출되지 않아 **수 밀리초 이상 CPU를 독점하게 되고, 결국 Task WDT가 발동해 시스템을 강제 재부팅**시킵니다.
 * **개선 방안:** FSM에서 수집/통신 태스크는 반드시 Core 0에, DSP/ML 태스크는 Core 1에 PinnedToCore로 락업(Lock-up)해야 합니다. 더불어 T240의 다중 채널 루프(L/R 채널, X/Y/Z 축 연산 반복) 사이에 esp_task_wdt_reset() API를 끼워 넣어 감시견을 달래주는 링키지를 구축해야 합니다.
### 39. [T260 Storage ↔ T248 SeqBuild] 플래시 가비지 컬렉션에 의한 텐서 조립 타임아웃
물리적 디스크의 불규칙한 지연(Latency Spike)이 메모리 상의 AI 텐서 조립 파이프라인을 연쇄적으로 마비시킵니다.
 * **팩트 확인:** 스토리지 모듈(T260)이 pushAudioFrame으로 넘어온 데이터를 파일에 write() 할 때, 파일 시스템은 데이터를 클러스터에 동적으로 덧붙입니다(Append).
 * **크로스 모듈 모순:** SD 카드의 특성상 내부 웨어 레벨링(Wear Leveling)이나 가비지 컬렉션이 백그라운드에서 발동하면, 단 1회의 write()가 최대 300~500ms까지 블로킹됩니다. 이 지연은 비동기 큐를 꽉 채우고, 결국 특징량 추출부(T245)를 블로킹하며, 최종적으로 T248의 MultiRateTimeAligner가 설정된 _max_allowed_skew_us를 초과했다고 판단하여 해당 프레임의 텐서 조립을 영구 폐기하는 연쇄 붕괴를 일으킵니다.
 * **개선 방안:** 파일 시스템의 동적 할당 지연을 없애기 위해, T260이 openSession()을 호출할 때 파일 포인터를 단순히 여는 것에 그치지 않고, 예상되는 세션 크기(예: 10MB)만큼 미리 디스크 공간을 선할당(f_lseek 또는 fallocate)하여 쓰기 지연을 O(1) 수준으로 평탄화(Flattening)하는 디스크 초기화 인터페이스를 구축해야 합니다.


### 40. [T240 DspEng ↔ T245 Feature] 가변 RPM 설비에 대한 '정적 주파수 대역(Static Band)' 추적 실패
대부분의 현대 산업용 설비(인버터 제어 모터 등)는 부하에 따라 RPM이 변동합니다. 베어링 결함 주파수(BPFO, BPFI 등)는 이 1X(회전 주파수)에 엄격히 비례하여 이동하지만, 현재 시스템은 주파수 대역을 고정값으로 묶어두고 있습니다.
 * **팩트 확인:** T220_CfgMgr에서 band_start[i]와 band_end[i]를 정적(Static)으로 설정하고, T245_FeatExtra는 항상 이 고정된 인덱스 구간의 에너지만 누적하여 결함을 판정합니다.
 * **진단 논리 모순:** 설비의 RPM이 1800에서 2400으로 상승하면, 결함 피크(Peak) 주파수도 우측으로 이동하여 기존에 설정된 band_start ~ band_end 범위를 완전히 벗어납니다. 시스템은 **"대역 내 에너지가 정상"이라고 오판하여 실제 베어링 파손을 놓치게 됩니다(False Negative).**
 * **개선 방안:** T240 DSP 엔진 내부에 1X(기본 회전 주파수) 주 주파수를 동적으로 추적하는 피크 탐색 로직(Order Tracking)을 추가해야 합니다. 그리고 추출된 1X 주파수에 비례하여 T245의 band_start와 band_end 빈(Bin) 인덱스가 런타임에 유동적으로 시프트(Shift)되도록 크로스 모듈 바인딩을 구현해야 합니다.
### 41. [T290 FsmMgr ↔ T250 Trigger] STA/LTA의 '콜드 스타트(Cold Start)' 초기화 모순에 의한 오경보 폭주
단기/장기 평균(STA/LTA) 로직은 배경 소음 대비 돌발 충격을 잡는 데 탁월하지만, 장비 전원이 켜지는 시점의 물리적 상태를 간과하고 있습니다.
 * **팩트 확인:** 시스템 부팅 시 T245의 히스토리 버퍼나 LTA 관련 누적 변수들은 0.0f로 초기화됩니다. 이후 FSM(T290)이 INIT에서 MONITORING으로 전환되자마자 즉시 T250의 runDiagnostic()을 통해 임계치 판정을 시작합니다.
 * **진단 논리 모순:** 장비가 이미 시끄럽게 가동 중인 상태에서 엣지 디바이스의 전원이 켜지거나 재부팅되면, 현재 프레임의 진동(STA)은 높은 반면 과거 누적치(LTA)는 0.0f부터 시작합니다. 시스템은 이를 **"정상 상태에서 갑자기 엄청난 충격파가 발생했다"고 착각하여 수 분 동안 무더기로 오경보(False Positive)를 발생**시키고 쓰레기 데이터를 스토리지에 가득 채웁니다.
 * **개선 방안:** FSM(T290)에 WARM_UP (안정화) 상태를 신설해야 합니다. 부팅 후 LTA 버퍼가 최소 N초(예: 30초) 이상 가동되어 현재 현장의 배경 소음 수준까지 완전히 차오를 때까지는 T250의 트리거 판정 로직을 강제로 PASS 처리하는 블로킹 링키지가 필수적입니다.
### 42. [T220 Config ↔ T250 Trigger] 결함 진행 생애주기(Lifecycle)를 무시한 1차원적 임계치 OR 연산
통계적 진동 지표(RMS, Kurtosis, Crest 등)는 결함의 진행 단계에 따라 서로 반비례하며 변동하지만, 판정 엔진은 이를 단순 나열식으로 취급합니다.
 * **팩트 확인:** T250_Trigger_250.cpp의 판정 로직을 보면 if (rms > thresh) return NG; if (kurt > thresh) return NG; 형태로, 여러 지표 중 하나라도 임계치를 넘으면 즉시 결함으로 판정하는 단순 OR 논리로 구성되어 있습니다.
 * **진단 논리 모순:** 베어링 결함의 생애주기상 **'초기 결함'**일 때는 뾰족한 미세 충격파로 인해 첨도(Kurtosis)는 5.0 이상으로 치솟지만 전반적인 에너지(RMS)는 정상 범위입니다. 반면 마모가 완전히 진행된 **'말기 결함'**일 때는 진동 에너지가 커져 RMS는 극도로 높지만, 충격이 뭉개져 백색잡음화 되므로 오히려 첨도(Kurtosis)는 3.0(정상) 근처로 뚝 떨어집니다. 현재의 단순 OR 구조로는 "이 장비가 이제 막 고장 나기 시작했는지, 아니면 당장 멈춰야 하는 말기 상태인지"를 분별할 수 없습니다.
 * **개선 방안:** 단순 임계치 초과 판정이 아니라, 지표 간의 상관관계를 분석하는 **다변량(Multi-variate) 매트릭스 판정 로직**으로 진화해야 합니다. (예: if (Kurt > 4.5 && RMS < 경고치) -> EARLY_FAULT, if (Kurt < 3.5 && RMS > 위험치) -> CRITICAL_FAULT) 이를 통해 MLOps 파이프라인에 정확한 결함 단계 레이블을 공급해야 합니다.
### 43. [T250 Trigger ↔ T260 Storage] MLOps 데이터 파이프라인의 '레이블 모호성(Label Ambiguity)' 발생
클라우드 플랫폼(Azure Databricks 등)으로 전송된 데이터가 지도학습(Supervised Learning)이나 강화학습(RL)에 사용되려면 "이 데이터가 왜 수집되었는가?"에 대한 레이블이 반드시 동반되어야 합니다.
 * **팩트 확인:** T250 진단 엔진이 결함을 발견하면, T260 스토리지 엔진은 프리트리거 링버퍼 데이터를 파일(wav, bin)로 저장합니다. 하지만 이때 스토리지에 기록되는 파일 헤더나 메타데이터에는 **'트리거의 원인(어느 축의 어떤 지표가 몇의 값으로 임계치를 넘었는지)'**이 포함되지 않습니다.
 * **진단 논리 모순:** 클라우드의 데이터 엔지니어는 수백 개의 NG 폴더에 쌓인 바이너리 파일을 열어볼 때, 이것이 "가속도 X축의 밴드 에너지가 튀어서" 저장된 것인지, "오디오 노이즈의 RMS가 폭주해서" 저장된 것인지 역추적할 방법이 없습니다. 엣지 단에서 이미 명확하게 판정된 **그라운드 트루스(Ground Truth) 정보가 스토리지 덤프 과정에서 영구 증발**하는 치명적 설계 결함입니다.
 * **개선 방안:** T250의 runDiagnostic 반환값을 단순 EM_DetectionResult_t 열거형이 아니라, ST_TriggerReason 구조체(발생 축, 지표 명, 실제 값, 초과 비율 등 포함)로 확장해야 합니다. 그리고 이 구조체가 T290을 거쳐 T260으로 전달되어, 기록되는 .bin 파일의 최상단 커스텀 헤더에 반드시 직렬화(Serialization)되어 박히도록 데이터 파이프라인을 교정해야 합니다.

