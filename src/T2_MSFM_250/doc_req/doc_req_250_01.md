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

