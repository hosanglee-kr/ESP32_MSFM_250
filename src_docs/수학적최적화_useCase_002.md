


## 제조회사에서 수학적 최적화(MILP 등)를 적용할 수 있는 핵심 문제 유형
### 1. 생산 계획 및 스케줄링 (Production Planning & Scheduling)
 * **생산 용량 및 혼류 생산 계획 (Aggregate Production Planning)**
   * 자원(설비, 가동 시간, 인력)의 한계 내에서 주문량을 충족하기 위한 제품별 최적 생산량 결정
   * 라인 전환 비용(Setup Cost)과 가변 생산 비용의 트레이드오프 최적화
 * **잡샵/플로우샵 스케줄링 (Job-Shop / Flow-Shop Scheduling)**
   * 다단계 공정에서 작업(Job)의 투입 순서와 설비 할당을 결정하여 총 납기 지연(Tardiness) 최소화 또는 최대 완료 시간(Makespan) 단축
 * **실시간 재스케줄링 (Rescheduling)**
   * 설비 고장, 급작스러운 주문 취소/추가 등 돌발 상황 발생 시 기존 스케줄의 변경을 최소화하면서 새로운 최적 대안 수립
### 2. SCM 및 물류 거점 최적화 (Supply Chain & Logistics)
 * **네트워크 디자인 및 입지 선정 (Facility Location Problem)**
   * 공장, 물류센터(DC), 대리점의 최적 위치와 수 수를 결정하여 전사 물류 고정비 및 운송비 최소화
 * **수송 및 차량 경로 최적화 (VRP, Vehicle Routing Problem)**
   * 적재 용량 제한이 있는 배송 차량들의 이동 경로를 최적화하여 유류비, 운송 시간, 차량 대수 최소화
 * **자체 생산 vs 외주 조달 결정 (Make-or-Buy Decision)**
   * 내부 생산 케파 부족 시, 어떤 제품을 얼마만큼 외주로 전환해야 비용이 최소화되는지 계산 (앞선 예제 유형)
### 3. 재고 최적화 및 조달 (Inventory & Procurement Optimization)
 * **안전재고 및 보충 계획 (Inventory Lot Sizing)**
   * 제품 수요 변동성에 대응하면서도 재고 유지 비용(Holding Cost)과 주문 비용(Ordering Cost)의 합을 최소화하는 최적 발주 타이밍 및 발주량 결정
 * **공급사 선정 및 할당 (Supplier Selection)**
   * 복수의 원자재 공급사들이 제시하는 단가, 품질, 납기 준수율, 최소 주문 수량(MOQ) 조건을 고려한 최적 구매 물량 배분
### 4. 자원 및 에너지 효율화 (Resource & Energy Efficiency)
 * **노무 및 인력 배치 계획 (Shift Scheduling / Rostering)**
   * 교대 근무 조 편성, 숙련도별 인력 배치, 근로기준법 제약(최대 근무 시간 등)을 준수하는 최소 인건비 배치안 수립
 * **에너지 비용 최소화 스케줄링 (Energy-Aware Scheduling)**
   * 전기 요금이 비싼 피크 시간대를 피해 설비 가동 타이밍을 배치함으로써 제조 원가 절감
### 5. 제조 공정 효율화 (Process Optimization)
 * **조합 최적화 기반 절단 계획 (Cutting Stock Problem)**
   * 철강, 제지, 섬유 업종에서 원자재(Roll, Sheet 등)를 고객이 주문한 규격으로 절단할 때, 버려지는 스크랩(Scrap) 면적을 최소화하는 절단 패턴 조합 도출
 * **혼합/배합 최적화 (Blending Problem)**
   * 화학, 식품, 정유 업종에서 최종 제품의 품질 규격(성분 함량 등)을 만족하면서 원료 구입 비용을 최소화하는 원자재 배합 비율 결정




---



## 제조회사 수학적 최적화 추가 Use Case
### 1. 포트폴리오 및 수익 관리 (Product Portfolio & Revenue Management)
 * **생산 라인 폐기 및 신규 설비 투자 심사 (Capital Budgeting)**
   * 한정된 예산 내에서 제품 기여도, 미래 수요 예측치, 설비 도입 비용, 감가상각을 고려해 투자 수익률(ROI)을 최대화하는 설비 투자 조합 결정
 * **주문 승인 및 납기 확약 (Available-to-Promise, ATP)**
   * 긴급 주문이나 대량 주문이 들어왔을 때, 기존 고정 고객들의 납기를 저해하지 않으면서 추가 이익을 극대화할 수 있는지 실시간으로 생산 가능 여부를 판단하고 수락/거절을 결정
### 2. 친환경 및 순환 경제 (Green & Circular Economy Manufacturing)
 * **역물류 및 재제조 최적화 (Reverse Logistics & Remanufacturing)**
   * 수거된 반품이나 폐제품을 회수하여 분해, 세척, 재조립하는 공정에서 신품 원자재 조달과의 균형을 맞추어 총 재제조 비용 최소화
 * **탄소 배출권 거래제 대응 생산 계획 (Carbon-Constrained Production Planning)**
   * 정부 부여 탄소 배출 쿼터 제약 하에서, 제품별 탄소 배출 집약도를 고려해 탄소 배출권 구매 비용과 생산 이익의 합을 최적화하는 생산량 조절
### 3. 품질 관리 및 정비 (Quality Control & Maintenance)
 * **예방 보전 및 정비 스케줄링 (Preventive Maintenance Scheduling)**
   * 설비 고장 확률 모델을 기반으로, 생산 스케줄 타격을 최소화하는 최적 정비 주기 및 다운타임 타이밍 결정
 * **불량 발생 시 재작업 동선 최적화 (Rework Routing)**
   * 공정 중 불량 발생 시, 해당 반제품을 어떤 재작업(Rework) 라인으로 우회시켜야 기존 정상 제품들의 물류 흐름(Bottleneck)을 방해하지 않는지 최적 경로 지정
### 4. 물류창고 내부 운영 효율화 (Warehouse Inner-Operations)
 * **제품 슬로팅 최적화 (Product Slotting Optimization)**
   * 출하 빈도가 높은 제품을 랙(Rack)의 입출구 근처에 배치하고, 같이 자주 주문되는 연관 제품들을 인접 배치하여 작업자의 피킹(Picking) 동선 단축
 * **적재 최적화 (Pallet & Container Loading / Bin Packing)**
   * 제품의 크기, 무게, 적재 제한(무거운 것을 아래로), 방향성을 고려하여 팔레트나 컨테이너 내부 공간 낭비를 최소화하는 3차원 적재 패턴 도출

---

## 제조회사 수학적 최적화 추가 Use Case (2)
### 1. 연구개발 및 신제품 도입 (R&D & New Product Introduction)
 * **신제품 시험 생산 스케줄링 (Pilot Plant Scheduling)**
   * 정규 양산 라인 또는 파일럿 라인에서 신제품 테스트 제품을 생산할 때, 기존 양산 스케줄의 손실을 최소화하면서 실험 계획법(DOE)에 따른 필수 테스트 조건을 충족하는 최적 투입 시점 도출
 * **부품 공용화 및 플랫폼 설계 (Component Commonality Optimization)**
   * 신제품 설계 시 제품 라인업 전반의 제조 복잡성을 줄이기 위해, 부품 재고 비용과 설계 변경 비용의 트레이드오프를 계산하여 공용화할 부품의 최적 조합 결정
### 2. 고성능 장비 및 치공구 관리 (Tooling & Asset Management)
 * **고가 치공구 및 금형 공유 최적화 (Mold & Die Allocation)**
   * 프레스, 사출 공정 등에서 고가의 금형이나 치공구 수량이 한정되어 있을 때, 여러 생산 라인 간의 금형 이동 동선과 교체 시간(Changeover)을 고려한 라인별 금형 할당 스케줄링
 * **테스트 장비 로드 밸런싱 (Test Equipment Load Balancing)**
   * 반도체, 전자부품 양산 후 최종 검사(Inspection/Testing) 단계에서 각 테스트 장비의 사양과 제품별 검사 시간을 매칭하여 특정 장비에 병목이 생기지 않도록 검사 물량 균등 배분
### 3. 리스크 관리 및 복질성 대응 (Risk & Robust Optimization)
 * **공급망 중단 리스크 방어 계획 (Disruption Risk Mitigation)**
   * 특정 국가의 관세 인상, 지정학적 리스크, 협력사 파업 등으로 인해 특정 공급 경로가 차단될 시나리오를 가정하고, 복동선(Dual Sourcing) 조달 비율을 사전에 최적화하여 잠재적 손실 최소화
 * **환율 및 원자재가 변동 대응 헷징 생산 (Hedging via Global Production Strategy)**
   * 글로벌 다국적 제조 기업이 국가별 환율 변동, 관세 혜택, 에너지 비용 추이를 모니터링하며 주 단위/월 단위로 글로벌 공장별 생산 물량 가중치를 동적 재배분
### 4. 작업자 안전 및 공장 환경 최적화 (Ergonomics & Safety)
 * **작업자 피로도 고려 교대 조 편성 (Ergonomic Job Rotation)**
   * 공정별 노동 강도(근골격계 부담 등)를 데이터화하여, 작업자가 연속해서 고부하 공정에 배치되지 않도록 일일 작업 순환(Rotation) 경로를 최적화함으로써 산업재해율 감소
 * **유해물질 배출 및 환기 비용 최적화 (Hazardous Emission Control)**
   * 화학 물질 유출 리스크나 유해 가스가 발생하는 공정에서 공장 내 배기 시스템 가동 전력비와 생산 속도 간의 상관관계를 분석하여 환경 기준을 준수하는 최소 비용 가동 스케줄 수립



---


## 제조회사 수학적 최적화 추가 Use Case (3)
### 1. 지능형 공장 물류 및 제어 (Smart Factory AGV/AMR Routing)
 * **AGV/AMR 교차로 충돌 회피 및 최적 경로 지정 (Multi-AGV Routing & Conflict Resolution)**
   * 공장 내 수십 대의 무인 운반차(AGV/AMR)가 자재를 나를 때, 서로 충돌하거나 교차로에서 교착 상태(Deadlock)에 빠지지 않도록 그리드 맵 기반 실시간 최적 이동 경로 스케줄링
 * **자동 창고(AS/RS) 크레인 동선 최적화 (AS/RS Crane Scheduling)**
   * 수직/수평 이동이 가능한 자동 창고 크레인이 입고 유닛과 출고 유닛을 동시에 처리할 때, 크레인의 총 이동 거리를 최소화하는 입출하 작업 순서 배열
### 2. 설비 램프업 및 수명 관리 (Asset Life-Cycle & Ramp-Up Optimization)
 * **신규 라인 수율 램프업 계획 (Yield Ramp-Up Optimization)**
   * 신제품 출시 초기 양산 라인의 수율(Yield)이 점진적으로 상승하는 곡선을 모델링하여, 초기 불량률 발생 비용과 시장 선점 기회비용 간의 균형을 맞추는 투입 원자재량 최적화
 * **노후 설비 잔존 수명 극대화 스케줄링 (Degradation-Aware Scheduling)**
   * 고장 직전의 노후 설비에 대해 부하가 큰 고속·고온 공정 배치를 줄이고 저부하 작업 위주로 스케줄을 재할당하여, 차기 정기 정비 시즌까지 설비 다운타임 없이 가동을 연장하는 기법
### 3. 글로벌 관세 및 세제 최적화 (Global Tax & Tariff Optimization)
 * **이전가격 및 글로벌 세무 최적화 (Transfer Pricing Optimization)**
   * 다국적 제조 기업이 여러 국가의 법인 간 반제품을 거래할 때, 국가별 법인세율 차이와 국가 간 관세(Tariff) 협정 조건을 고려하여 전사 세후 이익을 극대화하는 글로벌 공급망 물동량 및 가격 책정
 * **자유무역협정(FTA) 원산지 충족 생산 계획 (FTA Origin Verification Planning)**
   * 특정 국가로 수출 시 관세 혜택을 받기 위한 부품 국산화 비율(RVC) 조건을 충족하도록 역내/역외 원자재 조달 비율을 동적으로 제어하는 모델링
### 4. 고난도 공정 스케줄링 (Advanced Process Scheduling)
 * **디커플링 버퍼 크기 최적화 (Buffer Size Allocation)**
   * 연삭, 열처리, 조립 등 공정 간 생산 속도 차이로 인해 병목이 발생하지 않도록 공정 사이에 위치한 재공(WIP) 버퍼 창고의 최적 용량을 수학적으로 도출
 * **열처리 요로(Furnace) 배치 최적화 (Furnace Batching Problem)**
   * 제강이나 반도체 공정의 열처리로(Furnace)처럼 한 번에 여러 제품을 넣고 구워야 하는 공정에서, 온도 조건과 처리 시간이 유사한 제품들을 하나의 배치(Batch)로 묶어 요로 가동 횟수와 에너지 낭비를 최소화




---



## 제조회사 수학적 최적화 추가 Use Case (4)
### 1. 설계 및 위상 최적화 (Engineering & Topology Optimization)
 * **제품 경량화 위상 최적화 (Structural Topology Optimization)**
   * 자동차 프레임, 항공기 부품 설계 시 허용 응력(Stress)과 강성을 유지하면서 제품의 무게를 최소화하도록 재료의 내부 형상과 밀도 분포를 수학적으로 계산
 * **허용 오차 누적 최적화 (Tolerance Allocation Problem)**
   * 정밀 기계 조립 시 각 부품의 가공 공차(Tolerance)가 누적되어 불량이 발생하는 것을 막기 위해, 가공 비용(공차가 까다로울수록 상승)을 최소화하면서 최종 조립 품질을 만족하는 부품별 최적 허용 오차 배분
### 2. 가치 사슬 기반 가격 및 판촉 최적화 (Value Chain Revenue Management)
 * **원자재가 연동형 동적 가격 책정 (Dynamic Pricing tied to Raw Materials)**
   * 비철금속, 석유화학 등 국제 원자재 가격 변동성이 큰 업종에서 원가 변동 시나리오와 고객사별 수요 탄력성을 실시간 연동하여 영업 이익을 최대화하는 제품별 최적 판매 가격 산출
 * **폐기 임박 자재 밀어내기 생산 및 프로모션 (Perishable Material Clearance Scheduling)**
   * 유통기한이나 유효기간이 존재하는 화학 제품, 식품 원료의 폐기 손실을 방지하기 위해, 자재 만료 전 전사 역량을 집중하여 생산할 수 있는 대체 제품군 조합 및 판촉 물량 확정
### 3. 고도화된 정비 및 서비스 네트워크 (Advanced Service Logistics)
 * **예비 부품 공급망 네트워크 (Spare Parts Logistics Optimization)**
   * 고가 장비(반도체 설비, 의료 기기 등)의 다운타임을 최소화하기 위해, 전 세계 거점 물류창고별로 고장 빈도를 고려한 최적 예비 부품(Spare Parts) 재고 수준과 긴급 수송 경로 수립
 * **정비 엔지니어 출장 동선 최적화 (Field Service Workforce Scheduling)**
   * 다수의 고객사에서 동시다발적으로 설비 고장 접수가 되었을 때, 엔지니어의 숙련도, 소지 공구, 이동 거리를 고려하여 당일 방문 스케줄 및 서비스 경로 최적화
### 4. 공장 레이아웃 및 설비 배치 (Plant Layout & Facility Assignment)
 * **공장 내 부서 및 설비 배치 최적화 (Facility Layout Problem)**
   * 신설 공장을 짓거나 라인을 개조할 때, 공정 간 물동량 이동 빈도를 고려하여 자재 운반 비용 및 동선 낭비가 최소화되도록 각 설비와 작업 셀(Cell)의 2차원 좌표 배치 최적화
 * **모듈형 조립 라인 밸런싱 (Modular Assembly Line Balancing)**
   * 다품종 소량 생산을 위한 셀 제조 시스템(Cell Manufacturing)에서 작업자 간 부하 균형(Load Balancing)을 맞추고 공정 간 대기 시간(Idle Time)을 최소화하도록 작업 요소를 셀별로 최적 할당



---


## 제조회사 수학적 최적화 추가 Use Case (5)
### 1. 전력망 연동 및 ESS 운영 최적화 (Smart Grid & ESS Operation)
 * **ESS(에너지저장장치) 충방전 스케줄링 (ESS Charging/Discharging Optimization)**
   * 공장 내 설치된 대용량 배터리(ESS)를 활용하여, 실시간 전기요금이 저렴한 경부하 시간대에 충전하고 피크 시간대에 방전함으로써 공장 전력 비용 최소화
 * **신재생 에너지 발전 연동 생산 스케줄링 (Renewable Energy Integrated Scheduling)**
   * 공장 지붕의 태양광 발전이나 인근 풍력 발전의 출력 예측치와 연동하여, 친환경 에너지 발전량이 많은 시간대에 고전력 소비 공정(예: 용해, 열처리)을 우선 배치하는 동적 스케줄링
### 2. 가상 공장 및 디지털 트윈 최적화 (Digital Twin & Virtual Plant Optimization)
 * **디지털 트윈 기반 실시간 병목 예측 제어 (Real-time Bottleneck Resolution in Digital Twin)**
   * 디지털 트윈 환경에서 센서 데이터를 통해 특정 공정의 가동 속도 저하나 자재 지연이 감지될 때, 전체 공장의 물류가 마비되지 않도록 완충 재공(Buffer)의 흐름과 컨베이어 속도를 실시간 최적 재배치
 * **가상 시뮬레이션 최적 라인업 설계 (Simulation-based Assembly Line Layout)**
   * 신규 제품 라인 투입 전, 디지털 트윈 상에서 다양한 설비 조합과 배치 시나리오를 MILP로 선별한 뒤 최적의 대안만을 물리 공장에 적용하여 시행착오 비용 절감
### 3. 고정밀 조립 및 매칭 최적 Selective Assembly (정밀 기계 및 전자기기)
 * **선택적 조립 부품 매칭 최적화 (Selective Assembly Matching Optimization)**
   * 반도체 헤드, 고정밀 모터, 실린더 조립 시 부품 간 미세한 가공 오차(단위: \mu m)가 존재함. 임의 조립 시 불량이 발생하므로, 측정된 부품들의 치수 데이터를 기반으로 결합 시 공차가 제로에 수렴하도록 1:1 최적 매칭 쌍을 찾아내는 조합 최적화
 * **폐기 부품 최소화 셋 매칭 (Set Yield Maximization)**
   * A 부품 100개와 B 부품 100개의 치수가 각각 다를 때, 규격 만족 조립 쌍을 최대화하여 매칭 실패로 버려지는 잉여 부품을 최소화
### 4. 고도화된 패키징 및 다차원 적재 (Advanced Multi-drop Logistics)
 * **다중 목적지 배송 차량 적재 최적화 (Multi-drop Bin Packing & Loading)**
   * 하나의 탑차에 여러 고객사의 물품을 싣고 순차 배송할 때, 먼저 내릴 물건이 탑차 문 근처(바깥쪽)에 위치하도록 배송 경로(VRP)와 하역 순서(LIFO 제약)를 동시에 고려한 3D 적재 스케줄링
 * **진동 및 하중 균형 적재 최적화 (Center of Gravity Constraint Loading)**
   * 정밀 장비나 중량물 수송 시 차량의 무게 중심(Center of Gravity)이 한쪽으로 쏠려 사고가 나지 않도록, 하중 균형 제약식을 포함한 컨테이너 내부 배치 최적화

---
