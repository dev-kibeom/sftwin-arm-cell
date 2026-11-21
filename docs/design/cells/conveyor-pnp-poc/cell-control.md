# Conveyor Pick & Place Cell — Cell Control Design (초안)

- **대상 Cell:** `conveyor-pnp-poc`
- **Owner:** Cell 통합 제어 책임자 (선정 필요)
- **상태:** 설계 초안

> PLC와 로봇 BT 사이의 제어 권한·명령·상태 교환을 정의한다. 공정 Step/Gate/Commit 의미는 Cell Process Design을 참조하고, 재사용 가능한 공유 필드 의미 및 내부 실행 시퀀스는 정식 System ICD/FDS가 소유한다.

## 제어 책임과 설비 경계

| 역할·제어 주체 | 판단·명령 권한 | 외부에 제공하는 상태 | 비책임 경계 |
|---|---|---|---|
| **SoftPLC** | 컨베이어·스토퍼 명령, 공급·Pick 허가, Cell 단계 조정, Cell Commit | 준비/이송/위치 확정, 로봇 요청, Cell 완료·보류 | 로봇의 Motion 경로·BT 내부 Retry를 직접 결정하지 않음 |
| **로봇 Orchestration / Task Execution BT** | 수락한 Robot Task 실행, Pick·Place 전략과 내부 복구 | 요청 수락, 작업 중, terminal 결과, 자재 처리 결과 | PLC의 컨베이어 모터·센서 명령을 직접 결정하지 않음 |
| **PLC Adapter** | Modbus TCP와 Robot Task 경계의 변환, 요청 ID 상관, 재연결 동기화 | 링크 상태, 마지막 수락·완료 사실 | Cell Commit을 독자적으로 재판정하지 않음 |
| **Isaac 설비 모델** | PLC 명령에 따른 가상 기구 운동, Commit 후 트레이 자재 배출 | 벨트 실제 정지, 스토퍼 위치, 자재 감지, 트레이 슬롯 점유 | PLC 상태·BT 결과를 직접 확정하지 않음 |
| **Robot Backend** | 명목 티칭/계획 경로 실현과 Logical Attach | 로봇 관절·그리퍼 관측 | Cell 공정 완료 확정 아님 |
| **안전 제어계** | 실제 안전 정지·허가의 독립 권한 | 정지·해제/허가 상태 | 일반 PLC/BT가 우회할 수 없음 |

**기존 시스템과의 연결 시 주의:** 현재 공개 ARM Cell의 Gateway는 Delivery/Handoff와 VDA 문맥을, Orchestration은 Mission·PnP 결과와 수량을 별도로 소유한다. 이번 PoC는 이 의미를 임의 변경하거나 가상의 VDA/AMR 상태를 꾸며내지 않는다. PLC의 위치 확정·작업 요청을 기존 계약과 접속할 방법을 구현 전에 검토한다. 기존 범용 기능이 충분하지 않다면 공통 인터페이스 확장을 별도 시스템 설계로 다룬다. Cell Commit과 기존 로봇 PnP 완료 회계는 서로 다른 개념이며 중복 카운팅하지 않는다.

상위 설비나 HMI에서 PackML을 사용할 경우 PLC가 Cell 전체의 운전 상태를 표현한다. 예를 들어 연속 생산은 `Execute` 안에서 P1~P5를 반복하며, 개별 Cell Commit은 운전 상태 전이를 일으키지 않는다. [기존 `/packml/state`](../../interfaces/icd-external-safety.md)는 외부 CNC/공정 상태를 나타낸다. 로봇 실행·인수 판정과 Safety 허가는 각각의 기존 계약을 따른다.

## 단계별 명령·상태와 시퀀스

| 공정 단계 | 명령 주체 → 수행 주체 | 명령 효과 | 상태 출처 → 소비 주체 | 적용 계약 |
|---|---|---|---|---|
| P1 | PLC → Feeder | 빈 배출 슬롯 확인 후 `feed_part_command` | `infeed_part_detected` → PLC | Cell Process P1 |
| P2 | PLC → Conveyor/Stopper | `conveyor_run_command`로 운전·정지, `stopper_extend_command`로 고정 | `pick_station_part_detected`, `conveyor_stopped_feedback`, `stopper_extended_feedback` → PLC | Cell Process P2의 PLC 판정 `pick_position_confirmed` |
| P3 | PLC → Adapter → Robot Task | `pick_position_confirmed` 및 Mission 수락이 가능하면 해당 `command_id`의 PnP 요청 | 수락·실행·terminal 결과 및 `material_placed` → PLC | Cell Process P3 및 [PnP 결과 계약](../../interfaces/icd-orchestration-task-execution.md) |
| P4 | PLC | 해당 Cycle의 인수 기록을 한 번 확정 | 같은 요청의 `material_placed=true` 및 유효한 `tray_slot_occupied_feedback=TRUE` → PLC | Cell Process P4 |
| P5 | PLC → 배출 설비 | Commit 후 로봇의 배출 영역 이탈과 설비 동작 허가를 확인해 `discharge_tray_command` 발행 | 배출 후 슬롯 비움과 Mission 수락 가능 상태 → PLC | Cell Process P5 |

```mermaid
sequenceDiagram
    participant P as PLC
    participant E as 가상 설비
    participant A as PLC Adapter
    participant B as Robot BT
    E-->>P: 배출 슬롯과 투입·이송 영역 비어 있음
    P->>E: feed_part_command
    E-->>P: infeed_part_detected 및 Pick 위치 비어 있음
    P->>E: 컨베이어 운전
    E-->>P: pick_station_part_detected 신규 감지
    P->>E: 이송 정지 및 위치 고정
    E-->>P: conveyor_stopped_feedback 및 stopper_extended_feedback
    Note over P: pick_position_confirmed
    P->>A: command_id와 Robot Task 요청
    A->>B: 로봇 작업 요청
    B-->>A: 요청 수락 및 실행 상태
    par 슬롯 점유
        E-->>P: tray_slot_occupied_feedback
    and 로봇 결과
        B-->>A: terminal 결과와 material_placed
        A-->>P: 동일 command_id 결과 및 배치 사실
    end
    Note over P: 같은 Cycle의 배치 사실과 유효한 점유 확인
    P->>P: P4 조건에 따라 Cell Commit 1회
    Note over P: 배출 영역 이탈 및 안전 제어계의 배출 동작 허가 확인
    P->>E: discharge_tray_command
    E-->>P: 슬롯 비어 있음
    Note over P: Mission 수락 가능 상태 등 P5 완료 조건 확인 후 다음 공급
```

## 공정 인터록과 안전 인터록

| 구분 | 조건 출처 → 적용 주체 | 영향받는 명령·동작 | 해제/복귀 |
|---|---|---|---|
| 공정 · P1 | 활성 자재·영역 및 트레이 슬롯 점유 → PLC | 슬롯 점유·상태 불명 시 새 공급 보류 | P1 시작 조건 재평가 |
| 공정 · P2 | Robot Pick 구역 사용 중 → PLC | 컨베이어 이동·스토퍼 해제 보류 | Robot 영역 작업 종료 후 재평가 |
| 공정 · P3 | `pick_position_confirmed`, Mission 수락 가능 상태 → PLC | 조건 미확정 시 Robot Task 요청 보류 | P3 시작 조건 재평가 |
| 공정 · P4 | 요청 ID별 배치 사실과 유효한 슬롯 점유 → PLC | 인수 근거 미확정 시 Commit 보류 | 두 근거가 갖춰지면 로봇의 실패 결과를 보존하고 인수 기록 |
| 공정 · P5 | 로봇의 배출 영역 이탈 및 배출 설비 동작 허가 → PLC | 영역 점유·상태 불명 또는 동작 미허가 시 배출 명령 보류 | 영역 이탈·동작 허가 확인 후 배출. 다음 Cycle은 P5 완료 조건 적용 |
| 안전 | 실제 안전 제어계 상태 → 권한 있는 제어기 | 허용되지 않는 기구 동작 정지/금지 | 안전 권한자의 허가 후 공정 상태 재확인 |

PLC 상태 대기와 오류는 공정상 영향받는 단계에 국한한다. Safety 정지 해제만으로 업무 Cycle이 자동 완료·재개된다고 추정하지 않는다.

## 통신 단절·중복 명령·상태 재동기화

| 상황 | 필요한 정보 | 처리/다음 명령 조건 |
|---|---|---|
| 요청이 전달됐으나 ACK 없음 | `command_id`, 마지막 요청/로봇 실행 상태 | 먼저 실행 여부 조회. 미확정 상태에서 새 ID로 중복 요청 금지 |
| 같은 ID 재수신 | ID의 범위, 기존 실행 또는 terminal 기록 | Adapter/Robot는 같은 작업을 재실행하지 않고 기존 결과를 반환하도록 계약 |
| 로봇 완료 후 응답 소실 | 같은 ID의 terminal 결과·배치 사실, 실제 슬롯 점유와 Commit 여부 | Cell Process P4에 따라 미확정 Cycle만 한 번 확정 |
| PLC/Adapter 재시작 | 마지막 ID와 결과, Commit 기록, 실제 자재·로봇·슬롯 상태 | 미확정 작업 재실행 금지. 이미 Commit했으면 P5에서 실제 배출 여부와 현재 동작 조건 확인. 상관을 복원할 수 없으면 보류 |
| 실제 설비 상태 불일치 | 관측의 유효성 및 해당 Cycle과의 연관성 | Cell Process의 재확인 경로 사용, 독립적 재검출이 불가능하면 작업자 확인 |

실행 코드의 영속화 수단·레지스터 표현은 구현 계약에서 결정하되, 재시작·통신 단절 시험이 주장하는 범위에 맞게 해당 단계에 필요한 결과와 관측 증거를 보존하거나 재조회할 수 있어야 한다.

## PLC·Equipment Adapter 구성 참조

| 대상 | 논리 계약과 구현 구성 연결 | 설정 원본 |
|---|---|---|
| SoftPLC ↔ 설비 I/O | P1·P2 공급/이송/스토퍼 명령과 설비 피드백, P4·P5 슬롯 점유 및 배출 명령 | [Conveyor I/O 매핑](../../../../ros2_ws/src/plc_adapter/README.md#conveyor-pnp-cell-공정-프로그램) |
| SoftPLC ↔ Robot Task | 요청 ID, 수락·실행·terminal 결과, `material_placed`, Mission 수락 가능 표본 | PLC Adapter Mapping Configuration |
| Isaac 설비 | Asset/Joint, 센서 Prim과 I/O Binding | Cell / Equipment Configuration |
| Robot Task | 정적 Pick/Place 목표, 상면 흡착 전략, Task Profile | Robot/Tool/Cell Profiles + Calibration |

Modbus 레지스터 번호, PLC 타이머, 실제 티칭 좌표와 속도는 이 설계서가 아닌 해당 구성의 정식 원본에서 관리한다.

## Robot Task 연결과 후속 Cell 통합

1. PLC 요청은 [PLC 단일 작업 ICD](../../interfaces/icd-plc-task.md)를 통해 [Configured-source Mission](../../interfaces/icd-configured-supply.md)에 연결한다. Task는 Mission이 예약한 도착 자재에 고정 Pick/Place 티칭을 적용한다. 기존 Vision PnP에는 실제 Acquisition 후보가 필요하다.
2. 공급자는 실제 Pick 도착과 PLC 위치 확정을 대조하고 해당 자재 ID를 PLC에 응답한다. PLC는 연결을 확인한 뒤 P3 요청을 발행한다. 공급 예약과 Robot Task는 같은 설비 슬롯과 그때 도착한 개체를 함께 참조한다.
3. Task 실패는 기존 Mission을 종료한다. 배치 사실은 같은 Mission의 terminal 결과로 전달하며 PLC가 P4에서 별도 인수를 판단한다. PoC의 Modbus TCP 장치 주소와 I/O 매핑은 Adapter 구성에서 관리한다.
4. 고장 주입은 실제 I/O 관측 경계에서 수행한다. `Fault Injector`가 PLC 내부 상태나 Cell Commit을 직접 수정하는 것은 금지한다.


## 공급 설정과 Isaac 설비 연결

Conveyor의 UI Supply Port는 자동운전을 멈춘 상태에서 종류와 공급 가능 수량을 설정하고 자동운전을 시작·일시 정지한다. 설비는 PLC 공급 명령과 자재 ID로 생성 동작을 한 번 수행하고 실제 생성 확인 후 수량을 차감한다. 투입 도착은 별도 센서가 확인한다. 생성 결과가 불명확하면 해당 ID를 보류하고 자재와 생성 이력을 대조한다. Tape·CNC의 공급 요청은 각 Cell의 기존 공급자가 처리한다.

후속 Isaac Cell의 `material_supply_factory`는 [ConveyorFeeder](../../../../infra/isaac_sim/scripts/conveyor_pnp_cell/supply.py)를 기존 UI 포트에 등록하고 종료 시 해제한다. 설비 측 [ConveyorIO](../../../../infra/isaac_sim/scripts/conveyor_pnp_cell/equipment_io.py)는 같은 Modbus 서버의 설비 블록을 교환한다. 설비가 유효한 공급 명령을 읽으면 `feed(cycle, spawn_one)`에 기존 자재 생성 함수를 연결한다. 복원 시 마지막 생성 ID와 미확정 생성 건을 실제 자재에 대조한다.

Pick 센서가 관측한 자재와 PLC가 위치를 확정한 자재가 일치하면 공급자가 그 개체를 고정 Pick 슬롯에서 공개한다. 같은 개체를 다시 관측해도 품종과 개체 ID를 유지한다. 목적지 Release를 확인한 뒤에는 해당 개체를 퇴역시키며 다음 도착 자재가 같은 슬롯을 사용한다. [Configured Supply ICD](../../interfaces/icd-configured-supply.md)

### 슬롯과 자재의 관계

아래 ER 도식은 공급 설정, 설비 관측, 예약 사이의 논리 관계를 나타낸다. 점유는 특정 개체가 Pick 위치에서 예약 가능한 기간이며, 로봇이 이송 중일 때는 예약이 계속 그 개체를 가리킨다.

```mermaid
erDiagram
    CELL ||--o{ EQUIPMENT : contains
    EQUIPMENT ||--o{ SLOT : owns
    SLOT ||--o{ OCCUPANCY : receives
    SUPPLY ||--o{ MATERIAL_INSTANCE : feeds
    MATERIAL_TYPE ||--o{ MATERIAL_INSTANCE : classifies
    MATERIAL_INSTANCE ||--o| OCCUPANCY : arrives
    SUPPLY ||--o{ SUPPLY_EPOCH : opens
    SUPPLY_EPOCH ||--o{ RESERVATION : issues
    OCCUPANCY ||--o{ RESERVATION : reserves
    SLOT ||--o{ TEACHING_BINDING : pick
    SLOT ||--o{ TEACHING_BINDING : place
    MATERIAL_TYPE ||--o{ TEACHING_BINDING : supports
    TEACHING ||--o{ TEACHING_BINDING : executes
```

| 식별자 | 범위와 수명 | 배포 예 |
|---|---|---|
| `slot_id` | Cell·설비·로컬 위치의 복합 주소. 자재 교체 후에도 유지 | `conveyor_pnp_cell/CV-01/pick`, `conveyor_pnp_cell/TR-01/receive` |
| `material_type_id` | 한 번에 한 품종과 수량을 투입 설정으로 받는다. 실제 생성 시 선택한 품종을 개체에 기록한다 | `ConveyorPart` |
| `material_instance_id` | 설비 스트림 UUID와 실제 생성 때의 PLC 자재 번호를 결합한다. 예약 epoch와 독립적이며 새 PLC 스트림의 첫 자재도 구분한다 | `<stream>/1`, `<stream>/2`, `<stream>/3` |
| `reservation_id` | 공급 epoch에서 특정 슬롯·품종·개체에 부여한 예약 | RESERVE 응답에서 발급 |

한 슬롯에는 한 번에 한 개체와 최대 한 개의 활성 예약이 있다. 작업 시작 전 BUSY로 반환하면 새 예약이 같은 점유를 이어받을 수 있다. Task가 BT에 진입한 개체는 같은 실행 프로세스에서 다시 Pick하지 않는다. 프로세스를 재시작할 때는 기존 PLC 스트림과 설비의 초기화·대조 계약을 따른다.

배포 설정의 품종 목록은 한 쌍의 Pick/Place 위치, 동일한 물리 규격 및 기존 흡착 티칭을 공유할 수 있는 품종을 명시한다. 운영자는 정지 상태에서 다음 공급 품종을 선택한다. 이미 투입된 개체는 기존 품종을 유지한다. 기본 PoC 프로필은 `ConveyorPart` 한 품종을 설치한다.
