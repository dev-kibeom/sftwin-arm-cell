# [SF-Twin] ARM Cell 외부 상태·Safety 인터페이스 계약

- **범위:** `AMR / PackML → Orchestration; Safety-HW → Safety Supervisor`
- **Owner:** `ARM Cell Shared Interface`

## 1. Endpoint와 Authority

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/amr/docking_report` | `arm_cell_interfaces/msg/AMRDockingState` | 외부 AMR Adapter | Orchestration의 작업 준비 상태 소비자 |
| `/packml/state` | `arm_cell_interfaces/msg/PackMLState` | 외부 PackML Adapter | Orchestration의 작업 준비 상태 소비자 |
| `/safety/hardware_state` | `arm_cell_interfaces/msg/SafetyHardwareState` | 외부 Safety-HW Adapter | Safety |

외부 Adapter는 상태와 Freshness 근거만 발행해야 한다. 정지 강도, Motion Capability, Safety Latch/Reset, 작업 승인 또는 Mission Result를 결정해서는 안 된다. 최종 Motion 안전 허가 및 Stop/Recovery의 유일한 Authority는 Safety이며, Orchestration은 기존 AMR/PackML Topic과 기타 준비 상태 입력을 이용한 작업 준비 상태 및 Admission의 유일한 Owner다. 이 계약은 Orchestration에 새로운 Message, Service 또는 Safety Authority를 추가하지 않는다.

## 2. 공유 Freshness 의미

모든 외부 상태 메시지는 `std_msgs/Header header`와 `bool valid`를 포함한다. `header.stamp`는 Source Adapter가 부여한 타임스탬프이며, `valid=false`인 Sample은 유효한 외부 상태를 확정할 수 없다.

AMR/PackML 소비자는 현재 ROS Clock Domain에서 Source Timestamp의 타당성과 자체 Monotonic Clock 기준의 로컬 Receipt Freshness를 평가한다. ROS 현재 시간보다 미래인 Source Timestamp나 Profile에 설정된 `source_timestamp_max_age_ms` 한계를 초과한 Timestamp는 Fresh AMR/PackML Sample의 근거가 될 수 없다. 이 한계는 메시지 상수가 아닌 Profile 설정이다. Publisher가 발행을 중단한 경우는 수신 Payload 무효, 알 수 없는 상태, 부적절한 Source Timestamp, Receipt Clock 불일치와 구분한다.

이들은 Communication Health 진단이지 독립적인 물리적 Safety Hazard가 아니다. AMR/PackML Timestamp와 전달 품질은 Safety Cause가 아닌 Communication Health 관측이다. Orchestration은 현재 작업에 해당 데이터가 필요한지를 판단하고 대응하는 Readiness/Progression 처리를 적용하며, 모든 작업에 일괄 Admission 선행 조건을 부과하지 않는다. 이 데모에서 이러한 외부 상태가 Safety Stop·Latch·Reset 동작을 만들지는 않는다. Safety-HW에는 5절의 별도 Receipt Freshness 계약이 적용된다.

## 3. AMR Docking 상태

[`AMRDockingState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/AMRDockingState.msg)의 필드는 `header`, `amr_id`, `current_node_id`, `docking_state`, `driving`, `valid`다.

`docking_state`는 `AMR_DOCKING_UNKNOWN`, `AMR_DOCKING_DOCKED`, `AMR_DOCKING_UNDOCKED`만 사용한다. 이는 ARM Cell이 지원하는 Docking Subset이며 전체 AMR/VDA 상태 모델은 아니다. Orchestration은 현재 자재 Delivery나 작업 진행에 필요한 경우에만 Docking/Driving을 평가한다. Unknown/Invalid 근거는 이에 의존하는 작업만 차단하며 Docking만으로 Material Readiness를 확정하지 않는다. 이 Simulation Demo에서 확인된 조기 Undocking은 Safety Interlock이 아니며 `UNDOCKED`/`driving` 값으로 Safety 정지 강도를 선택하지 않는다. 데모는 AMR–Robot 충돌 위험을 범위에서 제외한다. 실장비에는 물리적 위험 평가가 필요하다.

## 4. PackML 상태

[`PackMLState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/PackMLState.msg)의 필드는 `header`, `state`, `valid`다.

지원하는 정규 Subset은 `PACKML_STATE_UNKNOWN`, `PACKML_STATE_IDLE`, `PACKML_STATE_STARTING`, `PACKML_STATE_EXECUTE`, `PACKML_STATE_COMPLETE`, `PACKML_STATE_ABORTED`로 한정한다. 이는 전체 PackML State Machine 지원을 뜻하지 않는다. PackML은 외부 CNC/공정 상태이며 ARM Mission 상태가 아니다. Orchestration은 현재 Workflow가 해당 상태에 의존할 때에만 Unknown/Invalid 또는 작업과 맞지 않는 상태를 Not Ready로 취급한다. 일반 PackML 상태는 Safety Cause를 생성하지 않으며 `EXECUTE`만으로 로봇 움직임을 전역 금지하지 않는다. `ABORTED` 또한 Motion 실행 여부와 무관하게 Safety Cause·StopMotion·Latch·Reset을 일으키지 않는다. Orchestration은 현재 Workflow가 해당 CNC 상태에 의존할 때에만 대응하고, 일반 상태나 Timestamp 품질을 전역 PnP Admission Gate로 사용하지 않는다. Final Demo는 정규 CNC 상태를 읽기 전용 Isaac Indicator 투영으로 관측하며 CNC·공유 구역 점유 Interlock을 요구하지 않는다.

## 5. Safety Hardware 상태

[`SafetyHardwareState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/SafetyHardwareState.msg)의 필드는 `header`, `e_stop_active`, `sto_active`, `valid`다.

이 메시지는 현재 승인된 Supervisory Input만 전달하며, Safety는 Fail-closed Hardware-State Supervision을 위해 이를 소비한다. 무효·Unknown·Unavailable인 Hardware State 근거를 E-Stop/STO 해제 상태로 해석해서는 안 된다. 승인되지 않은 Protective-Stop Domain은 포함하지 않으며 별도의 Safety-rated E-Stop/STO 경로를 대체하지 않는다.

현재 Isaac Sim ARM Cell Demo에서는 Safety-HW의 의미적 유효성과 로컬 Monotonic Receipt Freshness가 Supervisory E-Stop/STO 상태를 확정할 수 있는지를 결정한다. Source Timestamp의 타당성과 Age는 `source_timestamp_max_age_ms`로 제한되는 별도 Communication Health 진단이다. Source Timestamp만 부적절하다고 해서 그 외에는 유효하고 로컬에서 Fresh한 Hardware State가 무효화되거나 Safety Cause, Capability Restriction, StopMotion, Latch가 발생하지는 않는다. Source 측정 시간이 안전에 필수적인 실장비에는 별도 계약 검토가 필요하다.

## 6. Adapter 교체 가능성

Mock과 실제 Adapter는 내부 구현이 달라도 Vendor 고유 Code가 Safety에 노출되는 의미가 되지 않도록 이 정규 Subset을 발행해야 한다. 외부 Mock 조건의 Clear는 발행된 외부 조건만 해제한다. Fresh하고 유효한 근거가 돌아오면 통신만으로 발생한 활성 차단은 해제하지만, 별도로 Latch된 Safety Cause나 명시적 Reset 정책을 우회하지는 않는다.

## 7. 검증 요구사항

### VR-ICD-EXT-SAFE-01 — 작업 범위에 한정된 AMR 의미

Orchestration은 현재 작업의 Material Delivery 또는 작업 진행이 AMR 상태에 의존하는 경우에만 평가해야 한다. 이 Simulation Demo에서 AMR 상태/통신만으로 Safety Cause·StopMotion·Latch·Reset을 발생시켜서는 안 된다.

### VR-ICD-EXT-SAFE-02 — PackML의 Safety Authority 비소유

`ABORTED`를 포함한 PackML 상태만으로 Safety Cause·StopMotion·Latch·Reset을 발생시켜서는 안 된다. Orchestration은 현재 작업 관련 상태에만 대응하고 일반 상태나 Timestamp 품질을 모든 PnP의 Admission Gate로 사용해서는 안 된다.

### VR-ICD-EXT-SAFE-03 — Hardware State 경계

`SafetyHardwareState`는 동기화 근거이며 Safety-rated 경로를 대체하지 않는다. Unavailable/Invalid 근거로 E-Stop/STO 해제를 확정해서는 안 된다.

### VR-ICD-EXT-SAFE-04 — 데모 위험 범위

Simulation Demo는 AMR–Robot 충돌 위험을 범위에서 제외한다. 실장비 사용에는 물리적 위험 평가와 승인된 Safety Design이 필요하다.
