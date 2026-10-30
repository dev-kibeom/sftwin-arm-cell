# [SF-Twin] ARM Cell 외부 자재 인계 Trigger 공유 계약

- **인터페이스 / 경계 ID:** `EXTERNAL-MATERIAL-HANDOFF`
- **소유자:** ARM Cell 공유 인터페이스
- **참여자:** 외부 Trigger Source, Isaac Simulation UI Hub, Gateway, VDA Mock, 선택된 Material Supply Provider
- **관련 설계:** [Architecture Record 0011](../../../record/arm-cell/architecture-decision/external-material-handoff-trigger.md), [Material Handoff FDS](../components/gateway/fds-material-handoff.md)

## 1. 목적과 범위

외부에서 이미 배정된 자재 Delivery에 필요한 최소 Event 계약을 정의한다. ARM Cell은 Delivery를 요청·배정·Scheduling하지 않는다. 외부 Trigger는 Delivery Identity·Material Type·목표 Quantity를 지정한다. Demo Terminal과 Isaac UI Hub는 동일한 Event 의미를 사용한다. 현재 지원 자재는 `RawPart`이며 이 경계는 Job Scheduling·Database·다중 Material Process Logic을 추가하지 않는다. Publish Trigger 한 번은 기존 공급을 대체하는 요청이 아닌 **추가 공급 Event 하나**다. Simulation과 Production은 Gateway Trigger 계약 및 구현이 같고 Supply Provider만 다르다. Simulation UI Hub는 외부 공급자를 모사하는 Demo/Development 편의 입력원이며 로컬 Trigger 발행만 보고한다.

## 2. 이벤트 및 소유권

| Endpoint | ROS Type | Producer | Consumer |
|---|---|---|---|
| `/external/handoff_trigger` | `arm_cell_interfaces/msg/HandoffTrigger` | External Terminal/IT/MES/AMR Cell 또는 Isaac Demo Source | ARM Cell Gateway |
| `/integration/accepted_handoff` | `arm_cell_interfaces/msg/HandoffTrigger` | ARM Cell Gateway | VDA Mock 및 Isaac Material Supply Adapter |
| `/integration/handoff_trigger_status` | `arm_cell_interfaces/msg/HandoffTriggerStatus` | ARM Cell Gateway | Hub |
| `/vda/material_handoff_state` | `arm_cell_interfaces/msg/MaterialHandoffState` | VDA Mock | ARM Cell Gateway |
| `/material_supply/transfer_completion` | `arm_cell_interfaces/msg/TransferCompletion` | Supply Provider; Simulation에서는 Isaac Material Supply Adapter | ARM Cell Gateway |
| `/integration/material_readiness` | `arm_cell_interfaces/msg/MaterialReadiness` | ARM Cell Gateway | Orchestration 및 Hub |

`MaterialHandoffState`는 전달의 VDA 운송 사실을 Delivery별로 표현한다. `header.stamp`는 VDA 상태 생산자가 부여한 시각이고 `delivery_id`는 원본 Trigger의 Delivery Identity다. `phase`는 `HANDOFF_UNKNOWN=0`, `HANDOFF_ARRIVED=1`, `HANDOFF_DOCKED=2`, `HANDOFF_UNLOADED=3`, `HANDOFF_DEPARTED=4`, `HANDOFF_FAILED=5` 중 하나다. `valid`는 해당 상태 사실의 유효성을 나타내며, 알 수 없거나 유효하지 않은 상태는 성공 Phase로 해석하지 않는다.

정상 성공 Phase의 상호운용 순서는 `ARRIVED → DOCKED → UNLOADED → DEPARTED`다. 상태는 같은 전달의 `delivery_id`를 유지해야 하며 다른 전달의 증거로 귀속할 수 없다. 실패했거나 유효하지 않은 Phase는 후속 성공 Phase를 뜻하지 않는다. `UNLOADED`와 `DEPARTED`는 자재 공급 완료 또는 `MATERIAL_READY`를 뜻하지 않고, VDA 상태 자체는 Mission Admission을 승인하지 않는다. VDA 상태 생성과 Gateway의 검증 순서는 각 FDS가 소유한다.

`HandoffTrigger`에는 `std_msgs/Header header`, `unique_identifier_msgs/UUID delivery_id`, `uint32 quantity`, `string material_type`, `bool has_random_seed`, `uint64 random_seed`, `bool valid`가 포함된다. `quantity`는 0보다 커야 하고 `material_type`은 설정된 Material Profile을 식별해야 한다. 현재 Demo Profile은 `RawPart`다. 선택적인 `has_random_seed`/`random_seed`는 Isaac Demo Replay 자극으로, 동일 Scene·Profile·Adapter Version 및 Simulator Determinism 조건에서 Initial Pose Sampling을 반복할 수 있도록 한다. Production Supply 동작을 정의하지 않으며 실제 Provider는 이를 무시할 수 있다. Gateway는 Demo Trigger를 전달할 때 Seed 필드를 그대로 보존하지만 Seed 사용 여부는 수락·완료·Readiness를 바꾸지 않는다.

Trigger Producer는 `delivery_id`를 소유하고 Gateway는 수락 후에도 이를 보존한다. Gateway만 원본 Trigger의 수락·거부와 Accepted Event를 확정한다. Hub는 Trigger 발행 결과만 알리며 Gateway 수락, 공급 완료, `MATERIAL_READY` 또는 Mission Admission을 확정하지 않는다. 하위 근거가 누락·실패한 Delivery는 Ready가 아니며 다른 Delivery Identity를 대신 만들지 않는다.

Gateway의 수락은 Provider Completion이나 `MATERIAL_READY`와 별개의 사실이다. Trigger는 재소비할 수 없고, 각 수락 Delivery는 기존 Identity에 연관된 Completion으로 닫힌다. 서로 다른 유효 Delivery는 활성 ExecuteCycle, Motion Goal 또는 이미 인계된 PICK Pose를 취소·교체하지 않는다. 이들 시점과 처리 순서는 [Material Handoff FDS](../components/gateway/fds-material-handoff.md) 및 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다.
사용 가능한 0이 아닌 Delivery ID의 Trigger마다 Gateway는 검증 후 연관된 `HandoffTriggerStatus` 하나를 발행한다. 상태는 `ACCEPTED` 또는 `REJECTED`이며 거부에는 실제 Gateway Reason이 포함된다. Status는 Trigger 수락 여부만 확정하며 VDA 완료·물리적 공급·`MATERIAL_READY`·Mission Admission을 뜻하지 않는다. Hub는 Delivery ID로 연관시킨다. Status가 없으면 Unknown/Pending이며 암묵적인 수락이나 거부로 해석하지 않는다.

`HandoffTriggerStatus.msg`의 공유 필드는 다음과 같다.

| Field | 의미 |
|---|---|
| `header` | Gateway의 Status 발행 시각 |
| `delivery_id` | Source가 지정한 Trigger Identity |
| `state` | `TRIGGER_ACCEPTED` 또는 `TRIGGER_REJECTED` |
| `rejection_reason` | 안정적인 Reason Code; 수락 시 `NONE` |
| `diagnostic_detail` | 표시용 선택적 Gateway 상세 정보 |

거부 Reason Code는 `NONE`, `INVALID_TRIGGER`, `INVALID_DELIVERY_ID`, `STALE_SOURCE_TIMESTAMP`, `INVALID_QUANTITY`, `UNSUPPORTED_MATERIAL_TYPE`, `DUPLICATE_DELIVERY_ID`다. Status는 Source Delivery ID를 상관관계 식별에 사용할 수 있을 때만 발행한다. 이 Status는 Gateway의 Raw Trigger 판단만 설명하고 이후 Delivery 실패는 Transfer Completion/Readiness 경로로 보고한다.

`TransferCompletion`은 수락된 Delivery 하나에 대해 Provider가 Gateway로 전달하는 사실이다. Delivery Identity·Material Type·`requested_quantity`·Provider가 보고한 `supplied_quantity`·Completion Result·Validity·유한한 실행 실패 Reason을 포함한다. `supplied_quantity`는 Provider가 자체 Completion 계약으로 공급 완료를 확인한 수량이다. 생성 시도 횟수나 Region Occupancy 관측 수, Vision 검출 수량이 아니다. Isaac Material Supply Adapter에서는 RawPart 생성 및 Physics Drop 수행이 정상 완료된 수량을 뜻한다. Production Provider는 설비별 완료 조건으로 확인한 수량을 보고한다. 현재 ROS Message 필드는 `supplied_quantity`다.

성공 사실은 `supplied_quantity == requested_quantity`, 성공 Completion Result 및 Valid Evidence를 요구한다. 이 계약에 Isaac Prim·Region·Sensor·Simulator Observation을 필수로 추가해서는 안 된다. Gateway는 Delivery Identity·Material Type·요청 및 공급 Quantity·Validity·Freshness·Completion Result·필수 VDA Context를 검증하지만 Provider의 물리적 완료 알고리즘은 소유하지 않는다. `valid`는 Provider 사실의 계약 유효성을 뜻한다. Provider 전용 진단은 별도로 보존할 수 있으나 공유 Success 계약에 새로운 Predicate를 추가하지 않는다.

Simulation Provider의 정상 완료는 요청 수량의 Material 생성과 설정된 공급 동작 완료를 뜻한다. 이는 Scene Settling, Vision 가시성, 파지 가능성 또는 PnP 성공을 보장하지 않는다. Isaac 내부 Prim Identity·좌표·생성 객체 목록은 공유 Completion 계약에 포함되지 않는다. Simulation 구현 및 세션 종료 처리는 해당 FDS가 소유한다.

Trigger·Accepted Trigger·Completion Event는 기존 ROS Event Convention에 따라 Reliable/Volatile을 사용한다. Consumer는 기존 Profile 설정에 따라 Source Timestamp와 로컬 Receipt Freshness를 평가한다. 여기서 새로운 Freshness Duration을 정하지 않으며 VDA·Readiness QoS는 기존 Owner 계약을 따른다.

## 3. 완료 사실과 상호운용 조건

Gateway는 §2의 유효한 Provider Completion과 적용 가능한 VDA Context를 Delivery별로 결합해 Readiness를 확정한다. VDA `UNLOADED`/`DEPARTED`, Trigger 수락, Timer, UI 상태 또는 정적 Material 존재만으로 공급 Completion을 확정할 수 없다. Simulation의 Region Occupancy·Settling·Vision 가시성·파지 가능성은 공유 Completion 조건이 아니다.

`MATERIAL_READY`는 Gateway가 발행하는 Delivery별 공급 완료 사실이며 Admission의 필요조건일 뿐 Mission 승인·PnP 성공을 보장하지 않는다. Delivery별 누적 수량, 순차 Cycle과 실패 결과는 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다. Delivery Identity는 최대 한 번 소비하며 새로운 Trigger는 실패한 ExecuteCycle의 Replay가 아니다.

## 4. 데모 및 실제 운영 경계

External Terminal과 Isaac UI는 동일한 Trigger 필드·의미를 사용한다. Demo UI의 UUID 및 선택 Seed는 입력 자극 값이며 Hub에 Delivery 배정·관리 권한을 주지 않는다. VDA Phase는 운송 상태 사실이며 공급 Completion을 대신하지 않는다. Hub의 표시·Timeline 조작·Fault Injection 범위는 [UI Hub FDS](../components/ui-hub/fds.md)가 소유한다. Reset/Acknowledgement 요청은 [Safety Operator ICD](icd-safety-operator.md)를 따르며 Hub는 Safety 상태를 변경하지 않는다. Simulation 증거는 실제 현장 증거를 대체하지 않는다.

## 5. 실패 시 공유 의미

- 무효 Identity·0 이하 Quantity·알 수 없는 Material Profile·Stale/Invalid Trigger·활성 또는 완료된 중복 `delivery_id`는 새 Delivery를 생성하지 않는다. 거부된 Trigger는 Forward하지 않으며 ID를 상관 식별할 수 있으면 Gateway Rejection Status를 발행한다.
- 무효·실패한 VDA Phase 뒤에는 같은 Episode의 후속 Success Phase가 발생해서는 안 된다.
- Provider Failure·Partial Completion·Invalid Completion Fact는 실패한 Transfer Completion으로 관측되며 Positive `MATERIAL_READY`를 만들지 않는다.
- Trigger/Completion의 Delivery·Type·Quantity 불일치, Stale Evidence, Evidence 누락은 Fail-closed로 처리한다. Trigger Response·VDA Phase·Timer·UI 상태만으로 Readiness를 만들지 않는다.
- Reset/Acknowledgement는 기존 Safety 소유의 Request/Result 계약을 따른다. Material Supply는 Safety State를 Clear할 수 없다.

## 6. 검증 요구사항

### VR-ICD-EXT-MAT-01 — Single Trigger Acceptance와 Identity
Raw Trigger는 Gateway만 수락하고 Accepted Field를 변경 없이 전달해야 한다. VDA 및 Material Supply는 Forwarded Event만 소비한다. 중복 Delivery가 두 번째 Episode를 생성해서는 안 되며 Identity·Type·Quantity는 Completion과 Readiness까지 연관되어야 한다.

### VR-ICD-EXT-MAT-02 — Provider 공급 수량 완료 근거
Positive Transfer Completion은 수락된 요청 Quantity에 대한 Fresh·Valid·Delivery-correlated Provider Completion을 요구한다. Simulation은 자체 Creation·Physics Drop 수행을 검증하고 Production Provider는 설비별 Completion Fact를 제공한다. Timer·VDA Phase·UI 상태·기존 Material은 충분한 증거가 아니며 Isaac Region Observation은 공유 요구사항이 아니다.

### VR-ICD-EXT-MAT-03 — 공급 실패 보존
Provider 실행 실패·부분 수량·Invalid/Stale Fact·Correlation Mismatch는 관측 가능해야 하며 Positive Readiness를 생성하지 않아야 한다.

### VR-ICD-EXT-MAT-04 — 공급 Readiness와 Mission 결과의 구분
`MATERIAL_READY`는 Gateway 소유의 공급 완료 사실이다. 이 Event만으로 Safety Permission·Mission Admission·PnP 성공을 추론할 수 없다. Vision/Motion 결과와 수량 기반 Mission 종료는 각각 Vision, Motion 및 Mission Cycle FDS가 소유하며 공급 실패나 Trigger 재전송으로 재생성하지 않는다.

### VR-ICD-EXT-MAT-05 — Seed Replay와 보정하지 않는 Randomness
Scene·Material Profile·Adapter Version·Seed가 같으면 Sampled Initial Supply Pose가 재현되어야 한다. Sampler는 Vision/Motion/PnP 성공률을 높이려고 Pose를 최적화·거부·재배치·복구하지 않는다. Collision·Occlusion·접근 불가 Placement는 관측 가능한 결과로 남는다.

### VR-ICD-EXT-MAT-06 — Hub와 Authority 경계
Terminal과 Isaac UI는 의미가 동등한 Trigger를 발행한다. Hub는 Owner Fact 관측·Simulation Control·Fault Injection을 제공하지만 Delivery·Completion·Readiness·Admission·Safety State는 소유하지 않는다. Reset/Acknowledgement는 Safety 소유다.

### VR-ICD-EXT-MAT-07 — Provider 교체 경계
Isaac Demo Provider를 Production Provider로 교체할 때 Gateway가 Isaac Prim Identity·좌표·Spatial Observation을 소비하도록 요구해서는 안 된다. Simulation Evidence를 실제 현장 또는 Hardware Acceptance 근거로 격상하지 않는다.

### VR-ICD-EXT-MAT-08 — Acceptance Receipt 및 Rejection Provenance
사용 가능한 Delivery ID의 Trigger마다 Gateway는 실제 거부 사유를 포함한 연관 Accepted/Rejected Status를 발행해야 한다. Status가 없으면 Pending/Unavailable로 취급해야 한다. Oracle은 발행 Trigger·일치하는 Gateway Status·Acceptance일 때만 Forward된 Event·Hub 로컬 Publish Outcome과 Gateway의 정규 Acceptance Receipt 사이의 구분이다. 표시 범위는 [UI Hub FDS](../components/ui-hub/fds.md)가 소유하며 Hub에서 Acceptance를 표시하도록 요구하지 않는다.

### VR-ICD-INT-VDA-MAT-01 — 전달 식별자
한 전달의 VDA 상태 업데이트는 원본 Trigger의 동일한 `delivery_id`를 유지해야 하며 다른 Delivery의 업데이트를 해당 전달에 귀속해서는 안 된다.

### VR-ICD-INT-VDA-MAT-02 — 순서가 있는 인계 사실
정상 VDA 상태 사실은 `ARRIVED → DOCKED → UNLOADED → DEPARTED` 순서를 따른다. 실패했거나 유효하지 않은 Phase 뒤에 후속 성공 사실이 발생한 것으로 해석해서는 안 된다.

### VR-ICD-INT-VDA-MAT-03 — 인계 사실과 Readiness의 구분
`UNLOADED` 또는 `DEPARTED`는 Provider Completion이나 `MATERIAL_READY`가 아니다. VDA 상태는 Readiness 검증을 위한 Delivery 상관 근거이며 Mission Admission을 승인하지 않는다.


## 7. 계약 상태 및 구현 경계

Trigger Acceptance·Provider 독립 Completion 의미·Gateway Validation/Readiness·Quantity 기반 Batch Outcome이 설계 계약이다. `TransferCompletion.supplied_quantity`는 Provider가 확인한 공급량을 나타낸다. Region Count와 Spatial Observation Failure Category는 공유 Completion Predicate가 아니다. Isaac 로컬 진단은 Provider 경계 내부에 남아야 하며 Gateway는 Provider 내부 Class·Scene Path Set·Region Observation Algorithm에 의존해서는 안 된다. Provider Deadline은 미완료 실행을 제한하고 초과 시 실패로 처리한다. Physics Settling이나 Spatial Observation을 기다리는 시간이 아니다.

위 Topic과 Field의 공유 의미가 이 계약의 대상이다. 구현·통합 완료 여부는 현재 설계 계약으로 누적하지 않는다.

## 8. 관련 설계

- [Material Handoff FDS](../components/gateway/fds-material-handoff.md)
- [VDA Material Delivery FDS](../components/vda/fds-material-delivery.md)
- [Material Readiness ICD](icd-integration-orchestration-material.md)
- [ExecuteCycle ICD](icd-orchestration-execute-cycle.md)
- [Vision ↔ Orchestration ICD](icd-vision-orchestration.md)
- [RawPart Vision FDS](../components/vision/fds-rawpart-depth-perception.md)
- [Simulation Environment Semantics](../components/simulation-environment-semantics.md)
- [Simulation UI Hub FDS](../components/ui-hub/fds.md)
- [Safety Operator ICD](icd-safety-operator.md)
