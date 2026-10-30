# [SF-Twin] ARM Cell Material Readiness ↔ Orchestration Contract

- **Interface / Boundary ID:** `MATERIAL-READY`
- **Owner:** `ARM Cell Shared Interface`
- **Participants:** `Gateway ↔ Orchestration`
- **Related Design:** [Material Handoff FDS](../components/gateway/fds-material-handoff.md), [Batch Mission FDS](../components/orchestration/fds-mission-cycle.md)

## 1. 목적

기존 AMR Docking 및 PackML Interface는 자재 이송 완료 사실이나 Batch Admission 허가를 표현하지 않는다. 이 ICD는 Gateway가 소유하는 `MATERIAL_READY` 사실을 Orchestration에 전달하는 최소 공유 경계를 정의한다.

## 2. 범위

### 2.1 포함 범위

- Delivery ID 하나, 설정된 Material Type, Delivery별 요청 Quantity
- 요청 Quantity에 대한 Gateway 소유의 유효한 공급 완료 사실
- Delivery와 연관된 Transfer Completion Event 및 일회성 소비 의미
- Admission 경계에서의 중복 Delivery 처리

### 2.2 제외 범위

- AMR Navigation 또는 AMR Arrival/Docking 의미
- Orchestration이 소유하는 Batch Admission 결정
- Conveyor 동작 순서 또는 Simulator 구현
- ExecuteCycle과 Orchestration이 소유하는 Mission 진행 및 결과

## 3. 참여자

| 참여자 | 역할 |
|---|---|
| Gateway | Handoff 성공 이후 Transfer Complete / `MATERIAL_READY` 사실 발행 |
| Orchestration | 해당 사실 소비 후 현재 Readiness 및 Safety를 별도 평가하여 Batch Admission 결정 |

Endpoint `/integration/material_readiness`는 `arm_cell_interfaces/msg/MaterialReadiness`를 전달한다. 현재 Message 필드는 `std_msgs/Header header`, `unique_identifier_msgs/UUID delivery_id`, `string material_type`, `uint32 quantity`, `bool material_ready`, `bool valid`다. `delivery_id`·Type·요청 Quantity는 Gateway가 수락한 Handoff Trigger에서 복사한다. `quantity`는 양수여야 한다. 이는 Delivery별 공급 Quantity이며 과거 공급 자재 중 미처리분이 남았다면 다음 ExecuteCycle의 Quantity와 다를 수 있다.

`material_ready=true`는 식별된 Delivery에 대한 Gateway의 일회성 완료 Event다. 단일 Gateway 구현은 공통 External Handoff Trigger를 소비하고, 현재 유효하며 Identity·Type·Quantity가 일치하는 Provider Completion 근거와 필요한 VDA Context를 검증한다. Simulation에서는 Isaac Material Supply Adapter가, Production에서는 설정된 외부 공급자가 Provider다. 양쪽 모두 동일한 TransferCompletion 계약으로 보고하며 Gateway는 동일한 검증을 적용한다. 이 사실만으로 자재의 가시성·안정성·기하 유효성·파지 가능성 또는 PnP 성공을 주장하지 않는다.

`material_ready=false` 또는 `valid=false`는 완료 근거가 될 수 없다. 유효한 완료 확정 이후 일반적인 Source/Receipt Age만으로 완료 사실이 만료되지는 않는다. 수락된 Trigger의 `delivery_id`·Type·양수 Quantity가 Event 상관관계 기준이다.

## 4. 인터페이스 소유권

Gateway는 Provider Completion의 Identity·Type·Quantity·Validity·Freshness와 필수 계약 Context를 검증한다. `MATERIAL_READY`의 유일한 Publisher는 Gateway의 단일 Node다. Provider는 자체 물리적 완료 알고리즘을 소유한다. Isaac Spawn/Region 관측은 Simulation 구현 세부사항이며 이 공유 경계의 필수 조건이 아니다.

Batch Admission의 유일한 Authority는 Orchestration이다. 성공한 Transfer는 Delivery와 연관된 `MATERIAL_READY` Event 하나를 만든다. Orchestration은 이 사실을 소비하되 Workflow Readiness와 Safety Permission을 독립적으로 판단한다. 별도 운영자 또는 Hub Admission 요청은 이 경계에 없다. Delivery ID마다 독립된 사실을 가지며, 다른 Delivery나 ExecuteCycle이 활성 상태여도 추가 Delivery를 수락할 수 있다. `MATERIAL_READY`는 Admission의 필요조건이지만 충분조건은 아니다.

## 5. Delivery Event의 유효 범위 및 소비 의미

설정된 운영 Profile은 Reliable Transient-local 전달을 사용하여 늦은 Consumer가 최신 완료 사실을 받을 수 있게 한다. Gateway가 유효성을 확인해 확정한 `MATERIAL_READY`는 Source/Receipt Age만으로 만료되지 않으며 별도의 500ms Admission TTL은 없다.

Orchestration은 Delivery ID를 한 번만 소비한다. 중복 Event는 공급량을 중복 반영하지 않으며 서로 다른 Delivery ID의 완료 사실은 독립적으로 보존한다. 추가 Readiness는 활성 ExecuteCycle을 취소하거나 교체하지 않는다. 각 Cycle의 누적 미처리 수량, FIFO 귀속, PLACE 회계와 순차 Admission은 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다.

`ExecuteCycle.quantity`는 동일 Orchestration Process에서 알려진 공급 성공량 중 `object_disposition=PLACED`로 아직 완료 처리되지 않은 수량을 나타낸다. 이는 회계 상한이며 물체의 현재 존재·가용성을 보장하지 않는다. 회계의 수명과 갱신은 Mission Cycle FDS, Session 전환 시 새 Process가 시작되는 절차는 Simulation Session FDS가 소유한다.

## 6. 검증 요구사항

### VR-ICD-MATERIAL-01 — Readiness 소유권
선택된 Gateway의 검증된 Transfer Complete 사실만 `MATERIAL_READY`를 확정해야 한다. 유효한 사실은 Orchestration이 Readiness·Safety Permission을 기다리는 동안 Pending으로 보존된다. Pending 재평가와 Admission 시점은 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다.

### VR-ICD-MATERIAL-05 — 유효한 상관 공급 완료 필수
Provider Completion이 실패·부분 완료·Invalid·Stale 또는 불일치라면 긍정적 `MATERIAL_READY`가 관측되어서는 안 되며 Orchestration도 해당 Delivery의 Batch를 승인해서는 안 된다. Region Occupancy·Vision 가시성·안정성·파지 가능성은 Readiness 조건이 아니다.

### VR-ICD-MATERIAL-06 — 공급 Readiness에서 Scene Stability 제외
Supply Completion은 Provider가 보고한 요청 Quantity 충족만 요구한다. Vision Depth Scene Stability는 성공한 Supply Completion을 변경하거나 무효화하지 않는다. DetectTarget 요청마다 자체 Depth 관측·Geometry·Visibility 선행 조건을 독립적으로 평가한다.

### VR-ICD-MATERIAL-03 — 중복 Identity
동일 Orchestration Instance에서 한 Delivery Identity의 중복 Readiness Update를 받더라도, 후속 ExecuteCycle Rejection 또는 Failure와 무관하게 해당 Delivery는 최대 한 번만 소비해야 한다.

### VR-ICD-MATERIAL-04 — 전송 지연 이후에도 완료 사실 보존
유효하게 확정된 Completion은 Source/Receipt Age와 무관하게 Admission 후보로 남아야 한다. 형식이 잘못된 Tuple·Invalid Event·이미 소비된 `delivery_id`는 승인해서는 안 된다. Oracle은 Delivery Tuple, Completion/Validity 필드, Admission Identity와 Count다.

### VR-ICD-MATERIAL-07 — 추가 Delivery와 누적 Work
서로 다른 Delivery ID는 각각의 요청·공급 Quantity를 보존해야 한다(SHALL). 활성 ExecuteCycle 중 새 Readiness가 도착해도 기존 Cycle을 취소·대체해서는 안 된다(SHALL NOT). 후속 Cycle의 `quantity`는 성공 공급 수량에서 같은 Process에서 `PLACED`로 정확히 한 번 반영된 수량을 뺀 알려진 미처리량이며, 이전 Terminal Failure의 잔량도 포함한다. Delivery별 FIFO 귀속과 수량 갱신 순서는 Mission Cycle FDS가 소유한다. Oracle은 Delivery ID·요청/공급 수량, ExecuteCycle Identity·Quantity 및 중복 없는 `PLACED` 반영이다.

## 7. 전달과 구현의 경계

공유 계약은 Gateway가 게시하는 Delivery ID·Material Type·Delivery별 Quantity·Readiness·Validity와 완료 Event의 상관·중복 소비 의미다. `MaterialReadiness.msg`가 실행 가능한 필드 선언을 소유한다. 동일 Orchestration Process 내 Delivery 소비는 ID로 중복 방지하며 재시작을 넘는 영속적 Deduplication은 보장하지 않는다. 이 ICD는 Gateway 검증 절차나 Mission Accounting 알고리즘을 정의하지 않는다.


## 8. 관련 설계

- [External State ↔ Safety ICD](icd-external-safety.md)
- [Gateway CDS](../components/gateway/cds.md)
- [VDA CDS](../components/vda/cds.md)
