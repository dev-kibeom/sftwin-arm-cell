# Configured Supply 인터페이스 계약

- **Owner:** Cell 공급 관리, Orchestration, Task Execution
- **범위:** 정렬 슬롯을 작업 원천으로 사용하는 ROS Mission. Vision Acquisition 계약과 별개다.

## 예약과 공급 사실

`/cell/configured_supply`의 `ConfiguredSupply` service가 RESERVE·COMPLETE·ABANDON을 제공한다.
RESERVE는 현재 epoch의 점유 슬롯 하나와 예약 ID를 반환한다. 활성 예약이 있으면 새 예약을 수락하지 않는다.
COMPLETE·ABANDON은 supply_id, supply_epoch, slot_id, reservation_id가 정확히 일치해야 한다.
개체를 추적하는 공급자는 RESERVE 응답에 `material_type_id`와 `material_instance_id`를 함께 반환한다. 상태의 두 필드는 한 자리 공급의 현재 점유를 나타내며, COMPLETE·ABANDON은 응답의 두 값을 그대로 돌려줘야 한다. Tape의 명목 슬롯에서는 두 필드가 빈 문자열이다.
COMPLETE는 성공한 Task 시퀀스와 같은 예약의 장치 Release 확인으로 해당 점유의 처리를 끝낸다.
material_placed는 기존 필드명이며 실제 수령·안착의 GT 증거가 아니다. 실제 자재 완료 수량은 집계하지 않는다.
Conveyor Mission에서 확인된 Task 종료의 ABANDON 응답이 유실되면 공급 서비스 재발견 후 동일 요청을 한 번 재전송할 수 있다. 중복·만료·이전 epoch 요청은 슬롯 진행을 바꾸지 않는다. ABANDON은 예약을 해제하며 확인된 목적지 Release를 기존 `material_placed` 필드로 함께 전달한다. Tape의 명목 슬롯은 기존 실패 정책을 따른다. Conveyor 공급자는 같은 예약의 Release가 확인됐으면 해당 도착 자재를 선택 대상에서 제외한다. 실패한 Mission을 성공으로 집계하지 않는다. Task 결과가 불명확하면 예약을 유지한다.

`/cell/configured_supply/state`는 ID·epoch·슬롯 점유·예약·호환용 completed_count(항상 0)·valid·소진 상태를 발행한다.
수신 신선도와 valid를 함께 평가하며, Topic 수신이나 슬롯 생성 자체를 파지 성공으로 취급하지 않는다.
Conveyor에서 새 Mission·Task의 수락과 BT 진입에는 500ms 미만의 유효한 표본과 해당 단계의 예약 조건이 필요하다. 수락되어 실행 중인 Task는 공급 표본이 지연되거나 공급자가 PLC·실물 대조 중임을 표시하면 기존 Goal과 예약을 유지한다. 대조 중인 표본은 `observation_pending=true, valid=false`이다. 대조가 끝난 표본에서 예약 철회, epoch·슬롯·품종·개체 변경 또는 유효성 상실이 확인되면 기존 실행을 중단한다. Safety 선점과 사용자 취소는 통신 복구를 기다리지 않는다.
Conveyor는 로봇 작업 중 같은 예약의 자재 개체와 품종이 정지된 벨트의 픽 위치에서 관측되고 스토퍼 전진과 빈 트레이가 확인되면 `reserved_material_at_pick=true`를 발행한다. 자재가 픽 위치를 벗어나면 예약이 유지되는 동안에도 이 값은 false다. 재파지를 시작하려면 현재 실행의 예약과 일치하는 500ms 미만의 유효한 표본에서 이 값이 true여야 한다.

Conveyor Mission은 관측 지연 500ms에 WARN, 5초에 ERROR, 정상화에 INFO를 기록한다. 같은 단절의 반복 관측은 합산한다. 목적지 Release가 확인된 뒤 COMPLETE 응답만 유실되면 Mission은 같은 요청으로 확인을 계속한다. 공급자는 원장에 이미 반영된 동일한 Release를 확인할 수 있다.

Tape Cell의 Timeline Stop은 이전 예약을 종료하고 다음 epoch를 빈 컨베이어와 공급 대기 상태에서 시작한다. Conveyor의 Timeline Stop은 설비 동작을 멈추며 미확정 자재·예약·결과를 대조 대상으로 보존한다. 같은 스트림에 재접속하면 현재 설비 관측과 대조하고, 명시적 Initialize가 완료되면 새 공급 epoch의 빈 상태에서 시작한다.

## Mission과 단일 Task

Configured-source Mission은 기존 `/orchestration/execute_cycle`을 사용한다. target_id는 등록 자재,
quantity는 단일 슬롯의 티칭 시퀀스를 요청하며 `quantity == 1`만 수락한다. 1이 아닌 요청은 Action 수락 단계에서
거부하고 슬롯 예약·Task 요청·진행 상태 변경을 수행하지 않는다. 이 source의 delivery_id는 zero UUID이며
외부 Delivery/AMR 인계를 가장하지 않는다. 기존 CNC Mission의 수량 계약은 변경하지 않는다.
Orchestration은 fresh 정상 Safety와 fresh 유효 공급을 요구하고 슬롯을 예약한 뒤 단일 Task를 요청한다.
Configured Task Action은 운영 Backend 초기화가 완료된 뒤 제공하며, configured Mission Action은
그 Task와 Cell 공급 service가 발견된 뒤 제공한다. 프로세스 시작이나 Safety 정상 상태만으로
하위 실행 준비를 가정하지 않는다. 이 준비는 장면 시작이나 Supply 요청 수락을 차단하지 않는다.

`/task_execution/execute_configured_grasp`의 `ExecuteConfiguredGrasp` goal은 공급·슬롯·epoch·예약 ID와 예약에서 받은 품종·개체 ID를 전달한다.
Task Execution은 현재 유효한 예약의 공급·자재·슬롯을 확인하고 배포된 기하·티칭을 선택한다. Tape는 예약 슬롯별 티칭을, Conveyor는 도착 자재들이 공유하는 고정 Pick/Place 티칭을 사용한다. Conveyor의 기존 `material_id`는 Mission과 PLC 프로필이 선택한 공급 대상이며, `material_type_id`는 투입 당시 개체의 품종이다. 고정 티칭은 배포된 설비 주소와 호환 품종이 둘 다 일치해야 선택된다. Task Execution이 예약과 실행을 연결한다. 흡착기의 구동과 보유 관측은 [Motion Backend ICD](icd-motion-backend.md)를 따른다. 호출자가 관절값·임의 Tool 기하·Vision 후보를 주입하지 않는다.
Task는 공통 GraspCycle BT와 동일 운영 MotionCore/GripperPort를 사용하고 ROS Goal UUID를 움직임·Tool 의도와 연결한다.
새 Task→Motion ROS 경계는 만들지 않는다.

Result는 result_code, material_placed, failure_stage, diagnostic_detail이다. 목적지에서 확인한 장치 해제 사실은 이후 Retreat 실패로 소거하지 않는다. Pick 실패 복구 중 원위치 Release는 배치 증거가 아니다.
실패한 시퀀스의 예약을 완료 처리하지 않는다.
단일 configured Mission은 관측한 Task terminal의 `material_placed`를 ExecuteCycle Result에
`has_material_placed=true`와 함께 그대로 전달한다. 해당 Task terminal이 없으면 유효성은 false다.
이 값은 슬롯 퇴역·Mission 성공·PLC 인수 여부와 독립적이다. 배치 후 Task 실패에 Safety 선점이 겹치면 Task의 실패 코드·단계·배치 사실을 보존하고, Mission은 [ExecuteCycle의 종료 우선순위](icd-orchestration-execute-cycle.md)에 따라 `MISSION_EXIT_SAFETY_PREEMPTED`로 종료한다.
런타임 종료는 작업 완료 검증이 아니다. 충돌 없는 1사이클의 작업 완료는
[Tape Cell FDS](../components/simulation/fds-tape-cell-supply.md)의 독립 진단으로 평가한다.
이 source의 batch_processed_count와 completed_count는 항상 0이며 물리 자재 수량을 뜻하지 않는다.
Task 실패 또는 Safety·취소는 Mission을 종료하며 자동 재시도하지 않는다. Task 수락/terminal이 불확실하면
예약을 유지하고 늦은 수락도 취소해 중복 처리를 막는다. 초기화된 공급은 해당 Cell의 새 공급 epoch에서 사용한다.

## Conveyor Mission 수락 표본

PLC 프로필의 `mission_admission_available`은 Configured Mission이 새 단일 작업을 받을 수 있는지 나타낸다. Mission 유휴, 예약 없음, 신선한 공급·정상 Safety 허가, Task·공급 서비스 발견 및 신선한 Motion 실행 상태를 사용한다. Goal 수락 시 대상·수량·Delivery ID도 확인한다. 수락한 작업의 실행 중에는 Motion이 EXECUTING으로 바뀌는 것이 정상이다. 실행 중단은 기존 Safety·공급·예약·취소 계약을 따른다.

Motion은 `/motion/state`의 Owner다. 활성 실행이 없는 정상 IDLE에는 이전 Stop 증거가 필요하지 않다. STOPPED는 Motion이 확인한 정지 완료와 정상 Safety 허가가 갖춰져야 새 생산 작업에 사용할 수 있다. STOPPING·EXECUTING·FAULTED·EMERGENCY_STOPPED·UNKNOWN 또는 오래된 표본에서는 새 작업을 보류한다. Task Execution은 공유 실행 mutex·예약 fence와 실행 결과를 소유한다. 실행 mutex 충돌로 BT 진입 전에 종료하면 `TASK_RESULT_BUSY`와 단계 `configured_supply:execution_admission`을 전달한다. PLC 프로필의 Mission만 이 조합을 별도 종료 사유로 전달한다. [Motion 상태](icd-safety-motion.md) · [Safety 및 Controlled Recovery](../components/orchestration/fds-controlled-recovery.md)

## 검증 조건

- 잘못된 예약·epoch·중복 COMPLETE는 물체나 슬롯 진행을 바꾸지 않는다.
- Safety/공급 신선도 손실, Goal 취소, 불확실 Task terminal은 새 슬롯 실행을 허용하지 않는다.
- Vision server 없이 실제 configured-source Task를 실행하되 검출 성공을 합성하지 않는다.
- 동일 예약은 성공한 시퀀스와 장치 Release 뒤 한 번만 퇴역하며 실제 자재 완료 수량을 집계하지 않는다.
- 자재 GT·동적 Stage 좌표·접촉 상대·정확한 물체/TCP 관계는 Mission/Task 준비·성공 입력이 아니다.
