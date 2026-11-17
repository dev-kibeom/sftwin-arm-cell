# Configured Supply 인터페이스 계약

- **Owner:** Cell 공급 관리, Orchestration, Task Execution
- **범위:** 정렬 슬롯을 작업 원천으로 사용하는 ROS Mission. Vision Acquisition 계약과 별개다.

## 예약과 공급 사실

`/cell/configured_supply`의 `ConfiguredSupply` service가 RESERVE·COMPLETE·ABANDON을 제공한다.
RESERVE는 현재 epoch의 미완료 슬롯 하나와 예약 ID를 반환한다. 활성 예약이 있으면 새 예약을 수락하지 않는다.
COMPLETE·ABANDON은 supply_id, supply_epoch, slot_id, reservation_id가 정확히 일치해야 한다.
COMPLETE는 Task의 material_placed 입력과 Cell의 실제 수령 위치·안정성·물리 Release 확인을 모두 요구한다.
중복·만료·이전 epoch 요청은 수량을 바꾸지 않는다. ABANDON은 점유와 완료 수량을 보존한다.

`/cell/configured_supply/state`는 ID·epoch·슬롯 점유·예약·완료 수량·valid·소진 상태를 발행한다.
수신 신선도와 valid를 함께 평가하며, Topic 수신이나 슬롯 생성 자체를 파지 성공으로 취급하지 않는다.
Stop은 새 epoch와 초기 카트리지·완료 0을 만들며 이전 예약·결과를 이월하지 않는다.

## Mission과 단일 Task

Configured-source Mission은 기존 `/orchestration/execute_cycle`을 사용한다. target_id는 등록 자재,
quantity는 양의 처리 수량이다. 이 source의 delivery_id는 zero UUID이며 외부 Delivery/AMR 인계를 가장하지 않는다.
Orchestration은 fresh 정상 Safety와 fresh 유효 공급을 요구하고 슬롯을 예약한 뒤 단일 Task를 요청한다.

`/task_execution/execute_configured_grasp`의 `ExecuteConfiguredGrasp` goal은 공급·슬롯·epoch·예약 ID만 가진다.
Task Execution이 배포된 registry에서 기하·티칭을 찾는다. 호출자가 관절값·임의 Tool 기하·Vision 후보를 주입하지 않는다.
Task는 공통 GraspCycle BT와 동일 운영 MotionCore/GripperPort를 사용하고 ROS Goal UUID를 움직임·Tool 의도와 연결한다.
새 Task→Motion ROS 경계는 만들지 않는다.

Result는 result_code, material_placed, failure_stage, diagnostic_detail이다. 확인된 실제 해제 사실은
이후 Scene 인계·Retreat 실패로 소거하지 않는다. Cell이 실제 수령을 확인해야 공급 완료를 반영한다.
Task 실패 또는 Safety·취소는 Mission을 종료하며 자동 재시도하지 않는다. Task 수락/terminal이 불확실하면
예약을 유지하고 늦은 수락도 취소해 중복 처리를 막는다. Stop 이후 새 세션에서만 초기화된 공급을 사용한다.

## 검증 조건

- 잘못된 예약·epoch·중복 COMPLETE는 물체나 수량을 바꾸지 않는다.
- Safety/공급 신선도 손실, Goal 취소, 불확실 Task terminal은 새 슬롯 실행을 허용하지 않는다.
- Vision server 없이 실제 configured-source Task를 실행하되 검출 성공을 합성하지 않는다.
- 수량은 실제 수령·Release 근거 뒤에만 한 번 반영한다.
