# Configured Supply 인터페이스 계약

- **Owner:** Cell 공급 관리, Orchestration, Task Execution
- **범위:** 정렬 슬롯을 작업 원천으로 사용하는 ROS Mission. Vision Acquisition 계약과 별개다.

## 예약과 공급 사실

`/cell/configured_supply`의 `ConfiguredSupply` service가 RESERVE·COMPLETE·ABANDON을 제공한다.
RESERVE는 현재 epoch의 미완료 슬롯 하나와 예약 ID를 반환한다. 활성 예약이 있으면 새 예약을 수락하지 않는다.
COMPLETE·ABANDON은 supply_id, supply_epoch, slot_id, reservation_id가 정확히 일치해야 한다.
COMPLETE는 성공한 Task 시퀀스와 같은 예약의 장치 Release 확인으로 슬롯을 퇴역한다.
material_placed는 기존 필드명이며 실제 수령·안착의 GT 증거가 아니다. 실제 자재 완료 수량은 집계하지 않는다.
중복·만료·이전 epoch 요청은 슬롯 진행을 바꾸지 않는다. ABANDON은 명목 슬롯 상태를 보존한다.

`/cell/configured_supply/state`는 ID·epoch·슬롯 점유·예약·호환용 completed_count(항상 0)·valid·소진 상태를 발행한다.
수신 신선도와 valid를 함께 평가하며, Topic 수신이나 슬롯 생성 자체를 파지 성공으로 취급하지 않는다.
Stop은 새 epoch와 Cell 초기 공급 상태·집계 없음을 복원하며 이전 예약·결과를 이월하지 않는다.
Tape Cell의 초기 상태는 빈 컨베이어와 공급 대기다.

## Mission과 단일 Task

Configured-source Mission은 기존 `/orchestration/execute_cycle`을 사용한다. target_id는 등록 자재,
quantity는 단일 슬롯의 티칭 시퀀스를 요청하며 `quantity == 1`만 수락한다. 1이 아닌 요청은 Action 수락 단계에서
거부하고 슬롯 예약·Task 요청·진행 상태 변경을 수행하지 않는다. 이 source의 delivery_id는 zero UUID이며
외부 Delivery/AMR 인계를 가장하지 않는다. 기존 CNC Mission의 수량 계약은 변경하지 않는다.
Orchestration은 fresh 정상 Safety와 fresh 유효 공급을 요구하고 슬롯을 예약한 뒤 단일 Task를 요청한다.
Configured Task Action은 운영 Backend 초기화가 완료된 뒤 제공하며, configured Mission Action은
그 Task와 Cell 공급 service가 발견된 뒤 제공한다. 프로세스 시작이나 Safety 정상 상태만으로
하위 실행 준비를 가정하지 않는다. 이 준비는 장면 시작이나 Supply 요청 수락을 차단하지 않는다.

`/task_execution/execute_configured_grasp`의 `ExecuteConfiguredGrasp` goal은 공급·슬롯·epoch·예약 ID만 가진다.
Task Execution이 배포된 registry에서 기하·티칭을 찾는다. 호출자가 관절값·임의 Tool 기하·Vision 후보를 주입하지 않는다.
Task는 공통 GraspCycle BT와 동일 운영 MotionCore/GripperPort를 사용하고 ROS Goal UUID를 움직임·Tool 의도와 연결한다.
새 Task→Motion ROS 경계는 만들지 않는다.

Result는 result_code, material_placed, failure_stage, diagnostic_detail이다. 장치 해제 사실은
이후 Retreat 실패로 소거하지 않지만 실패한 시퀀스의 예약을 완료 처리하지 않는다.
런타임 종료는 작업 완료 검증이 아니다. 충돌 없는 1사이클의 작업 완료는
[Tape Cell FDS](../components/simulation/fds-tape-cell-supply.md)의 독립 진단으로 평가한다.
이 source의 batch_processed_count와 completed_count는 항상 0이며 물리 자재 수량을 뜻하지 않는다.
Task 실패 또는 Safety·취소는 Mission을 종료하며 자동 재시도하지 않는다. Task 수락/terminal이 불확실하면
예약을 유지하고 늦은 수락도 취소해 중복 처리를 막는다. Stop 이후 새 세션에서만 초기화된 공급을 사용한다.

## 검증 조건

- 잘못된 예약·epoch·중복 COMPLETE는 물체나 슬롯 진행을 바꾸지 않는다.
- Safety/공급 신선도 손실, Goal 취소, 불확실 Task terminal은 새 슬롯 실행을 허용하지 않는다.
- Vision server 없이 실제 configured-source Task를 실행하되 검출 성공을 합성하지 않는다.
- 동일 예약은 성공한 시퀀스와 장치 Release 뒤 한 번만 퇴역하며 실제 자재 완료 수량을 집계하지 않는다.
- 자재 GT·동적 Stage 좌표·접촉 상대·정확한 물체/TCP 관계는 Mission/Task 준비·성공 입력이 아니다.
