# [SF-Twin] ARM Cell ExecuteCycle 인터페이스 계약

- **범위:** `Orchestration Batch Admission/Execution`
- **소유자:** `ARM Cell Shared Interface`

## 1. 엔드포인트와 식별자

| 엔드포인트 | ROS 타입 | 생산자 | 소비자 |
|---|---|---|---|
| `/orchestration/execute_cycle` | `arm_cell_interfaces/action/ExecuteCycle` | Orchestration Batch 승인 제어기 | Orchestration Batch Mission 실행기 |

Goal은 설정된 작업 Profile 키 `target_id`, Admission을 촉발한 `delivery_id`, 그리고 양수의 알려진 미처리 작업량 `quantity`를 공유한다. `delivery_id`는 공급 Event의 Identity이며, `target_id`는 인지 Algorithm Parameter가 아니다. 과거 잔여 공급량이 없는 첫 Cycle에서 `quantity`는 해당 Delivery의 성공 공급량이다. 후속 Cycle에서는 같은 Orchestration Process에서 공급됐지만 Terminal Motion Result의 `object_disposition=PLACED`로 아직 반영되지 않은 알려진 수량을 포함한다. 자재 공급·PLACE 회계 의미는 [Material Readiness ICD](icd-integration-orchestration-material.md), Mission 계산·진행 절차는 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다. ROS Action Goal UUID는 이 ExecuteCycle의 Identity다.

ExecuteCycle은 Orchestration이 발행하는 내부 Mission 실행 경계다. 외부 Hub는 이 Action으로 Batch를 승인하거나 실패한 Mission을 재시작하지 않는다. Orchestration은 Delivery ID를 최대 한 번 소비하며 Goal 제출·수락·Terminal Result는 서로 다른 사실이다. 구체 Admission, 단일 활성 Mission, retry 및 recovery 절차는 Mission Cycle FDS를 따른다.

## 2. Feedback

실행 인터페이스는 [`ExecuteCycle.action`](../../../../ros2_ws/src/interfaces/arm_cell_interfaces/action/ExecuteCycle.action)이다. `MissionPhase.value`는 `MISSION_PHASE_ENSURING_HOME`, `MISSION_PHASE_DETECTING_TARGET`, `MISSION_PHASE_PICKING`, `MISSION_PHASE_PLACING`, `MISSION_PHASE_RETURNING_HOME`, `MISSION_PHASE_WAITING_RECOVERY`, `MISSION_PHASE_RETRACTING`, `MISSION_PHASE_FINISHED`를 사용하며, `MISSION_PHASE_UNKNOWN`은 미설정 값이다.

이 Phase는 공개 Mission 진행 상황만 전달하며 BehaviorTree 구현 세부사항을 노출하지 않는다. 내부 BehaviorTree.CPP 트리 및 Node Status는 ROS Action 계약의 일부가 아니다. Goal·Feedback·Result·식별자·취소 의미는 이 문서의 정의를 유지한다.

Action Feedback은 Mission 진행과 수량·Retry·중단 사실을 관측하기 위한 공유 Snapshot이다. 실행 가능한 Field Type은 [`ExecuteCycle.action`](../../../../ros2_ws/src/interfaces/arm_cell_interfaces/action/ExecuteCycle.action)이 소유하며, 이 ICD는 소비자가 동일하게 해석해야 하는 의미를 정의한다.

| 필드 | 의미 |
| --- | --- |
| `delivery_id` | 승인된 Batch와 연결된 Gateway Delivery UUID |
| `quantity` | 이 ExecuteCycle의 누적 미완료 작업에 대해 `object_disposition=PLACED`로 완료해야 하는 배치 수 |
| `batch_processed_count` | 이 ExecuteCycle에서 Terminal Motion Result의 `object_disposition=PLACED`로 확인한 배치 수. 정상 PLACE 성공 및 정상 목표 해제 후 Retreat 실패를 각각 정확히 한 번 반영하며 `RETURNED`·`UNCHANGED`는 포함하지 않음 |
| `retry_count` | 현재 Target Episode에서 사용한 Vision 관측 재시도 횟수. 재시도 자격·예산과 초기화 시점은 Mission Cycle FDS가 소유 |
| `selected_target_pose` / `has_selected_target_pose` | 활성 반복에서 실제로 사용한 객체 기준 Pose |
| `target_valid` | 최근 Vision 관측에서 선택 Target의 유효성 |
| `target_observation_stamp` | 선택한 Vision 관측 Stamp |
| `interruption_count` | Batch에서 관측된 Safety·Task 중단 횟수 |
| `first_interruption_reason` | 처음 발생한 중단의 정식 분류. Batch 수명 동안 보존 |
| `first_interruption_causes` | 첫 Safety 중단 당시 수집한 Safety 원인 집합 |
| `first_interruption_stop_mode` | 첫 Safety 중단 당시 선택된 심각도 |

`target_valid`와 `target_observation_stamp`는 최근 Vision 관측의 Mission 진행 정보다. 이미 제출한 Motion Goal의 Pose를 변경하거나 Live Freshness/Scene Stability Gate로 재사용하지 않는다. `retry_count`는 현재 Target Episode의 Vision 관측 Retry 관측값이며 새 Mission 요청의 수가 아니다. 각 Feedback은 현재 ExecuteCycle Goal UUID에, `delivery_id`는 해당 공급 Event에 연결된다. 첫 중단의 reason·causes·stop mode는 해당 Cycle에서 관측된 최초 중단을 가리키며 뒤의 중단이 이를 덮어쓰지 않는다. 실제 Retry·Recovery·중단된 Task의 수명은 Mission Cycle FDS가 소유한다.

## 3. 결과 및 취소

ExecuteCycle Goal 제출은 Delivery Admission 이후 Orchestration이 소유한다. Goal 수락은 Mission 성공을 뜻하지 않고, Caller 취소는 Action 수명에 대한 요청이다. Timeline Session 전환·프로세스 종료와 Safety Stop/Reset의 관계는 [Simulation Session FDS](../components/simulation/fds-sim-session.md) 및 해당 Safety 계약을 따른다.


Result에는 `MissionExitReason exit_reason`과 비규범적 `diagnostic_detail`이 포함된다. 종료 상수는 다음과 같다.

| 상수 | 의미 |
|---|---|
| `MISSION_EXIT_NONE` | 아직 종료 사유가 확정되지 않음 |
| `MISSION_EXIT_DEPLETED` | 과거 소진 결과. Final Demo 수량 기반 Batch의 정상 완료에는 사용하지 않음 |
| `MISSION_EXIT_QUANTITY_REACHED` | `batch_processed_count == quantity`이고 최종 단일 GO_HOME 성공. 수량 기반 정상 완료 |
| `MISSION_EXIT_QUANTITY_SHORTFALL` | `quantity` 달성 전 유효한 Target 소진. Safety 상태·심각도가 아닌 비성공 Soft Error |
| `MISSION_EXIT_VISION_ERROR` | 소진 외 Vision 결과로 Mission 중단 |
| `MISSION_EXIT_MOTION_ERROR` | Motion Task 실패로 Mission 중단 |
| `MISSION_EXIT_SAFETY_PREEMPTED` | Safety가 Mission 선점 |
| `MISSION_EXIT_CANCELED` | Caller의 취소로 Mission 중단 |
| `MISSION_EXIT_CONFIGURATION_ERROR` | Orchestration이 요청된 Mission Recipe·설정을 로드하거나 검증할 수 없음 |

로컬 비Safety 종료 판단 경계마다 Orchestration은 Caller 취소·Vision 결과·Motion 결과를 확정하기 직전에 현재 Safety 상태를 다시 관측해야 한다(SHALL). 현재 Safety 상태가 정상 Motion을 더 이상 허용하지 않으면 결과는 `MISSION_EXIT_SAFETY_PREEMPTED`여야 한다(SHALL). 그렇지 않을 때만 해당 비Safety 결과를 확정할 수 있다. 이 규칙은 결정적인 로컬 재검증이지 분산 컴포넌트 전체의 원자적 관측을 주장하지 않는다. Safety 선점은 구별 가능하게 유지되며 로봇 정지는 Action 취소에 의존하지 않는다.

현재 수량 기반 Cycle의 유일한 정상 완료 결과는 `MISSION_EXIT_QUANTITY_REACHED`이며, `batch_processed_count == quantity`와 최종 성공 GO_HOME을 함께 뜻한다. `MISSION_EXIT_QUANTITY_SHORTFALL`은 수량 도달 전 유효한 Target 부재의 비성공 결과이며 ROS Action Terminal은 ABORTED다. `MISSION_EXIT_DEPLETED`는 기존 호환 결과 값으로 남지만 현재 수량 기반 정상 완료를 뜻하지 않는다. 수량 진행, Retry/Recovery 자격, 최종 결과 확정 순서는 Mission Cycle FDS가 소유한다. `diagnostic_detail`은 비규범적이며 기계 판정에 사용하지 않는다.

`MISSION_EXIT_CONFIGURATION_ERROR`는 Orchestration 소유 결과다. 요청한 `target_id`의 Recipe가 없거나 필수 Grasp Policy 또는 PLACE Pose·Frame이 누락·무효인 경우 적용한다. Vision 또는 Motion 실패가 아니다.

ROS Action Terminal 상태는 정식 `MissionExitReason`과 구분된다. `MISSION_EXIT_QUANTITY_REACHED`는 SUCCEEDED, Caller 취소(`MISSION_EXIT_CANCELED`)는 CANCELED에 매핑해야 한다(SHALL). 수량 부족·Vision·Motion·Safety·설정 중단은 ABORTED에 매핑해야 한다(SHALL). Result Payload는 정식 사유를 보존한다. 비Safety 결과를 확정하기 전 기존 Terminal Safety 재검증 및 우선순위를 적용한다.

## 4. 검증 요구사항

### VR-ICD-ORCH-CYCLE-06 — 수량 기반 완료
`batch_processed_count == quantity`이고 최종 GO_HOME에 성공한 `MISSION_EXIT_QUANTITY_REACHED`만 현재 Cycle의 정상 완료를 나타내야 한다(SHALL). 수량 도달 전 유효한 Target 부재는 `MISSION_EXIT_QUANTITY_SHORTFALL`이며 정상 완료가 아니다. `object_disposition=PLACED`만 해당 Cycle의 처리 수량으로 반영하고, `RETURNED`는 처리 수량을 늘리지 않는다. 후속 Cycle의 미처리 Quantity 산정은 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md) 및 [Material Readiness ICD](icd-integration-orchestration-material.md)를 따른다.

### VR-ICD-ORCH-CYCLE-02 — 결과 및 출처 보존
복구 완료는 Batch의 첫 중단 출처를 삭제하거나 다시 기록해서는 안 된다(SHALL NOT). Batch Terminal 결과는 최종 Batch 결과를 나타내야 하며(SHALL) 중단된 Task·Iteration의 결과도 독립적으로 관측 가능해야 한다.

### VR-ICD-ORCH-CYCLE-03 — 선점 구분
Safety 선점과 Caller 취소는 구분 가능하게 유지해야 한다(SHALL).

### VR-ICD-ORCH-CYCLE-04 — Terminal Safety 재검증
Caller·Vision·Motion에서 비Safety 종료 결과가 발생해도 Orchestration의 종료 판단 경계에서 현재 Safety 상태가 정상 Motion을 선점하면 공개 결과는 `MISSION_EXIT_SAFETY_PREEMPTED`여야 한다(SHALL).

### VR-ICD-ORCH-CYCLE-07 — 수량 부족 결과 매핑
`batch_processed_count < quantity`인 상태에서 유효한 Target 소진이 발생하면 결과는 `MISSION_EXIT_QUANTITY_SHORTFALL`이어야 하고(SHALL), Action은 Safety 상태·심각도 도입 없이 ABORTED여야 한다(SHALL).

### VR-ICD-ORCH-CYCLE-05 — 설정 결과 매핑
Orchestration이 소유한 Mission Recipe가 누락되거나 유효하지 않다면 ExecuteCycle은 `MISSION_EXIT_CONFIGURATION_ERROR`를 반환해야 하며(SHALL), ROS Action Terminal 상태는 ABORTED여야 한다(SHALL).

### VR-ICD-ORCH-CYCLE-10 — 단일 Mission과 취소 의미
동일 Orchestration Instance는 한 번에 하나의 ExecuteCycle만 활성화해야 한다(SHALL). 다른 Delivery의 Readiness나 외부 관측자는 활성 Action을 취소·교체하거나 새 ExecuteCycle을 병렬 실행하지 않는다(SHALL NOT). 프로세스·Session 전환 시의 실행·Accounting 수명은 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)와 [Simulation Session FDS](../components/simulation/fds-sim-session.md)가 소유한다.

## 5. 계약과 구현의 경계

정상 ExecuteCycle 계약은 내부의 일회성 Delivery 승인 경로를 유지한다. 실행 세션 Service나 Timeline 기반 승인 Fence는 이 계약에 포함되지 않는다.
