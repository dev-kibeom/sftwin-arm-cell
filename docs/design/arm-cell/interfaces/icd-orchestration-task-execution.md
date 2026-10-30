# [SF-Twin] Orchestration–Task Execution PnP·교란 공유 계약

- **Contract ID:** `ARM-CELL-ORCH-TASK-EXECUTION`
- **Semantic Owner:** Task Execution은 PnP 결과 및 교란 Action lifecycle, Orchestration은 Mission 입력·수량·상위 재관측·재시도 정책
- **Consumers:** Orchestration, Task Execution

## 1. 공유 계약 경계

이 ICD는 Orchestration이 요청하는 단일 PnP와 별도 교란 operation, Task Execution이 확정하는 외부 결과를
정의한다. Orchestration은 Goal 입력, terminal 결과의 상위 의미, 자재 배치 사실 및 선택적
다음 Acquisition 선행 알림만 소비한다. 내부 PICK/PLACE 단계, Gripper 관측, 후보 전환,
개별 Motion 결과와 복구 원인은 이 경계를 통해 Mission 판단 입력으로 노출하지 않는다.

Mission 순서·수량·상위 재관측 정책은 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md),
PnP 실행·복구·결과 매핑은 [Task Execution PnP FDS](../components/task-execution/fds-pnp-execution.md)가
각각 소유한다.

## 2. Endpoint 및 Goal

| 인터페이스 | Producer | Consumer | 상호작용 |
|---|---|---|---|
| `ExecutePnp` (`/task_execution/execute_pnp`) | Orchestration | Task Execution | 단일 PnP ROS Action; Goal UUID가 execution identity |
| `ExecuteDisturb` (`/task_execution/execute_disturb`) | Orchestration | Task Execution | 단일 교란 ROS Action; Goal UUID가 요청 identity |

Goal은 하나의 Vision Acquisition과 Orchestration이 확정한 Recipe snapshot을 나타낸다.

| 입력 | 의미·제약 |
|---|---|
| `vision_acquisition_id` | 전체 후보가 나온 하나의 Acquisition identity |
| `material_candidates[]` | 동일 Acquisition의 SELECTED HIGH·MEDIUM 정상 PICK 적격 후보 전체와 후보별 관측 Pose·Geometry 근거를 담은 단일 목록. Mission은 전체 목록을 변경 없이 전달하며 내부 순위와 전환은 Task Execution 소유; Candidate Grade 필드는 추가하지 않음 |
| `place_target_pose` + `has_place_target_pose` | Recipe destination pose. PICK candidate pose와 별도인 필수 입력 |
| Recipe snapshot fields | 기존 PICK/PLACE geometry·approach·tolerance·orientation·grasp width 계약을 변경 없이 전달 |
| Action Goal UUID | 불변 PnP identity; 내부 movement 및 Cancel 상관관계에 사용 |

기존 개별 `ExecuteTask.Goal.target_pose`는 source 후보와 destination을 함께 표현하지 못한다.
따라서 PnP Goal에는 PLACE target pose와 availability bit가 필요하다. 나머지 Recipe·candidate
의미는 기존 필드를 재사용한다. 새 Action의 종류가 PnP 하나로 고정되므로 `task_type`은
제거할 수 있다.

### ExecuteDisturb 공유 계약

Task Execution은 정상 `ExecutePnp`와 독립된 `ExecuteDisturb` Action을 제공한다. 이 Action은 요청 수락, 기존 lifecycle feedback, terminal 전달만 소유한다. 후보 선택·접근·밀기·후퇴와 Motion operation 결과의 의미는 [Motion 교란 실행 FDS](../components/motion/fds-disturb-execution.md)가, Mission 진입·재관측 반복은 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다.

| Goal 필드 | 의미 | 무효·비보장 |
|---|---|---|
| `acquisition_id`, `scene_generation` | 동일 Vision Acquisition·scene 관측의 상관관계 | 다른 관측의 후보 혼합 금지 |
| `disturbance_candidates` | Vision이 반환한 전체 미확정 관측 후보 원본 | PICK 적격성·접근 가능성 보장 아님; 순서 재정렬 금지 |

| Result 필드 | 의미 | 무효·존재 조건 |
|---|---|---|
| `outcome=COMPLETED` | Motion이 계획한 접근·밀기·후퇴 동작의 측정 완료 | 객체 이동, 재배치, PICK 성공을 보장하지 않음 |
| `outcome=FAILED` + `failure_reason` | 계획 탐색 소진 또는 실제 실행 실패 | `PLAN_EXHAUSTED`와 `EXECUTION_FAILURE`를 구분; 실행 실패를 후보 전환으로 바꾸지 않음 |
| `outcome=CANCELED` + 실제 `stop_evidence`가 있으면 보존 | 요청자 명시적 Cancel; Motion 실행이 실제 중단된 경우 기존 software stop 완료 근거 | Cancel 수락 또는 실행 미시작만으로 `backend_inactivity_confirmed`를 추론하지 않음 |
| `outcome=SAFETY_PREEMPTED` + `safety_cause` + 관측된 stop evidence | Safety direct Stop 선점, Safety 원인과 존재하는 기존 software stop 완료 근거 | 일반 Cancel이나 계획 실패로 합치지 않으며, stop evidence가 관측되지 않았으면 만들어내지 않음 |

`backend_inactivity_confirmed`는 Motion 계약의 실제 관측된 근거를 뜻한다. 이는 명령 생산자·Action 종료와 Stop/Hold 전달 완료를 뜻하며 물리적 완전 정지나 Hardware Safety 인증이 아니다. 실행이 시작되지 않은 Goal의 `CANCELED` 결과에는 이 근거가 없을 수 있다. ROS terminal status는 `COMPLETED`→SUCCEEDED, 요청자 `CANCELED`→CANCELED, `FAILED`·`SAFETY_PREEMPTED`→ABORTED로 매핑한다. `outcome`은 terminal status와 별도 보존되는 정규 도메인 결과이며 조합은 이 매핑과 모순되어서는 안 된다. `material_placed`, PnP 성공 또는 처리 수량을 만들지 않는다. Feedback은 Action 진행 관측이며 Mission 진행 수량의 근거가 아니다.

## 3. Feedback 및 Terminal Result

| 계약 부분 | 공유 의미 |
|---|---|
| Feedback `phase` | 기존 typed progress를 진행·진단 관측으로 제공; 실행 승인이나 결과 재분류 근거가 아님 |
| Feedback `next_acquisition_hint` | 호환용 진행 힌트. Orchestration Mission은 `NONE`·`PREFETCH_NEXT` 값으로 Acquisition을 시작하거나 순서를 변경하지 않음 |
| 기존 release/candidate progress | migration 중 필요한 기존 진행·진단 정보로만 유지; Mission policy input이 아님 |
| Terminal `pnp_outcome` | 최소 상위 의미: `COMPLETED`, `RETRYABLE`, `FAILED`, `SAFETY_PREEMPTED`, `CANDIDATES_EXHAUSTED`. `RETRYABLE`은 새 Acquisition 기반 재시도가 가능한 작업 결과를 뜻하며 재시도 승인 자체는 아님 |
| Terminal `PNP_OUTCOME_CANDIDATES_EXHAUSTED = 4` | 삽입 전 후보 탐색 소진 및 배치되지 않은 상태(resolved unplaced)가 확정됨; `material_placed=false`, ROS Action Terminal은 `SUCCEEDED` |
| Terminal `material_placed` | confirmed release와 필요한 scene update가 끝난 배치 사실. terminal outcome과 독립적이며 Mission 회계에서 Goal identity당 최대 한 번 소비 |
| Action terminal status | 기존 ROS Action 성공·취소·abort lifecycle. 명시적 Cancel은 기존 Action 취소 경계를 유지 |
| 기존 `result_code`, `object_disposition`, candidate provenance, diagnostic | legacy 호환/추적용 상세. Mission 분기 입력이 아니며 내부 상태를 파싱하거나 결과를 재승인하지 않음 |

`pnp_outcome`과 `material_placed`만 새 상위 의미로 정의한다. `pnp_outcome`은 세부 Motion·Gripper
원인을 감추고 상위 retry 가능성·실패·Safety 선점을 전달한다. 상세 원인은 기존 내부 Result와
구조화 진단에 남긴다. Orchestration은 상세 진단을 retry predicate로 사용하지 않는다. 기존
Result code/disposition 필드는 호환 adapter와 전환 중 추적을 위해 보존할 수 있으나 새 Mission
정책은 이를 읽지 않는다. 따라서 `GRASP_FAILURE + UNKNOWN` 같은 내부 조합이 공개 Mission
판단을 만들지 않는다.

`material_placed=true`는 confirmed release 후 즉시 확정되어 이후 Retreat 실패·Cancel·Safety
선점에도 되돌리지 않는다. Mission은 실패 outcome과 무관하게 이 사실을 Goal UUID당 정확히 한
번 반영한다. `COMPLETED`는 배치 완료를 뜻하고 반드시 `material_placed=true`와 함께 온다. `RETRYABLE`은
`material_placed=false`일 때만 가능하며 Task Execution이 내부 복구를 종료하고 새 Vision
Acquisition이 필요한 결과다. Orchestration은 Mission 예산·Safety·Cancel 정책을 확인해 재시도
또는 종료를 결정한다. `FAILED`와 `SAFETY_PREEMPTED`는 배치 사실이 이미 확정된 경우
`material_placed=true`를 유지할 수 있다. 기존 재시도 가능한 실제 Gripper 실패는 이
상위 결과에 포함될 수 있으므로 세부 Gripper 상태가 Mission 종료 원인이 되지 않는다.

`CANDIDATES_EXHAUSTED` 이후 복구 조건과 흐름은 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다. Mission은 Motion 세부 Result Code가 아니라 PnP Outcome을 해석한다.

## 4. Lifecycle 및 Acquisition 순서

| Lifecycle 상태·사건 | Owner와 확정 권한 |
|---|---|
| 제출 후 수락 미확정 | Orchestration은 Submission ID와 Mission lifecycle 소유; Task Execution의 명시적 거부 전까지 미제출·비활성으로 간주하지 않음 |
| 수락된 Goal 실행 | Task Execution은 Action lifecycle·terminal 매핑, Motion은 실제 Motion operation 결과·정지 근거, Orchestration은 Mission 대기·취소 의도·최종 상위 결과 소유 |
| 요청자 Cancel | Orchestration은 Mission 취소 의도, Task Execution은 같은 Goal identity의 전파, Motion은 실제 operation 취소·결과 확정 소유 |
| Safety 선점 | Safety는 직접 Stop 원인·mode, Motion은 정지 수행·근거와 operation preemption 결과, Task Execution은 Action 결과 relay, Orchestration은 Mission 종료·기존 Recovery 흐름 소유 |
| Action terminal | Task Execution은 Goal당 한 번 terminal을 확정하고 Motion의 확정 operation 결과를 보존; Orchestration은 이 불변 결과를 Mission에 한 번 반영 |
| 기존 Recovery handoff | 기존 Safety/Motion 정지·복구 근거를 받은 Orchestration이 미해결 Mission 진행을 중단하고 기존 Recovery 책임으로 넘김; 중단 작업 replay 금지 |

- 하나의 Goal은 하나의 PnP 실행과 하나의 terminal result를 만든다.
- 이 lifecycle 의미는 `ExecutePnp`와 `ExecuteDisturb` 모두에 적용된다. Orchestration은 제출 전 부여한 Submission ID를 활성 lifecycle에 등록하고 실제 ROS Goal UUID를 관측하면 같은 Submission에 연결하며, 제출 이후 Task Execution이 명시적으로 거부하거나 Action이 terminal을 확정하거나 기존 Safety/Recovery 계약에 따라 안전하게 인계될 때까지 그 Goal의 유일한 요청자이자 활성 작업의 lifecycle owner로 남는다.
- Submission ID는 ExecuteCycle ID와 해당 Cycle에서 재사용하지 않는 Action 순번을 결합한 Orchestration 내부 식별자다. ROS Goal UUID와 별개이며 Acquisition ID에 의존하지 않는다. 실제 Goal UUID가 관측되지 않았다면 미관측으로 남기고 서버 Goal 상태를 추론하지 않는다.
- Goal 제출 호출의 완료와 서버의 Goal 수락 응답은 별개다. 수락 응답 지연·누락·연결 단절은 거부, 실행 미개시, 취소 또는 Backend 비활성의 증거가 아니다. 수락 여부가 미확정이면 Goal은 잠재적으로 활성 상태로 남고 같은 Mission 작업을 재제출해서는 안 된다.
- Mission Cancel 또는 Safety 선점이 수락 미확정 중 발생하면 Orchestration은 해당 Goal identity에 대한 취소 의도를 보존한다. handoff 전 후속 수락은 같은 Goal의 취소를 이어가며, 명시적 거부만 해당 미션 요청이 실행되지 않았음을 확정한다. Safety direct Stop은 기존 Safety→Motion 경로에서 독립적으로 진행한다.
- 수락된 Goal의 Action Server 연결 또는 Result 응답이 끊겨도 작업 결과·Motion 비활성은 미확정이다. Orchestration은 새 Goal을 제출하거나 미확정 작업을 완료로 회계하지 않는다. 기존 Safety stop/recovery 경계가 미해결 작업을 안전하게 인계했음이 확정되기 전까지 active lifecycle 책임을 유지한다. 이는 새 정지 API나 복구 Gate를 만들지 않는다.
- Cancel 응답은 취소 요청의 수락 여부만 뜻하며 Action terminal이나 실제 중단 완료를 뜻하지 않는다. 요청자 취소·Safety 선점과 Action terminal 응답은 독립 사건이다. 서버가 확정한 terminal 결과는 Goal identity에 한 번만 귀속하고 불변으로 보존한다.
- 명시적 거부·terminal·안전한 기존 Recovery handoff 뒤 도착한 늦은 수락·Feedback·Terminal은 이미 확정된 Mission 결과나 회계를 되돌리거나 새로운 Mission 작업을 열지 않는다. 늦은 응답이 아직 미해결인 같은 Goal의 상태를 해소하면 그 Goal에만 적용하며, terminal은 한 번만 확정한다.
- 후보 전환, PICK→PLACE, Gripper, 공정 내부 HOME/Tray 경유·복구는 같은 Goal에 속한다.
- 정상 다음 Target Acquisition은 현재 PnP의 마지막 GO_HOME을 포함한 terminal 완료와 Mission 회계·진행 결정을 마친 뒤 새 `DetectTarget` 요청으로 시작한다. Feedback 힌트로 사전 Acquisition하거나 결과를 캐시·재사용하지 않는다.
- `RETRYABLE`을 받은 뒤 Mission이 재시도를 승인하면 이전 후보 목록을 폐기하고 새 Acquisition과 새 Goal identity를 사용한다.
- Mission counter, Delivery quantity 및 Vision state는 Task Execution이 갱신하지 않는다.
- 개별 Action Cancel은 해당 Goal identity와 Submission에만 귀속된다. handoff 전 수락 지연 중 의도는 같은 Goal에 전달하며, 거부·terminal·Safety handoff로 분리된 Goal의 취소를 후속 Action에 적용하지 않는다. Cycle 취소는 별도의 Mission 의도이며 기존 정책을 따른다.
- Action Cancel은 현재 Goal에 한정한다. Safety direct Stop은 기존 Safety–Motion ICD를 따르며 Orchestration/Task Execution이 대신 발행하지 않는다.
- Goal acceptance·feedback·진단 기록은 terminal PnP outcome을 대체하지 않는다.

## 5. 기존 Action 호환 범위

Orchestration 정상 자재 처리 경로는 `/motion/execute_task`의 개별 PICK/PLACE 호출에서
`/task_execution/execute_pnp` 단일 Goal로 전환한다. 새 경계의 authoritative 결과는
`pnp_outcome` 및 `material_placed`다. 기존 `ExecuteTask.Result` 필드는 migration 동안
legacy client/adapter 호환과 진단 상관관계를 위해 유지할 수 있으나 Orchestration의 새 PnP
분기에서는 소비하지 않는다.

기존 Motion Action Server lifecycle은 Task Execution 소유다. migration 기간에는 Task Execution
compatibility adapter가 기존 `ExecuteTask` type 및 `/motion/execute_task` 이름으로 기존
PICK/PLACE 구 caller와 standalone GO_HOME/RETRACT caller를 처리할 수 있다. 이는 별도 Motion
ROS endpoint나 Motion의 Action ownership을 뜻하지 않는다. 정상 Mission은 단일 ExecutePnp만
호출한다. ExecutePnp의 성공 순서는 PICK → PLACE → GO_HOME이며 GO_HOME 결과가 확정된 뒤에만
`COMPLETED` terminal을 반환한다. Mission-level 초기 GO_HOME, 승인된 Retry/Recovery GO_HOME 및
standalone RETRACT는 호환 경계의 명시적 movement 요청이고, PnP 내부 HOME/Tray 복구와 별개다.
정상 PnP의 마지막 GO_HOME을 Mission이 다시 호출하지 않는다.

## 6. Verification Requirements

### VR-ICD-ORCH-TE-DISTURB — 교란 요청과 결과 의미

ExecuteDisturb는 동일 Acquisition identity와 전체 후보를 변경 없이 MotionPort에 전달해야 하며(SHALL), `COMPLETED`는 계획된 동작의 측정 완료만 뜻해야 한다(SHALL). 요청자 Cancel과 Safety 선점은 각각 `CANCELED`와 `SAFETY_PREEMPTED`로 구별하고 정지 원인·근거를 보존해야 한다(SHALL). 실제 물체 이동, `material_placed` 또는 PnP 성공을 결과로 추론해서는 안 된다(SHALL NOT).

### VR-ICD-ORCH-TE-LIFECYCLE — 공통 Action 제출·종료 계약

`ExecutePnp`와 `ExecuteDisturb`에서 Orchestration은 제출한 Submission의 lifecycle 책임을 명시적 거부, 단일 terminal 확정 또는 기존 Safety/Recovery 계약에 따른 안전한 인계까지 유지해야 한다(SHALL). 수락 미확정·연결 상실을 거부, 미실행, 완료, 취소 또는 Backend 비활성으로 해석해서는 안 되며(SHALL NOT), 그 Goal이 미해결인 동안 같은 Mission 작업을 새 Goal로 중복 제출해서는 안 된다(SHALL NOT). 늦은 응답은 이미 확정된 Mission 결과를 변경해서는 안 된다(SHALL NOT).


### VR-ICD-ORCH-TE-01 — Goal identity와 입력 경계

한 Acquisition의 전체 후보와 Recipe snapshot은 한 ExecutePnp Goal로 전달되고, source candidate pose와 destination pose는 독립 필드여야 한다.

### VR-ICD-ORCH-TE-02 — 상위 결과 추상화

Mission 분기 입력은 `pnp_outcome`, `material_placed`, Action terminal status여야 한다. 기존 Motion result code, disposition 세부값, Feedback, Holding 또는 diagnostic detail로 결과를 재판정해서는 안 된다.

### VR-ICD-ORCH-TE-03 — 배치 사실의 단조성

확정된 release 이후 `material_placed=true`는 Retreat 실패·Cancel·Safety 선점에서도 보존되고 Mission이 해당 Goal에 대해 한 번만 회계 반영해야 한다.

### VR-ICD-ORCH-TE-04 — 재관측 경계와 다음 Acquisition 순서

Task Execution의 내부 복구는 같은 Goal에 남는다. 다음 Target의 Acquisition과 승인된 새 Acquisition 재시도는 현재 ExecutePnp terminal 결과 이후 새 요청으로 시작한다. `PREFETCH_NEXT`는 Acquisition을 조기 시작하거나 별도 HOME Gate를 추가하는 근거가 되지 않는다.

### VR-ICD-ORCH-TE-CANCEL-IDENTITY — 취소 귀속

개별 Cancel은 해당 Submission의 Goal identity에만 전달해야 하며(SHALL), 늦은 수락 이후에도 다른 Goal에 전달해서는 안 된다(SHALL NOT). Cancel 수락, Action terminal과 Backend 정지 완료는 각각 독립된 사실이다. 검증 근거는 각 Goal의 제출·수락·취소·terminal 및 정지 근거다.

## 참조

- [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)
- [Task Execution FDS](../components/task-execution/fds-pnp-execution.md)
- [Vision–Orchestration ICD](icd-vision-orchestration.md)
- [Task Execution–Motion ICD](icd-task-execution-motion.md)
- [ExecuteCycle ICD](icd-orchestration-execute-cycle.md)
