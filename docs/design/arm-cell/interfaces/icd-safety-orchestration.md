# [SF-Twin] ARM Cell Safety ↔ Orchestration 인터페이스 계약

- **범위:** `Safety Supervisor ↔ Orchestration`
- **소유자:** `ARM Cell Shared Interface`

## 1. 엔드포인트와 권한

| 엔드포인트 | ROS 타입 | 생산자 | 소비자 |
|---|---|---|---|
| `/safety/state` | `arm_cell_interfaces/msg/SafetyState` | Safety | Orchestration, Motion |

`MotionCapability`, `selected_stop_mode`, Safety 원인·래치, 정지 시작, 복구·Reset 승인에 대한 유일한 권한은 Safety에 있다. Orchestration은 운영 준비 상태, 작업 승인, 작업 순서 및 Mission 진행을 소유한다. Orchestration은 Safety 권한을 소비하며 AMR/PackML 상태, 통신 진단, 준비 상태, Mission 상태로부터 Safety 원인이나 정지 심각도를 추론·생성·중복·상향해서는 안 된다(SHALL NOT). 실행과 정지 완료 증거는 Motion이 소유한다.

## 2. SafetyState 전송 계약

실행 가능한 인터페이스는 [`SafetyState.msg`](../../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/SafetyState.msg)로 구현되며, 공유 도메인은 [`MotionCapability.msg`](../../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/MotionCapability.msg)와 [`SafetyCauseSet.msg`](../../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/SafetyCauseSet.msg)이다.

| 필드 | 타입 | 의미 |
|---|---|---|
| `header` | `std_msgs/Header` | 활성 ROS Clock에서 Safety 평가·게시 시각 |
| `safety_state` | `uint8` | `SAFETY_STATE_UNKNOWN`, `SAFETY_STATE_SAFE`, `SAFETY_STATE_INTERLOCKED`, `SAFETY_STATE_STOPPING`, `SAFETY_STATE_STOPPED`, `SAFETY_STATE_RECOVERY_REQUIRED`, `SAFETY_STATE_EMERGENCY_STOPPED` 중 하나 |
| `motion_capability` | `MotionCapability` | Safety가 소유하는 동작 권한 도메인 |
| `selected_stop_mode` | `StopMode` | Safety가 선택한 현재 정식 정지 심각도. 더 높은 심각도의 정지가 없을 때 `STOP_MODE_CONTROLLED`가 가장 낮은 기준값 |
| `active_causes` | `SafetyCauseSet` | 현재 존재하는 원인 |
| `latched_causes` | `SafetyCauseSet` | Safety Reset 정책으로 해제하기 전까지 유지되는 원인 |
| `valid` | `bool` | 현재 필수 Safety 입력으로 유효한 감시 상태를 구성했는지 여부 |
| `required_inputs_fresh` | `bool` | 시작 시 Safety 감시에 필수인 Safety-HW/Motion 입력이 Safety Watchdog 정책상 최신인지 여부 |
| `diagnostic_detail` | `string` | 선택적·비규범적 진단 텍스트 |

`SafetyCauseSet.value`는 비트 집합이다. 정의된 원인은 `E_STOP`, `STO`, `COMMUNICATION_LOSS`, `COLLISION`, `PREMATURE_UNDOCK`, `PACKML_ABORT`, `GRIPPER_FAILURE`, `OTHER`, `REQUIRED_INPUT_INVALID`다. 이 비트는 Safety가 소유한 조건을 전달하며 전송 품질 또는 작업 준비 상태 API가 아니다. `COMMUNICATION_LOSS`, `REQUIRED_INPUT_INVALID`, `PREMATURE_UNDOCK`, `PACKML_ABORT`는 전송 호환성을 위해 공개 원인 집합에 남지만 현재 데모의 Safety는 AMR/PackML 상태·타임스탬프·통신 이상을 근거로 이 원인을 발생시키지 않는다. AMR/PackML은 Safety 필수 입력이 아니다. 작업별 정보 충분성은 Orchestration이 판단한다. 실제 필수 Safety-HW/Motion 증거 손실 및 확인된 Motion 위험은 해당 Safety 계약에 따라 Fail-closed로 처리하고, 확인된 위험은 기존 Latch/Reset 동작을 유지한다. `OTHER`는 열거되지 않은 원인에 대해서도 Fail-closed를 유지하며 진단 텍스트에는 기계 판정 권한이 없다.

메시지 타임스탬프가 Safety의 로컬 수신 시간 기반 Watchdog 평가를 대체하지 않는다. Motion과 Orchestration은 SafetyState에 각자의 수신 경과시간 계약을 적용한다. SafetyState 누락·오래됨, `valid=false`, 필수 Safety-HW/Motion 감시 손실은 계속 Fail-closed로 처리한다. AMR/PackML 준비 상태 및 통신 건전성은 Orchestration이 기존 외부 상태 Topic으로 별도 평가하며, 오래된 SafetyState를 이 입력으로 대체할 수 없다.

기존 Orchestration 소비자는 승인·진행·복구 경계에서 동일한 읽기 전용 Safety 관측 계약을 따른다. Safety가 게시한 `/safety/state` 메시지를 소비하고 설정된 동일한 로컬 Monotonic 수신 경과시간 유효성 규칙을 적용한다. 이는 수신된 메시지의 수락 조건을 설명할 뿐 집계 Snapshot, Service, Database, 조합 계층 또는 별도 상태 저장소를 추가하지 않는다. 누락·무효·기한 초과 메시지는 권한 없음으로 판단해 해당 의존 작업만 차단하거나 일시 중단한다. 수신 유효성 판단은 원인을 재해석하거나 위험·정지 심각도를 도출하거나 Safety 소유 상태·래치를 만들거나 중복 정지를 요청하지 않는다. 유일한 생산자와 권한 소유자는 계속 Safety다.

`selected_stop_mode`는 Safety만 기록한다. 공유 `StopMode` 도메인을 사용하며 `valid=true`, `required_inputs_fresh=true`일 때만 유효하다. 소비자는 순서(`CONTROLLED < IMMEDIATE < EMERGENCY`)를 비교하지만 `active_causes`에서 다시 계산하지 않는다. `CONTROLLED`는 공유 도메인의 최저값이자 더 높은 정지 심각도가 없을 때의 기준값이며 숨겨진 진단 인코딩이 아니다.

## 3. Orchestration 동작

| 권한 | Orchestration에 허용되는 동작 |
|---|---|
| `MOTION_NONE` | 정상·복구 궤적 요청 모두 금지 |
| `MOTION_RECOVERY_ONLY` | 복구 조건이 충족된 경우에만 `RETRACT` 조율 가능 |
| `MOTION_NORMAL` | Safety가 정상 동작 허용. Orchestration은 승인·진행 전에 현재 작업 준비 상태도 확인해야 함 |

Mission 중 권한을 잃으면 조율을 중단하거나 취소한다. 이러한 조율은 Safety의 직접 StopMotion 경로보다 후순위다. 작업 시작 전 준비 상태 손실은 승인을 보류한다. Mission 도중 준비 상태 손실은 Orchestration FDS에 따라 진행을 중단하지만 새 Safety 심각도를 도출하지 않는다. Orchestration은 `RETRACT` 요청 전에 복구를 명시적으로 허용하는 Safety 상태를 기다리며, 자신의 Mission이나 준비 상태만으로 승인 권한을 추론할 수 없다.

## 4. 검증 요구사항

### VR-ICD-SAFE-ORCH-01 — 동작 권한 소비
Orchestration은 Safety 동작 권한을 소비해야 하며(SHALL), 재정의해서는 안 된다.

### VR-ICD-SAFE-ORCH-02 — 정상 동작 Gate
정상 Mission 동작에는 `MOTION_NORMAL`이 필요하다(SHALL).

### VR-ICD-SAFE-ORCH-03 — 복구 Gate
복구 요청에는 현재 유효하고 복구를 허용하는 Safety 권한이 필요하다(SHALL).

### VR-ICD-SAFE-ORCH-04 — 선택된 정지 심각도의 권한
Safety는 정식 선택 정지 심각도를 `SafetyState.selected_stop_mode`로 게시해야 한다(SHALL). Orchestration은 `active_causes`, Mission 상태 또는 로컬 정책으로 심각도를 도출하지 않고 해당 필드를 소비·비교해야 한다(SHALL).

### VR-ICD-SAFE-ORCH-05 — 준비 상태와 Safety 권한의 구분
현재 작업에 필요한 AMR/PackML 데이터가 누락·무효·오래된 경우 Orchestration은 Safety 원인을 생성하거나 정지 심각도를 변경하지 않고 의존 작업만 보류해야 한다(SHALL). 무관한 외부 상태나 타임스탬프 품질을 일괄 승인 Gate로 사용해서는 안 된다(SHALL NOT). 현재 시뮬레이션 데모에서 AMR/PackML 상태만으로 Safety 정지·Latch·Reset을 유발해서는 안 된다(SHALL NOT). Safety 권한을 사용할 수 없거나 Safety 선점이 발생하면 Orchestration은 Mission 진행을 보류·중지해야 하며(SHALL), 직접 정지·심각도·원인 생명주기·복구/Reset은 Safety에 맡긴다. 판정 기준은 준비 상태·승인, SafetyState, StopMotion 발행, Mission 진행이다.
