# [SF-Twin] ARM Cell 운영자 확인 및 Safety Reset 계약

- **인터페이스 / 경계 ID:** `SAFETY-OPERATOR-RESET`
- **소유자:** ARM Cell 공유 인터페이스
- **참여자:** UI Hub ↔ Safety
- **관련 설계:** [Safety 정지·복구 FDS](../components/safety/fds-stop-recovery.md), [Orchestration 제어 복구 FDS](../components/orchestration/fds-controlled-recovery.md)

## 1. 목적

이 ICD는 Final Demo에 필요한 운영자 확인 및 Reset 요청의 최소 경계를 정의한다. Hub는 요청을 제출하고, Reset 정책의 평가와 적용은 Safety만 수행한다.

## 2. 서비스 상호작용

서비스는 `/safety/reset`이며 타입은 `arm_cell_interfaces/srv/ResetSafety`이다.

| 방향 | 필드 | 타입 | 의미 |
|---|---|---|---|
| 요청 | `request_id` | `unique_identifier_msgs/UUID` | 운영자 요청의 상관 식별자 |
| 요청 | `operator_acknowledged` | `bool` | 이번 Reset 시도에 대한 명시적 운영자 확인 |
| 응답 | `applied` | `bool` | Safety가 Reset 정책을 적용했는지 여부. `false`는 거부를 의미 |
| 응답 | `diagnostic_detail` | `string` | 비규범적 설명 |

관측 가능한 결과의 정식 기준은 SafetyState다. `applied=true`만으로 정상 Motion이 허용됐다고 판단할 수 없다.

## 3. Reset 선행 조건

Safety는 정지를 유발한 물리적·외부 원인이 해소되고, E-stop/STO가 비활성 상태이며, 필수 입력이 유효하고 최신 상태이고, 이전 제어 실행과 Action 소유권이 종료됐으며, Reset 정책이 래치된 원인의 해제를 허용하고, 운영자의 명시적 확인이 있을 때만 Reset을 적용한다. 물리적 관절 안정 상태는 Reset 선행 조건이 아니다.

제어 실행이 비활성인 상태에서 Motion이 `STOPPING`을 보고하면, `applied=true`는 해제 가능한 래치를 해제하지만 Motion이 물리적 정지를 확인하기 전까지 SafetyState는 Motion 권한을 계속 거부한다. 이전 명령 생산자가 활성 상태이거나 Action 소유권이 해소되지 않았거나 Stop/Hold 게시에 실패했거나 E-Stop/STO가 활성 상태라면 Reset을 거부한다. 외부 어댑터나 Hub가 정식 상태를 수정해 Safety 래치를 해제할 수 없다. 거부된 요청은 래치와 권한을 변경하지 않는다.

## 4. 검증 요구사항

### VR-ICD-SAFE-OP-01 — 명시적 확인
운영자의 명시적 확인 및 Reset 요청이 없다면, E-stop 조건이 해소됐다는 사실만으로 Safety 래치를 해제하거나 정상 동작 권한을 복원해서는 안 된다(SHALL NOT).

### VR-ICD-SAFE-OP-02 — Safety Reset 권한
Hub 요청은 SafetyState를 직접 변경해서는 안 된다(SHALL NOT). Reset의 적용 또는 거부는 Safety만 결정할 수 있으며, 결과의 판정 기준은 Safety가 게시하는 정식 SafetyState다.

## 5. 참조

- [Safety↔Orchestration ICD](icd-safety-orchestration.md)
- [Safety↔Motion ICD](icd-safety-motion.md)
- [UI Hub FDS](../components/ui-hub/fds.md)
