# PLACE 경로 재사용 실험과 PLANNED 전략 유지의 판단

*Engineering Story — 2026-10-30 설계자 회고 및 관련 PR 이력을 연결한 역사적 기록. 현재 전략 계약이나 실행 성능 Benchmark가 아니다.*

## 문제: 반복 PLACE Global Planning의 시간과 실패

초기 PnP 라이브 테스트에서 PICK 후 PLACE가 시작될 때 경로 계산이 오래 걸리거나 계획 실패가 발생했다. 개발자가 PLACE WAYPOINT 전략을 검토한 직접적인 이유는 **고정된 PLACE 목적지에 매번 Global Planning을 수행하는 비용과 실패 가능성을 줄이는 것**이었다(2026-10-30 회고). 핵심 동기는 반드시 HOME을 경유하는 동작 자체가 아니었다. 목표는 반복되는 PLACE 작업을 더 빠르고 안정적으로 수행하는 것이었다.

## 실험 및 변경 이력

1. **전략 분리와 대안 탐색 — PR #717.** PICK/PLACE에 독립적인 `PLANNED`·`WAYPOINT` 전략을 두고, 고정된 경유 자세나 planner를 설계 단계에서 불변조건으로 확정하지 않으려 했다. 시퀀스 구현은 실험으로 비교할 수 있도록 남겼다.
2. **고정 배치 영역 및 경로 재사용 구체화 — PR #718, PR #719.** `PLANNED`는 기존 개별 자재 목표와 Grasp 관계를 유지하고, `WAYPOINT`는 고정 배치 영역에 대한 접근·Release 경로를 분리했다. 초기 #719 시점에는 선택 설정만 추가됐고 WAYPOINT의 운영 구현은 미지원이었다.
3. **사전 계산 경로 실험 — PR #720.** PICK Retreat 후 필요하면 Y clearance를 거쳐 GO_HOME을 수행하고 저장된 HOME→PLACE Trajectory를 재사용하도록 구현했다. Global Planning을 매 사이클 반복하지 않기 위한 방법이었다. 사전 경로는 실험 당시의 로봇 모델과 프로파일에 맞춰 생성·검증하도록 했다.
4. **라이브에서 드러난 경유 경로의 새로운 문제 — PR #722, PR #723.** Clearance 목표의 6D IK가 실패해 위치 목표 기반 Planning으로 바꾼 사례와, 설치 좌표계의 Y 방향을 잘못 적용해 로봇 반대 방향으로 clearance하던 사례가 남았다. 이는 고정 경로를 추가하면 원래 계획 비용 대신 경계 좌표·clearance·경로 유효성이라는 별도 검증 문제가 발생함을 보여준다.

## 현재 판단: PLANNED가 충분히 개선되어 WAYPOINT는 미완

설계자는 실험 이후 **PICK Retreat 길이를 늘리자 기존 PLACE `PLANNED` 전략의 경로 계산이 스스로 개선된 것으로 관찰했다**고 회고했다. 그 결과 WAYPOINT 개발의 우선순위가 낮아졌으며 **WAYPOINT는 미완 상태로 남겼다.** 따라서 저장된 Trajectory 기반 접근이 실기 라이브에서 우월하다고 결론내리거나, WAYPOINT를 완료·채택된 최종 최적화로 소개해서는 안 된다.

현재 [Task Execution PnP FDS](../../design/components/task-execution/fds-pnp-execution.md)는 PICK과 PLACE 전략의 기본값을 모두 `PLANNED`로 명시하고 `WAYPOINT` 시퀀스를 별도 전략 계약으로 보존하고 있다. **설계서에 전략 경로가 정의돼 있다는 사실과 그 전략이 완성·검증돼 기본 운영으로 채택되었다는 사실은 다르다.** 이 기록의 '미완'은 개발자 회고에 근거하며, 현재 브랜치의 모든 프로파일·실제 실행 경로를 이번 작업에서 통합 시험한 결과는 아니다.

## 설계에서 얻은 교훈

- 느린 Global Planning을 해결하기 위해 무조건 사전 경로 캐싱을 도입할 필요는 없다. **Planner에 진입하는 시작 상태와 접근 거리**가 바뀌면 기존 전략의 성능도 달라질 수 있다.
- '새 전략을 구현할 수 있다'와 '새 전략을 채택할 가치가 있다'는 다른 판단이다. 후자는 라이브 실행에서 목표 문제의 실제 개선 여부를 확인해야 한다.
- 경유 전략은 계획 비용을 절약할 수 있는 대신 좌표 방향, clearance 및 사전 경로의 유효성이라는 새로운 제약과 실패 경로를 만든다.
- 전략을 독립적으로 선택 가능한 구조로 남기되, 미완성 전략을 완성된 성과로 포장하지 않는다. 확장 가능성을 확보한 상태에서 검증된 기본 전략을 유지할 수 있다.

## 실험 관찰과 정량 비교의 구분

위 PR은 설계·구현·집중 테스트 및 일부 실패 진단을 제공하지만, 이번 기록에는 동일 장면·프로파일에서 후퇴 거리 변경 전후 PLACE Planning 시간을 비교하는 통제된 표본이 없다. 따라서 'PLANNED가 개선됐다'는 설계자 라이브 관찰이고, 정확한 속도 개선율·실패율 감소·WAYPOINT 대비 우월성을 입증한 정량 결과가 아니다.

## 저장된 PLACE Planning 관측과 비교 조건

2026-10-22 세션 `1aef9756-64ac-4e1d-a056-1f623e581def`(revision `184336740fc4efa25262da5ddb990e3f5073570a`, clean; Profile SHA-256 `3e2813f00828bdb0a3272ad30a2b4ef6e5560c1792e79c7585475017aa5abf13`)에는 PLACE `place_global_approach` Planning 11건이 모두 성공했다. 기록된 `elapsed_planning_s`는 0.028119–0.062653 s, 중앙값 0.039583 s(n=11)였다. 이는 각 Planning 호출의 시간이며 전체 PLACE Action 또는 Mission 시간은 아니다. 이 revision은 #717–#723 전략 변경 이전이므로 후퇴 거리 연장 효과를 측정하지 않는다.

### 기존 PLANNED 성공 표본의 Planning 시간 (n=11)

| 호출 수 | 결과 | 최소 | 중앙값 | 최대 |
|---:|---|---:|---:|---:|
| 11 | 모두 성공 | 0.028119 s | 0.039583 s | 0.062653 s |

### 이후 PATH_OBSTRUCTED 기록 (별도 축, 조건이 서로 다름)

| 세션 | Planning 시도 | 결과 | Planning 시간 |
|---|---:|---|---:|
| `f66506eb…` (dirty) | 2 | `PATH_OBSTRUCTED` | 30.003793 s |
| `f875a936…` (clean) | 2 | `PATH_OBSTRUCTED` | 30.003775 s |

두 표본군은 같은 축에서 직접 비교하지 않는다. 앞 표본은 PR #717–#723 이전의 Planning 성공 11건이고, 아래 실패는 이후 서로 다른 세션·Profile에서 기록됐다. 후퇴 거리의 실제 설정값도 세션과 연결되지 않아 Retreat 연장 효과를 보여주지 않는다.

이후 기록에는 서로 다른 조건의 실패도 있다. 2026-10-29 세션 `f66506eb-aef1-480c-9448-87727b87f55d`(revision `4ca8528ff3cf9a161729ff1d4e972c7f8b3fe77d`, dirty; Profile SHA-256 `51106ace264a98497fba575b131abb9be9851cd2dda734c7e496bd696366a22b`)의 PLACE global approach는 2회 planning 시도 후 `PATH_OBSTRUCTED`, 30.003793 s였다. 같은 날 별도 세션 `f875a936-cc50-41d0-86e6-e7132a6473eb`(revision `45049b14d3ba4ab6857bbf003de8737ab0d446e4`, clean; Profile SHA-256 `e26a2601867152a0681aeca0f8ebeb54f9f9006a77b189339ad7c2041ecc5120`)도 2회 시도 후 `PATH_OBSTRUCTED`, 30.003775 s였다. 두 실패 기록은 planning 단계의 시간 초과/경로 실패 관측이며 Action 전체시간으로 해석하지 않는다. 후퇴 거리의 실제 값을 이 진단들과 연결할 근거가 없고 장면·revision·profile도 동일하지 않으므로, 앞선 11건과 합산하거나 전략 전후 비교로 취급하지 않는다.

원본은 `private local diagnostic artifact (not included in the public package)`, `20261029/174602/`, `20261029/184156/`의 session 및 Motion 기록이다. 따라서 회고의 “Retreat 연장 뒤 PLANNED 개선”은 정량적으로 검증된 효과가 아니라 설계자 관찰로 남는다.

### 관련 근거

- PR #717, PR #718, PR #719, PR #720, PR #722, PR #723
- [Task Execution PnP FDS](../../design/components/task-execution/fds-pnp-execution.md)
- 설계자 회고(2026-10-30): PLACE Planning 지연이 직접 동기였고 Retreat 연장 뒤 PLANNED 개선을 관찰하여 WAYPOINT를 미완으로 남겼음.
