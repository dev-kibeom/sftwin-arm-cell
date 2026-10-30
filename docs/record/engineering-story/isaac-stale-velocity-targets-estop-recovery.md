# Isaac 관절 위치 명령에 남아 있던 속도 목표와 E-Stop 복구

*Engineering Story. 2026-10-30 로컬 보존 라이브 세션과 PR 이력을 대조한 역사 기록이다. 현재 Safety·Motion 실행 계약이나 공식 Acceptance 결과를 대체하지 않는다.*

## 발견한 문제와 원인 분석

E-Stop 뒤 HOLD 및 GO_HOME 위치 명령을 보냈지만 관절이 기대한 방식으로 복구되지 않는 사례가 있었다. 세션 `253b59bf-2326-4726-9826-5a5ad8b3a4b5`의 clean revision `166e3f75522f476bc5892fdf3512488643e2ad90`에서 E-Stop 후 GO_HOME 두 건이 각각 `TASK_RESULT_JOINT_PROGRESS_STALLED`(71.326 s, 70.781 s)로 끝났고, 두 Mission은 처리 0건으로 `MISSION_EXIT_MOTION_ERROR`를 반환했다.

원인 분석은 Isaac articulation controller가 새 position target을 받더라도 명령에서 velocity 배열을 생략하면 이전 velocity target을 유지할 수 있다는 점에 집중했다. 이는 동작 원인에 대한 정적 분석이었으며, 당시 세션 기록에 관절별 position/velocity 목표 시계열이 저장돼 있지는 않았다.

## 수정과 시간순 검증

| 순서 | 기록과 실행 조건 | 확인된 결과 |
|---|---|---|
| 문제 관측 | 2026-10-30 02:59 세션 `253b59bf…`; revision `166e3f7…`, clean | GO_HOME 정체 2건(71.326 s, 70.781 s); Mission 처리 0건(요청 1건 및 2건) |
| 원인 분석·수정 | PR #745, 코드 revision `dd696382372075fc46350c93157e1daf9f320b2a` | 속도를 별도 지정하지 않은 위치 명령, HOME 재전송, Safety Hold에서 관절별 velocity target 0을 명시 |
| 중간 자동 검증 | PR #745 빌드 및 관련 Motion 회귀 테스트 | 빌드 PASS, 관련 테스트 3개 PASS. 전체 패키지 테스트는 554개 중 6개 실패로 unresolved였고, 이 단계에서는 Isaac Live E-Stop 검증을 하지 않음 |
| 수정 후 첫 라이브 실패 | 2026-10-30 07:51 세션 `20319dde-36b2-432a-9125-400865da57a1`; PR head `dd696…`, clean | GO_HOME `JOINT_PROGRESS_STALLED` 73.519 s, Mission 요청 1건 중 처리 0건. 이 사례는 수정이 모든 실행 조건에서 이미 검증됐다고 볼 수 없음을 보여줌 |
| 수정 후 라이브 성공 | 2026-10-30 07:57 세션 `d0fd2a6f-d2d1-47c4-9889-7b8afdaa9cef`; 동일 revision `dd696…`, clean | E-Stop → reset → Recovery GO_HOME 성공(6.003 s); Mission 1/1 처리 후 `MISSION_EXIT_QUANTITY_REACHED` |
| 최신 clean 복구 확인 | 2026-10-30 13:42 세션 `fc425b71-eeab-4c08-8671-fbe0d65510a1`; revision `e0e9bcc805fd190b1ddbfdfb041b99fb98ba648d`, clean | `EMERGENCY_STOPPED` → `RECOVERY_REQUIRED` → reset 적용 → `SAFE`; Orchestration `SafetyRecovery` 성공; Recovery GO_HOME `aaa53db9a017f3e5125474e9d667eba5` 성공(5.643 s). Mission은 Detection을 다시 수행하고 PnP를 재개했으나, 이후 `approach:TASK_RESULT_BACKEND_UNAVAILABLE`로 종료(요청 3건, 처리 0건) |

PR #745는 위치 명령의 `publish_joint_target_locked`에서 velocity를 생략하는 경우 관절 수만큼 0을 채우고, HOME 재전송과 Safety Hold에도 0 velocity를 명시하도록 수정했다. 위 최신 라이브 revision `e0e9bcc…`은 PR #745 merge commit `32b56df9fd0cb8a28b00ea75a70e72b4698f653c`의 후손이며, 해당 수정 코드가 실제 실행 revision에 포함된 것을 확인했다. 수정 위치는 `ros2_ws/src/arm_cell_motion/src/isaac_ros_motion_transport.cpp`다.

## 해결 확인과 별개 Backend 실패의 구분

**저장된 최신 clean 라이브 세션에서 E-Stop Reset 후 SafetyRecovery와 GO_HOME이 성공했고, Mission이 재개되어 후속 PnP를 시도한 것이 확인됐다. 따라서 stale velocity target으로 인한 E-Stop 복구 문제는 이 실행 조건에서 해결된 것으로 기록한다.** 최신 세션의 Mission 전체는 성공하지 않았다. 종료 원인은 후속 `BACKEND_UNAVAILABLE`이며 E-Stop Recovery GO_HOME은 이미 성공한 뒤 발생했다. 앞선 성공 세션에서는 Mission 1/1 완료도 확인됐다.

최신 clean 세션의 Motion 기록에는 실행 Profile fingerprint가 직접 기록되지 않았다. 해당 revision에 포함된 `operational_profile.yaml`의 SHA-256은 `e26a2601867152a0681aeca0f8ebeb54f9f9006a77b189339ad7c2041ecc5120`이다. 후속 2026-10-30 13:49 세션 `a3973cc8-6d40-44d9-a31b-de68c0340122`는 같은 revision 표기지만 worktree가 dirty였고 GO_HOME이 `TASK_RESULT_BACKEND_UNAVAILABLE`로 끝났다. 이 결과는 clean 세션의 해결 확인을 취소하지 않으며, dirty 상태에서 해당 실패를 stale velocity 수정에 귀속할 수도 없다.

이 자료는 통제된 A/B 시험이나 모든 장면·프로파일에 대한 성공률을 제공하지 않는다. 최신 clean 세션이 입증하는 범위는 E-Stop 이후 reset, Recovery, GO_HOME 성공과 Mission의 후속 PnP 재진입이다. 전체 Mission 완료 여부는 별도로 기록하며, 해결 상태를 다른 backend 실패까지 일반화하지 않는다.

## 재사용할 교훈

- ROS 메시지의 필드 생략이 controller 내부 목표의 초기화를 뜻하는지 별도로 확인한다.
- 상태를 유지하는 시뮬레이터 입력에서는 position·velocity·effort 채널의 조합을 하나의 명령 의미로 해석한다.
- 빌드·회귀 테스트, 라이브 Recovery 동작, Mission 전체 완료를 서로 다른 검증 결과로 기록한다.
- 과거 실패는 해결 과정의 이력으로 보존하되, 동일한 실패가 현재 검증 조건에서 재현됐는지를 별도로 판단한다.

## 원본 증거와 관련 변경

- 사전 수정 실패: `private local diagnostic artifact (not included in the public package)`의 `session.json`, Safety·Motion·Orchestration 기록.
- 수정 후 라이브 실패 및 성공: `private local diagnostic artifact (not included in the public package)`, `private local diagnostic artifact (not included in the public package)`.
- 최신 clean 복구 확인: `private local diagnostic artifact (not included in the public package)`, `safety/safety-1793335381074466031.jsonl`, `orchestration/orchestration-1793335381076397366.jsonl`, `motion/motion_planning_records/aaa53db9a017f3e5125474e9d667eba5.json`, 후속 PnP 결과 `motion/motion_planning_records/d8253b2a94205d872f0161bccbedd00c.json`.
- 후속 dirty 세션: `private local diagnostic artifact (not included in the public package)`.
- 구현 및 자동 검증 이력: PR #745.
- E-Stop 이후 HELD 자재 반환은 별도 변경 PR #746이며, 이번 velocity target 수정의 인과 근거와 혼동하지 않는다.
