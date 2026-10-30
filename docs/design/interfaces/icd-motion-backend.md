# [SF-Twin] ARM Cell Motion–Backend 공유 계약

- **계약 Owner:** ARM Cell Shared Interface
- **소비자:** Motion Core, 교체 가능한 Robot/Gripper Backend

## 1. 목적과 공유 계약 경계

Motion Core와 Isaac·실물 Robot Backend가 동일하게 유지해야 하는 **Backend 비종속 실행·보유 상태 계약**이다. Task 결정, TCP 보정, Safety 권한 집행, 정규 Motion Result는 Motion의 책임이다. Robot/Gripper Adapter는 장치별 구현을 소유한다.

## 2. 참여자와 인터페이스

| 논리적 계약 | Producer / Authority | Consumer |
|---|---|---|
| 실행 요청 및 실행 수명주기 | Motion 요청 / Backend 실행 결과 | Motion |
| 중단·Hold·Stop 전달 및 종료 근거 | Motion 요청 / Backend 증거 | Motion·Safety 상태 판단 |
| `GripperPort.close()/open()` | Motion 논리 명령 / Adapter 실현 | Motion |
| `HoldingObservation` | Backend 관측 | Motion |
| Backend 가용성·Capability·오류 | Backend | Motion |

Backend는 Controller·Prim·서비스명·Joint 토폴로지와 무관하게 동일한 논리 의미를 제공한다. 이 계약은 특정 ROS Transport나 Adapter 클래스 구조를 강제하지 않는다.

## 3. 공유 의미 계약

| 데이터·결과 | 보장 및 유효성 | 금지된 해석 |
|---|---|---|
| Trajectory 명령 수락 | Backend가 제공하는 명령 전달 경계의 성공; 실패·미완료 및 최종 수락을 구별 | 실제 관절 목표 일치·물리적 안정화 또는 제공되지 않은 장치 ACK로 해석 금지 |
| PICK/PLACE Trajectory 완료 | [Motion Execution FDS](../components/motion/fds-motion-execution.md)의 개별 실행 완료 결과 | 실행 종료·정지 근거·Gripper/객체 부착·분리나 공정 성공을 이 결과 하나에서 추론 금지 |
| 실행 종료 | 이전 Trajectory 명령 생산자가 더 이상 발행하지 않고 Action 소유권이 해제됨 | 단순 Stop 수락이나 Hold 발행으로 종료 확정 금지 |
| Stop/Hold 전달 결과 | 명령 전달이 완료됐는지 별도 보고 | 명령 전달과 물리적 완전 정지를 동일시 금지 |
| Backend 가용성 | 연결·기능·실패 상황을 구분해 보고 | 지원되지 않는 Gripper/Holding을 성공으로 대체 금지 |
| `close()` / `open()` | 장치별 구동을 요청하는 논리 명령 | 명령 성공을 `HELD`/`RELEASED`와 동일시 금지 |
| `HoldingObservation.state` | `HELD`, `RELEASED`, `UNKNOWN` | 이전 관측·장치 내부 Capture 성공을 현재 보유 증거로 승격 금지 |
| `HoldingObservation` 메타데이터 | 단조 관측 시각과 Sequence를 포함한 관측 신선도 근거 | 결측·오래됨·모호함·Adapter 소실을 확정 상태로 취급 금지 |

관측은 Validation 프로파일에서 **500ms**의 유효기간을 갖는다. 결측·오래된 관측·Adapter 연결 소실·모호한 후보·예기치 않은 Joint/Prim 소실은 `UNKNOWN`이다. PICK 후퇴·성공에는 새로운 `HELD`, PLACE 후퇴·성공에는 `open()` 성공과 새로운 `RELEASED`가 필요하다.

Motion이 제공하는 선택된 TCP Grasp Pose와 Geometry에서 도출한 예상 접촉 폭은 Simulator Capture가 같은 계획된 파지점을 평가하도록 하는 논리 입력이다. 예상 접촉 폭은 `close()`의 구동 목표가 아니다. Simulator별 Capture Volume/위치·방향·폭 허용 조건과 단계 실행은 [Motion Execution FDS](../components/motion/fds-motion-execution.md) 및 [Task Execution PnP FDS](../components/task-execution/fds-pnp-execution.md)가 소유한다. Backend는 이 기준을 무시할 수 있는 실물 Holding 센서를 사용할 수 있으며 최종 보유 근거는 유효한 `HoldingObservation`이다.

## 4. 상호운용 조건

- Stop 완료의 공유 의미는 [Safety–Motion ICD](icd-safety-motion.md)와 일치하며 소프트웨어 종료와 하드웨어 Safety 인증을 혼동하지 않는다.
- Hold 지원 같은 Backend별 선택 기능은 Capability로 보고하고, 지원하지 않는 요청을 성공처럼 완료하지 않는다.
- 불확실한 Holding 시나리오는 **정상 `GripperPort.holding()` 관측 경계**에서 fresh `UNKNOWN`을 반환하거나 관측을 불가능하게 하는 방식으로 재현할 수 있다. Safety/MotionStatus/Orchestration 상태를 직접 변조하지 않는다. 시나리오 해제만으로 `HELD`·`RELEASED`를 만들어서는 안 된다.
- Simulator Prim Path, Vendor Joint 구성, 원시 Sensor·Contact, 특화 API는 상위 논리 계약으로 노출하지 않는다.

## 5. 검증 요구사항

### VR-ICD-MOT-BACKEND-01 — Backend 독립성
Backend 교체 후에도 논리적 Gripper 의미, Stop 전달과 명령 생산 종료의 구분을 유지해야 한다. 소프트웨어 실행 종료는 하드웨어 안전 정지를 대체하지 않는다.

### VR-ICD-MOT-BACKEND-02 — 파지 확인 경계
`holding()`은 Boolean이 아닌 관측 시간·Sequence를 가진 `HoldingObservation`을 반환해야 한다. `HELD`·`RELEASED`·`UNKNOWN`은 명령 결과와 구분된다. PICK은 새로운 `HELD` 없이 후퇴하거나 성공해서는 안 된다. Isaac Production/Validation에서 Capture는 Motion이 선택한 TCP 파지 Pose·Closing Axis를 사용하며 World Z 수직 접근을 강제하지 않는다. Profile별 Capture 허용 조건은 유지한다.

### VR-ICD-MOT-BACKEND-03 — 선택 기능 미지원
요청된 Gripper·Holding 기능이 지원되지 않으면 정규 실패를 반환하고 PICK/PLACE 성공을 보고해서는 안 된다.

### VR-ICD-MOT-BACKEND-04 — Release 확인 경계
`open()` 성공 및 새로운 `RELEASED` 관측 없이 PLACE 후퇴나 성공을 보고해서는 안 된다. `UNKNOWN`은 해제를 뜻하지 않으며 장치별 Detach는 Adapter가 소유한다.

### VR-ICD-MOT-BACKEND-05 — 신선도와 가용성
결측·오래된 관측, Adapter 소실, 모호한 후보, 예상치 못한 Joint/Prim 소실은 `UNKNOWN`으로 전달하고 PICK/PLACE 진행을 허용하지 않는다. Validation의 500ms 신선도 기준은 Vision TTL과 독립적이다.

## 6. 계약 의존성과 소비 관계

- [Motion CDS](../components/motion/cds.md): Adapter·Port의 정적 책임
- [Motion Execution FDS](../components/motion/fds-motion-execution.md) 및 [Task Execution PnP FDS](../components/task-execution/fds-pnp-execution.md): PICK·PLACE 동작과 관측 사용
- [Motion Stop FDS](../components/motion/fds-stop-recovery.md): 중단 처리 순서
- [Safety–Motion ICD](icd-safety-motion.md): 외부 정지 완료 및 Safety 권한 의미
