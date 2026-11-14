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
| GRASP / RELEASE (`GripperPort.grasp()/release()`, 기존 `close()/open()` 호환) | Task Execution 의도 / Tool Plugin 실현 | Motion·Backend |
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

Tool 작업 의도 GRASP는 자재 확보 요청, RELEASE는 자재 해제 요청이다. 병렬 그리퍼는 CLOSE/OPEN으로, 흡착기 유형은 VACUUM_ON/OFF로 실현할 수 있다. 의도 수락·Tool 구동 완료·Qualification 통과·Attach/Detach 완료는 별개의 사실이다. 기존 close/open API와 폭 기반 ROS 명령은 병렬 그리퍼 호환 경로로 유지한다.

Isaac의 추가 `/tool/intent`는 `std_msgs/String`의 JSON `intent=GRASP|RELEASE|STOP`, 선택적 `execution_id`를 받는다. 제어 Capability가 있는 Tool은 선택적 `parameters` 객체를 해석할 수 있으며 지원하지 않는 항목·값은 거부한다. 현재 Physics 2F-85는 normalized target_position, speed_fraction, force_setting을 제공한다. 이 정보는 다른 Tool의 공통 필수 필드가 아니며 접촉력 센서값을 뜻하지 않는다. GRASP의 execution identity는 적용된 Capture Reference와 일치해야 하며, retained graph 값은 Session 시작 시 generation 기준선으로만 소비한다. STOP은 Tool별 구동 취소·Hold를 요청하고 새 Capture를 취소하며 자체로 Detach하지 않는다. 같은 Physics 경계에서 의도를 수락하면 이미 대기하던 호환 명령은 다시 실행하지 않는다. 기존 `/gripper/command`, `/gripper/close`, `/gripper/capture_reference`, `/gripper/status`의 병렬 그리퍼 의미는 보존한다. 폭 Capability가 없는 Tool의 status는 `width_mm`를 생략하며 Holding의 시각·Sequence·신선도 규칙은 동일하다. 폭 Capability는 선택된 Tool Profile에서 제공한다.

Fixed Attach에서 Tool별 Qualification은 해당 접촉·확보 조건을 제공하고 기존 Capture 소유자가 Attach를 완료한 뒤에만 HELD를 발행한다. RELEASE도 Tool별 해제 완료와 소유 Attachment 제거가 모두 확인되어야 RELEASED다. 명시적으로 선택한 Physical Grasp는 Fixed Joint를 생성하지 않으며 물리적 접촉·마찰에 의존한다. 선택된 유효 Capture Reference의 예상 폭과 실제 관절 위치·속도·닫힘 목표·지속 정지 조건을 함께 사용해 HELD를 추정한다. 추정 이후에도 실제 관측을 계속 평가하며 근거가 소실되면 UNKNOWN을 전달한다. 정지 또는 접촉 하나만으로 보유를 확정하지 않으며 자재 Ground Truth를 뜻하지 않는다. ROS status의 grasp는 보유 관측이고 attached는 실제 Attachment 소유 여부이므로 Physical HELD는 attached=0과 양립한다. Physical OPEN 완료는 실제 관절 관측으로 확인한다. 현재 지원 Profile은 Robotiq뿐이며 Fake Vacuum은 테스트에서만 주입한다. 다른 Tool의 실제 센서·물리·Capture Reference Geometry 구현은 이 계약만으로 지원 완료가 되지 않는다.

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

Capture Reference는 CLOSE 선택 근거다. 취소·실패의 `invalidate:true`는 해당 실행의 물리 보유 추정 근거도 무효화한다. 파지 승인 후 pending 참조의 정상 퇴역은 `invalidate:true, retire_pending:true`로 구별한다. 이 퇴역만 수락된 CLOSE의 실행 ID·예상 폭 근거를 유지하며 새 CLOSE를 허가하지 않는다. 유지된 근거도 실제 관절 관측과 계속 대조하며 STOP/OPEN/새 선택·Reset/Shutdown에서 폐기한다. 이는 자재 World Pose 추적이나 Attachment를 대신 생성하는 계약이 아니다.
