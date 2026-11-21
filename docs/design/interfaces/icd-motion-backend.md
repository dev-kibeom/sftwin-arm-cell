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

관측의 시간상 유효성과 보유 상태 확정은 구분한다. 최근 수신한 `UNKNOWN`은 유효한 불확실 관측이며 `HELD` 또는 `RELEASED` 확인을 충족하지 않는다.
관측은 Validation 프로파일에서 **500ms**의 유효기간을 갖는다. 결측·오래된 관측·Adapter 연결 소실·모호한 후보·예기치 않은 Joint/Prim 소실은 `UNKNOWN`이다. PICK 후퇴·성공에는 새로운 `HELD`, PLACE 후퇴·성공에는 `open()` 성공과 새로운 `RELEASED`가 필요하다.

Motion이 제공하는 선택된 TCP Grasp Pose와 Geometry에서 도출한 예상 접촉 폭은 Simulator Capture가 같은 계획된 파지점을 평가하도록 하는 논리 입력이다. 예상 접촉 폭은 `close()`의 구동 목표가 아니다. Simulator별 Capture Volume/위치·방향·폭 허용 조건과 단계 실행은 [Motion Execution FDS](../components/motion/fds-motion-execution.md) 및 [Task Execution PnP FDS](../components/task-execution/fds-pnp-execution.md)가 소유한다. Backend는 이 기준을 무시할 수 있는 실물 Holding 센서를 사용할 수 있으며 최종 보유 근거는 유효한 `HoldingObservation`이다.

Tool 작업 의도 GRASP는 자재 확보 요청, RELEASE는 자재 해제 요청이다. 병렬 그리퍼는 CLOSE/OPEN으로, 흡착기 유형은 VACUUM_ON/OFF로 실현할 수 있다. 의도 수락·Tool 구동 완료·Qualification 통과·Attach/Detach 완료는 별개의 사실이다. 기존 close/open API와 폭 기반 ROS 명령은 병렬 그리퍼 호환 경로로 유지한다.

Conveyor 흡착기는 GRASP에 진공을 인가하고 RELEASE에 해제한다. 접촉할 자재가 없어도 명령은 같은 방식으로 구동되며, 보유 여부는 명령 이후의 장치 피드백으로 보고한다. 시뮬레이션도 이 장치 경계를 따른다. 공급 예약 조회나 장면 객체 ID 일치를 구동 조건으로 사용하지 않는다. 물리 모델은 접촉에 따른 결합을 모사하고, 실제 접촉 상대의 ID는 독립 검증에만 사용할 수 있다.

Isaac의 추가 `/tool/intent`는 `std_msgs/String`의 JSON `intent=GRASP|RELEASE|STOP`, 선택적 `execution_id`를 받는다. 제어 Capability가 있는 Tool은 선택적 `parameters` 객체를 해석할 수 있으며 지원하지 않는 항목·값은 거부한다. 현재 Physics 2F-85는 normalized target_position, speed_fraction과 합계 파지력(N)인 grip_force_n을 제공한다. 이 정보는 다른 Tool의 공통 필수 필드가 아니며 접촉력 센서값을 뜻하지 않는다. Capture Reference를 사용하는 Tool에서 GRASP의 execution identity는 적용된 Reference와 일치해야 하며, retained graph 값은 Session 시작 시 generation 기준선으로만 소비한다. STOP은 Tool별 구동 취소·Hold를 요청하고 새 Capture를 취소하며 자체로 Detach하지 않는다. 같은 Physics 경계에서 의도를 수락하면 이미 대기하던 호환 명령은 다시 실행하지 않는다. 기존 `/gripper/command`, `/gripper/close`, `/gripper/capture_reference`, `/gripper/status`의 병렬 그리퍼 의미는 보존한다. 폭 Capability가 없는 Tool의 status는 `width_mm`를 생략하며 Holding의 시각·Sequence·신선도 규칙은 동일하다. 폭 Capability는 선택된 Tool Profile에서 제공한다.

모드별 Qualification·보유·Attachment 의미는 아래 표를 따른다. ROS status의 grasp는 보유 관측이고
attached는 실제 Fixed Joint 소유 여부다. 현재 지원 Profile은 Robotiq 2F-85와 M0609 Tape Cell의 C10/Long이며 Fake Vacuum은 테스트에서만
주입한다. 다른 Tool의 실제 센서·물리·Capture Reference Geometry 구현은 이 계약만으로 지원 완료가 되지 않는다.

### Backend 비종속 파지력 요청

질량 기반 파지 정책을 사용하는 GRASP는 공통 정책이 Tool·자재 한계를 검사한 목표 파지력을 뉴턴(N)으로 전달한다.
목표와 정책의 허용 최대값은 서로 마주 보는 두 손가락의 법선 압착력 합계이며, 손가락당 힘·관절 토크·제조사 정규화 명령과 혼용하지 않는다.
Robotiq 2F-85의 제조사 파지력 범위 20–235 N은 손가락당 기준으로 정규화하여 합계 40–470 N으로 다룬다.
근거는 [제조사 매뉴얼](https://s3.amazonaws.com/com-robotiq-website-prod-assets/website-assets/support_documents/document/2F-85_2F-140_TM-OMRON_20240402.pdf)의 Table 5-1과 §5.2.1의 양쪽 손가락 하중식이다. 해당 사양의 패드·결합부 적용 조건을 보존한다.
해당 두 손가락 힘 정책은 물리 구동이면서 병렬 파지와 뉴턴 단위 파지력 Capability를 제공하는 Tool에만 적용한다. 다른 Tool의 물리 구동만으로 이 정책을 선택하지 않는다.
공통 정책은 자재 명목 질량·접촉 마찰 하한·작업 가속도 한계·여유계수에서 목표를 결정한다.
Tool은 제조사 사양과 적용 조건을, 자재는 손상 방지를 위한 허용 최대 압착력을 소유한다.
목표는 두 한계 모두를 만족해야 하며, 여유를 포함한 필요 힘을 제공할 수 없으면 더 작은 값으로 잘라 성공으로 진행하지 않는다.

장치별 명령 변환은 Backend/Tool Adapter의 책임이다. Isaac은 선택 Tool의 기구학과 mimic 전달 관계를 반영한 물리 모델로
관절 구동값을 계산하고, 실물 Adapter는 제조사 힘 명령 의미를 적용한다. 시뮬레이션 토크 보정값을 실물 명령으로 재사용하지 않는다.
명령 가능한 최소 힘을 적용하는 경우에도 자재 허용 최대 힘을 초과할 수 없다.
필요 Capability·변환 모델·유효 입력이 없거나 장치 한계로 목표를 실현할 수 없으면 명시적으로 실패하며, 기존 무제한/기본 힘으로 대체하지 않는다.

목표 파지력과 구동 토크 제한은 관측된 접촉력 또는 실제 무손상을 증명하지 않는다.
동적 접촉 피크·닫힘 충격·패드와 접촉 면적·모델 오차에 대한 검증은 Backend별로 수행한다.
파지력 명령의 성공은 기존과 같이 HELD나 공정 성공을 대신하지 않는다.

Logical Snap의 일반 PICK은 목표 파지력을 요구하거나 질량 기반 파지력 진입 검사를 적용하지 않고 기존 논리적 파지 경로를 사용한다.
직접 제공된 선택적 힘 입력의 검증·보존은 물리 힘 실현 Capability나 PICK 진입 조건을 뜻하지 않는다.
Physics Snap과 Physics Real은 동일한 힘 정책을 적용한다. 실제 힘 실현은 Physics Actuator 또는 해당 Capability를 제공하는 Hardware Adapter의 책임이다.

### Tool 파지 모드와 Holding / Attachment

각 Tool은 지원하는 Actuator/Grasp 조합마다 파지 판정과 유지 방식에 맞는 공통 모드 하나를 지정한다. 신규 Tool과 파지 구현에도 다음 세 표시명을 사용한다.

| 파지 모드 | 판정과 유지 방식 |
|---|---|
| 논리 판정 + 고정 | 위치·형상 등 논리 조건을 확인한 뒤 대상 자재를 고정한다. |
| 물리 판정 + 고정 | 접촉 반력 등 물리 관측으로 파지 조건을 확인한 뒤 대상 자재를 고정한다. |
| 물리 파지 | Tool의 물리 모델로 자재를 유지한다. 평행·내경 그리퍼는 접촉과 마찰을, 흡착컵은 하중 한계가 있는 흡착 결합을 사용한다. |

런처는 선택한 Tool이 지원하는 모드만 표시한다. 표면 흡착·외측 파지·내경 파지 같은 **파지 전략**은 별도로 표시한다.
실행 시작 시 선택한 조합을 고정하며 실행 중 전환·실패 시 Fallback을 하지 않는다.

다음 표는 Robotiq 2F-85의 실행 ID별 관측과 Attachment 의미다.

| Mode | 구동과 Qualification | HELD 의미 | Attachment |
|---|---|---|---|
| `logical_snap` | Logical 구동·기존 Capture Qualification | Fixed Joint Attach 완료 확인 | 있음 |
| `physical_snap` | PhysX 구동·참조와 동일 대상의 좌우 접촉 반력·폭·위치 안정성 | Qualification 뒤 Fixed Joint Attach 완료 확인 | 있음 |
| `physical_real` | PhysX 구동·Tool Object Detection | 목표 미도달·지속 정지·관절 한계 안쪽 위치에 근거한 보유 추정 | 없음 |

Holding Observation(HELD/RELEASED/UNKNOWN)은 Attachment Ownership과 별개다. Fixed Joint가 없다는
이유로 Real의 HELD를 무효화하지 않는다. Snap의 Qualification은 Attach 완료 관측을 대신하지 못한다.
두 Snap은 OPEN/Detach 완료, Real은 실제 OPEN 완료 후 RELEASED다. Real의 CLOSE/OPEN 진행 중과
보유 근거 상실은 UNKNOWN이며 관측만으로 낙하를 확정하여 RELEASED로 바꾸지 않는다.
Real의 Object Detection은 순간 속도 진동만으로 정지를 거부하지 않고 지속적인 관절 위치 안정성과
closing target 미도달을 해석한다. Moving, Stopped Opening, At Requested Position은 HELD 근거가 아니다.
Simulation은 이 상태를 관절 관측으로 추정하고 Hardware는 제조사 Detection을 제공할 수 있다.
보유·해제 판정은 선택 Tool 구현이 소유한다. Physical Real의 `RELEASED`는 적용된 OPEN 목표에
실제 관절이 접근하고 연속 위치 안정 관측이 충족된 상태다. 초기 OPEN 접근 허용치는 Tool 설정의
0.5 mm이며 위치 안정 허용치와 지속 시간도 Tool 설정에서 제공한다. OPEN 명령 수락·경과 시간만으로
`RELEASED`를 만들지 않는다. 새 명령·STOP·취소·목표 이탈은 이전 안정 관측을 초기화한다.
실제 Hardware Adapter는 제조사 위치·동작/물체감지 상태를 해석해 같은 HoldingObservation을 제공할
수 있으며, simulator의 관절 모델이나 허용치를 그대로 재사용할 의무가 없다. 실제 Hardware Adapter
구현과 실물 해제 검증은 이번 범위에 포함하지 않는다.

보유·해제 판정은 선택 Tool 구현이 소유한다. Physical Real의 교정되지 않은 선형 폭 환산은 진단용이며 예상 접촉 폭과의 정밀 비교로 보유를 거부하지 않는다. Object Detection은 선택 자재의 동일성을 증명하지 않는다. Snap의 기하·접촉 Qualification은 별도로 유지한다.
좌우 Fingertip 반력이나 자재 World Pose는 Hardware/Real 공통 관측의 필수 필드가 아니다.

STOP은 pending 구동·검출을 취소하고 소유 Attachment는 기존 owner 규칙을 따른다.
Reset/Shutdown은 각 모드의 구동·Attachment·Holding을 정리한다. 기존 fresh HELD/RELEASED 조건과
상태 인터페이스·Timeout·Retry·Mission Gate를 유지한다. 자동 테스트는 상태 전이와 소유권을 검증하며
실제 마찰 파지·운반 성공의 검증을 대신하지 않는다.


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

Conveyor 흡착기는 공급 서비스가 없는 장치 단독 시험에서도 GRASP와 RELEASE에 따라 진공을 켜고 끌 수 있어야 한다. 빈 픽 위치에서는 진공 명령이 전달되어도 물체를 보유했다고 보고하지 않는다.

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

## Configured radial Tool

C10 Physical Real은 `internal_radial` 요구와 선택 Tool의 명목 기하를 검증한다. 주축 위치의
목표 미도달·관절 한계 안쪽의 지속 정지로 HELD를 추정하며 실제 수축 위치로 RELEASED를 확인한다.
이는 장치 보유 추정이며 예약 자재의 동일성·실제 수령을 증명하지 않는다. 공통 Holding의
신선도·UNKNOWN·STOP·실행 상관 규칙은 유지한다. 별도 접촉력 센서를 가정하지 않는다.

`/gripper/capture_reference`의 radial 참조는 motion_goal_uuid, strategy, material_id, supply_id, slot_id,
supply_epoch, reservation_id와 insertion_depth_m/thickness_m/inner_radius_m을 전달한다.
ACK는 유효 요청과 명목 Tool 기하 적용을 뜻하며 Stage identity의 조회·일치를 요구하지 않는다.
Motion은 같은 실행의 fresh HELD로 파지 단계를 확정하고 fresh RELEASED로 해제 단계를 확정한다.
정확한 자재/TCP GT 변환이나 동적 Planning Scene payload의 반영·복귀를 요구하지 않는다.
Configured radial 티칭은 환경 충돌 검사 없이 명시된 TCP 경로를 실행하며 물리 충돌은 진단에서 평가한다.
