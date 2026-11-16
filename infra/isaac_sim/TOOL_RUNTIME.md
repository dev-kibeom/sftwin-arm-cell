# Tool 구동·파지 구현

기본 Tool은 Robotiq 2F-85이며 `Logical Actuator + Fixed Attach`를 유지한다.
Physics 모드는 명시적 실험 선택이다. 물리 접촉·자재 유지를 라이브 검증한 구현으로 간주하지 않는다.

## 선택과 책임

Tool Profile의 `runtime`은 모델 구현, `actuator`, `grasp`, Capability와 제어 설정을 소유한다.
Bringup은 선택을 Runtime Configuration에 고정하고 지원 조합을 검사한다. 실행 중 전환·실패 시 자동 fallback은 없다.

| Mode ID | Actuator + Grasp | Qualification | 유지와 HELD | Release |
|---|---|---|---|---|
| `logical_snap` | logical + fixed_attach | 기존 Capture Qualification | Fixed Joint Attach 완료 뒤 HELD | OPEN·Detach 완료 뒤 RELEASED |
| `physical_snap` | physics + fixed_attach | 같은 참조 대상의 양손가락 반력·예상 폭·위치 안정성 | Qualification 뒤 Fixed Joint Attach 완료를 확인하여 HELD | Fixed Joint 제거 및 실제 OPEN 완료 뒤 RELEASED |
| `physical_real` | physics + physical | Tool Object Detection·예상 폭 | Fixed Joint 없이 접촉·마찰로 유지; 지속 정지 관측 시 HELD 추정 | 실제 OPEN 완료 뒤 RELEASED |

Mode ID는 기존 `actuator`/`grasp` 조합에서 파생한 표시 identity다. 독립된 모드 선택 설정은 추가하지 않는다.
`runtime.launcher_modes`의 id/label은 같은 조합을 명명하며 export의 `runtime.mode_id`는 조립 시 파생된다.

`logical + physical`과 알 수 없는 구현은 설정 오류다. Physics Actuator만 선택했다고 Physical Grasp가 되는 것은 아니다.

`gripper_runtime/`은 Cell 밖 `scripts/`의 재사용 Tool 구현이다. `tool_plugin.py`의 기존 선택 경계가
Actuator와 파지 구현을 조립한다. `GraspManager`는 기존 Fixed Attach를, `PhysicalGraspManager`는
선택적 접촉 진단과 Object Detection 기반 보유 추정을 소유한다. Physical Attach은 Tool Capture 전략만 교체하고 기존 GraspManager를 재사용한다. 새 Tool은 이 선택 경계에 구현과 Tool Profile을 등록한다.
공통 Session은 주입된 Tool의 command/update/관측·파지 계약을 소비한다. 장치 전용 Cell·Robot·Motion·Orchestration 분기는 추가하지 않는다.
현재 실제 지원 Tool은 2F-85뿐이다.

Cell은 Scene·설치·Material Supply·Camera와 기존 Action Graph 통합을 소유한다. Robot은 Arm Drive와 HOME을,
Model Assembly의 URDF/USD는 링크·관절·한계·mimic·collision·질량/관성을 소유한다.
공통 Isaac 실행은 선택된 Tool의 pre-Play 설정 hook과 기존 after-Play bootstrap을 호출한다.
기존 Script Editor·Launcher 진입점과 `gripper_runtime` Python import 이름은 유지한다.

## Logical / Physics Actuator

Logical은 기존 `Robotiq2F85Actuator._apply_position()`을 보존한다. 매 Physics tick에 여섯 관절의
위치·속도를 직접 쓰고 imported target을 정렬한다. 접촉 저항으로 구동 결과가 결정되는 모델은 아니다.

Physics는 Isaac Sim 5.1에 포함된 Dynamic Control의 `get_dof_properties`, `set_dof_properties`,
`set_dof_position_target`, `set_dof_velocity_target`, `get_dof_state`를 사용한다.
이 API는 deprecated이지만 5.1에서 제공된다. 주 관절에 force drive의 Position Target을 주고
속도·최대 토크를 제한한다. 정상 명령이나 tick에서 DOF 상태를 쓰지 않는다. tick은 실제 Position/Velocity를 읽는다.
CLOSE가 막혀도 제한된 closing target을 유지해 물리적 하중을 가한다. STOP은 측정 위치를 새 target으로 hold한다.
Snap에서 사용하는 Full OPEN 구동·Release의 기존 0.1 mm 개방 폭 조건은 유지한다.
Physics의 RELEASED는 아래 독립적인 Tool 목표 접근·위치 안정 관측으로 판정한다.
Timeline Stop은 유효하지 않을 수 있는 PhysX handle에 쓰지 않고 로컬 명령/관측·접촉 버퍼를 비운다.
재시작은 새 runtime handle과 새 관측 sequence를 사용한다.

초기 열린 자세는 imported model의 초기 좌표와 pre-Play target 0을 사용한다. Bootstrap의 OPEN은 정상 Drive 명령이다.
Physics용 mimic/Drive/material은 전용 anonymous session sublayer에만 작성하며 원본 USD를 저장·변경하지 않는다.
명시적 Scene 재조립 시 이전 Tool overlay를 교체한다. 모드를 바꿀 때 Timeline Stop 후 Before Play/After Play를 다시 실행한다.
실행 도중 exported JSON을 바꿔 자동 전환하는 경로는 없다.

## 2F-85의 실물과 현재 모델

제조사 [기계 사양](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_Instruction_Manual_Gen_HTML_20190524.zip/2F-85_2F-140_Instruction_Manual_Gen_HTML/Content/6.%20Specifications.htm)은
85 mm stroke, 20–150 mm/s finger speed, 20–235 N grasp force를 제시한다. 이는 지정 fingertip과 조건에서의 사양이며
아래 시뮬레이션의 토크/속도 보정값을 입증하지 않는다.

현재 upstream 기반 URDF는 주 left knuckle 0–0.8 rad, 부호에 따라 ±0.8 rad 범위의 다섯 종속 관절,
fixed outer finger와 회전 fingertip/inner knuckle로 구성된다. Physics 조립은 이 URDF의 mimic 관계를 export하고
PhysX mimic 제약(`follower + gearing * reference + offset = 0`)으로 복원한다. 다섯 follower의 독립 Drive는 끈다.
여섯 독립 모터보다 기존 모델의 평행 파지 연동에 적합하고, 접촉 저항이 주 Drive에 전달된다.

이것은 실물의 완전한 underactuated linkage, 비대칭 적응, encompassing grasp 또는 내부 전동기/기어 모델이 아니다.
별도 폐루프 링크와 tendon 모델은 현재 자산이 제공하지 않는 기구학/동역학 보정을 더 요구하므로 추가하지 않았다.
URDF의 0.5 rad/s·50 N·m limit은 모델 자산 값이며 제조사 모터 사양이라고 주장하지 않는다.
Physics의 속도는 이 URDF 상한에 대한 비율로 제한해 보수적으로 사용한다. 제조사 finger speed 범위를 정확히 재현하지 않는다.
`width_mm = 85 * (1-q)`는 기존 호환용 선형 폭 추정이며 실제 접촉점 사이 거리나 정밀 폭 센서가 아니다.

[제조사 제어 문서](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_Instruction_Manual_Gen_HTML_20190524.zip/2F-85_2F-140_Instruction_Manual_Gen_HTML/Content/4.%20Control.htm)의
position/speed/force request와 actual position, motor current, object status는 서로 다르다.
제조사 [Force register 설명](https://s3.amazonaws.com/com-robotiq-website-prod-assets/website-assets/support_documents/document/2F-85_2F-140_TM_InstructionManual_20240402.pdf)에서 rFR은 모터에 공급하는 최대 전류를 제한하고, 전류 한계 초과 시 정지·object detection을 유발한다. 실제 접촉력을 측정하는 정밀 Force Sensor 값이 아니다.
`gOBJ`의 moving / stopped opening / stopped closing / requested position은 보유 증거와 구분해야 한다.
현재 시뮬레이션은 position error·지속적인 위치 안정성으로 유사한 obstruction 상태를 추정하고 원시 `gOBJ`나 모터 전류를 합성하지 않는다.

## 설정 소유권과 단위

| 정보 | Owner / 근거 |
|---|---|
| 링크·관절 연결·mimic·위치/속도/effort 한계·collision mesh·질량/관성 | 기존 URDF/USD. Bringup이 URDF에서 `joint_model`을 파생해 export; 별도 수작업 관절 목록 없음 |
| 85 mm stroke | Tool Profile / 제조사 사양, 기존 호환 추상화 |
| force/speed request와 가용 Capability | Tool Profile / 제어 설정 |
| Drive stiffness/damping·토크 예산·정지 판정 허용오차/dwell | Tool Profile `runtime.physics` / 미보정 simulation tuning |
| fingertip 물리 material·마찰·반발 | Tool Profile `runtime.physics.contact_material` / 실험 보정 필요 |

Profile의 0.1–1.0 N·m 토크 예산과 1.0/0.8 마찰은 보정되지 않은 초기 시뮬레이션 값이다.
제조사 force request를 이 토크 범위에 선형 대응하는 것은 **시뮬레이션 제어 추상화**이며 실물 rFR→토크/N 보정 곡선이 아니다.
회전 joint의 Drive `maxForce`는 N·m 토크 한계이고 실제 손가락 접촉력(N)과 동일하지 않다.
DC gain은 radian 기준, USD angular gain/velocity는 degree 기준으로 변환한다. Arm 관절은 Tool 설정 대상에서 제외한다.
기존 mesh collision은 유지하며 URDF의 Gazebo/ODE surface 태그가 PhysX material을 정의한다고 가정하지 않는다.
Physics 모드에서 Tool 접촉 링크에만 명시적인 material과 contact reporting을 적용한다.

## 명령과 관측

기존 GRASP / RELEASE / STOP 및 width 호환 경로를 유지한다. ROS Action·Motion·Orchestration은 변경하지 않는다.
Physics Tool의 `/tool/intent` JSON에는 선택적 `parameters`를 사용할 수 있다.

```json
{"intent":"GRASP","execution_id":"<applied-motion-reference-id>","parameters":{"target_position":1.0,"speed_fraction":0.5,"grip_force_n":100.0}}
```

target_position과 speed_fraction은 normalized [0,1]이며 speed는 0보다 커야 한다. grip_force_n은 양쪽 손가락 법선력 합계(N)이고 Tool 한계 안이어야 한다. 파라미터 없는 GRASP/RELEASE는 현재 speed/force 설정으로 CLOSE/OPEN한다.
STOP에는 추가 파라미터를 받지 않는다. Logical 구현은 grip_force_n을 검증·보존하되 물리 힘을 실현하지 않는다. 다른 지원하지 않는 제어 파라미터는 거부한다.
추가 필드를 모든 Tool에 필수화하지 않는다. `ToolRuntime.observation()`은 제공하는 구현에서만 position/velocity,
target, motion status, inferred object status와 설정을 반환한다.

### Physical Attach Qualification

Physics Actuator를 사용하되 Qualification과 Attachment를 분리한다. 기존 Capture Reference의
object observation을 이용한 유일 후보 association을 재사용하며, 그 대상과 **동일한 Rigid Body**에
대한 좌우 fingertip 접촉만 인정한다. 기존 Logical Capture Qualification을 중복 적용하지 않는다.

- 유효한 CLOSE의 execution identity와 현재 Capture Reference가 일치해야 한다.
- 실제 관절 위치가 양쪽 joint limit에서 떨어지고 closing target에 미도달해야 한다.
- 실제 관절에서 계산한 폭이 선택 Reference의 기대 폭과 `width_tolerance_mm` 이내로 일치해야 한다.
- 같은 상대 body에 대한 좌우 fingertip의 normal impulse를 각각 Physics Step dt로 나눈 반력이
  `runtime.physics.holding.min_contact_force_per_finger_n` 이상이어야 한다.
- 위 조건과 `held_position_tolerance_rad` 이내 위치 안정성이 `stopped_dwell_s` 동안 지속돼야 한다.

Qualification 자체는 HELD가 아니다. 기존 GraspManager/Fixed Joint Owner가 Attach 생성과 소유권을
확인한 이후에만 HELD다. Attach 실패는 HELD를 발행하지 않는다. 운반 중 유지는 Fixed Attach가 담당한다.
최소 반력 0.1 N/finger는 미보정 초기값이며 제조사 force 사양이나 실제 유지력 증거가 아니다.

### Physics Object Detection

Actuator가 실제 관절 위치 변화 범위와 목표 오차로 `in_motion`, `stopped_closing`,
`stopped_opening`, `at_requested_position`을 계산한다. 순간 속도는 진단이며 정지 승인 조건이 아니다.
위치가 안정 허용 범위를 벗어나면 정지 시간을 초기화한다. Detection은 Actuator에서 한 번 계산하며
PhysicalGraspManager는 그 관측과 기대 폭을 소비한다. 기존 `blocked_dwell_s`와 Holding의
`stopped_dwell_s`는 같은 안정 관측 구간에 적용하며 직렬 대기 시간을 추가하지 않는다.

CLOSE target 미도달·관절 한계 배제·지속 `stopped_closing`·예상 폭 일치 시 HELD를 추정한다.
Moving/Stopped Opening/At Requested Position은 HELD 근거가 아니다. HELD 이후 정지·폭 근거가
사라지면 UNKNOWN이며, 낙하를 임의로 RELEASED로 분류하지 않는다. OPEN 진행 중 UNKNOWN,
적용된 full OPEN 목표 접근과 연속 위치 안정 관측 후 RELEASED다.
`physics.release`의 초기 설정은 `open_tolerance_mm=0.5`,
`position_stability_tolerance_rad=0.001`, `stable_duration_s=0.20`이다. 이는 미보정 초기 시뮬레이션
설정이며 제조사 실물 정밀도나 자재 배치 성공을 보증하지 않는다. 실제 관절의 목표 오차를 개방 폭
등가값으로 환산하여 접근을 확인하고, 접근 조건 안에서만 `PositionStability`를 누적한다.
목표 이탈·새 명령·STOP·취소 시 초기화하며, 순간 관절 속도는 안정 판정의 필수 Gate가 아니다.
이미 열린 위치에서 OPEN을 다시 요청해도 새 안정 관측을 기다린다. Task Execution은 별도 모드
우회 없이 fresh `RELEASED`를 소비하며 시간 소진은 성공이 아닌 실패다. Fixed Joint 부재는 HELD 추정을 무효화하지 않는다.
자재 World Pose나 좌우 반력 센서를 Real의 공통 보유 계약에 요구하지 않는다. 실제 Hardware Backend는
제조사 Object Detection을 관측으로 제공할 수 있으며 simulator 전용 추정 알고리즘을 요구받지 않는다.

### 접촉 관측과 생명주기

두 Physics 모드는 기존 Tool 링크에 Contact Report를 활성화한다. actor 경로는 Rigid Body link,
collider 경로는 그 하위 collision prim이다. 로컬 USD/PhysX API와 링크 구성은 자동 검사 대상이다.
좌우 힘은 벡터 합산하지 않고 body pair별로 계산한다. Snap은 선택된 Robot의 정확한 fingertip body와
참조 대상만 사용한다. 여러 Step의 미소비 batch, 유효하지 않은 dt, 누락·무효·이전 접촉은 성공 근거로 재사용하지 않는다.
Real에서는 접촉·반력은 선택적 진단이며 Object Detection 기록과 구분된다. 진단 장애는 Real Held를 차단하지 않는다.
STOP은 pending 검출을 취소하고, Snap의 이미 소유한 Attachment는 기존 owner가 유지한다.
Timeline Stop/Reset/Shutdown은 각 구현의 pending 검출·구동·소유 Attachment와 Holding을 정리한다.
Motion/Orchestration의 fresh Held 계약, Timeout, Mission Gate, Retry 정책은 변경하지 않는다.

현재 폭은 실제 관절에서 계산한 선형 추정이므로 실제 fingertip geometry 보정도 라이브 확인이 필요하다.

HELD 이후에도 같은 closing target을 유지하면서 실제 위치·속도·폭을 매 tick 재평가한다.
움직임·추가 닫힘·폭/목표 조건 상실은 즉시 UNKNOWN으로 되돌린다. 이후 적합한 정지가 다시 지속되면 재추정할 수 있다.
STOP, OPEN 시작, 참조 교체·취소·진정한 무효화, Reset/Shutdown은 누적 근거를 폐기한다.
OPEN 중에는 UNKNOWN, 실제 full OPEN 완료에만 RELEASED를 보고한다. 미끄러짐 추정만으로 RELEASED를 발행하지 않는다.

Motion의 파지 승인 후에는 pending Capture Reference가 퇴역한다. 이 경우만 `invalidate:true, retire_pending:true`로 표시하며
수락된 CLOSE의 실행 ID·예상 폭 snapshot은 해당 파지의 지속 관측 근거로 유지한다. 이는 자재 Pose snapshot이 아니다.
퇴역은 새 CLOSE를 허가하지 않는다. 새 명령에는 새 유효 참조가 필요하다. 취소/실패의 일반 `invalidate:true`는 유지 근거도 폐기한다.
재전송된 퇴역이 HELD를 만들거나 소실된 근거를 복구하지 않는다.

ROS `grasp`는 Holding Observation, `attached`는 실제 시뮬레이션 Attachment 소유 여부다.
따라서 Physical HELD는 `grasp=1, attached=0`이며 기존 신선도·Sequence·Timestamp 및 PICK의 fresh HELD 요구조건을 유지한다.
이 관측이 유효하면 기존 PnP가 진행할 수 있다. Motion·Orchestration 알고리즘에 예외나 새로운 Gate를 추가하지 않는다.
공통 Session은 CLOSE 거부·Attachment 관측의 공개 메서드를 사용하며, Fixed Attach의 post-attach Pose 진단과 종료는 해당 구현이 소유한다.
Physical 구현에는 자재 Pose 진단이나 의미 없는 Fixed Attach 메서드를 요구하지 않는다.

Isaac 5.1 DC `get_dof_state().effort`는 gravity를 포함하는 joint effort이며 순수 motor 저항/접촉 토크 센서가 아니다.
회전 joint의 해당 값은 N·m 진단으로만 노출하고 HELD 임계값으로 사용하지 않는다. `get_dof_effort()`의 applied command나
`max_effort`/Drive `maxForce`도 실제 관측 저항으로 치환하지 않는다. 현재 보유 추정은 위치·속도·지속 정지와 폭에 근거한다.
모터 전류나 정밀 Force Sensor 값을 합성하지 않는다.

Physics contact report의 actor pair·접촉점·법선·impulse와 실제 관절 상태는 선택적 진단이다.
`physical_grasp` 기록은 기존 bounded gripper diagnostic writer에 10 Hz로 제출한다. impulse(N·s)는 force(N)가 아니다.
Contact LOST/관측 부재는 보유 성공이 아니며, contact buffer는 tick마다 교체한다. 접촉 진단은 HELD 판정 입력이 아니다. 지속 보유/미끄러짐 검증은 아직 완료하지 않았다.

## 실행

GUI 런처는 선택한 Tool에 연동된 단일 `Grasp Mode`를 표시한다.
병렬 그리퍼의 `Logical Attach`은 Logical + Fixed Attach(기본), `Physical Attach`은 Physics + Fixed Attach,
`Physics`는 Physics + Physical로 접촉·마찰 기반 시뮬레이션을 의미한다.
Tool Profile의 `runtime.launcher_modes`가 표시 이름·설명을 제공하며, 실제 지원 범위는 기존 `supported_combinations`가 소유한다.
표시용 메타데이터가 미지원 조합을 추가하지 않는다. 내부 Actuator·Grasp 분리는 유지하고 실행 중에는 선택을 잠근다. 선택값은 Bringup의 공통 조립기가 검증하고,
Isaac과 ROS에 같은 Runtime Configuration을 전달한다. 원본 Tool/Deployment YAML을 수정하거나 복사하지 않는다.

GUI의 기본 설정과 접을 수 있는 고급 설정에서 선택 모드에 적용되는 값을 입력한다. 변경값은 굵은 색상으로 표시하며 기본값 복원으로 되돌린다. 모드·Tool 변경 시 입력값은 해당 Profile 기본값으로 초기화한다. 입력은 실행별로만 적용하며 저장하지 않는다. Bringup이 허용된 수치 항목과 범위, 마찰계수 관계를 검증한 뒤 동일한 실행 snapshot에 포함한다. 실행 중에는 편집과 복원을 잠근다. 미보정 물리 튜닝값은 실제 장치 보정값이나 파지 성공의 증거가 아니다.

CLI/Script Editor에서는 기존 export CLI에 선택을 전달한다. RB5는 Profile을 `rb5_850_profile.yaml`로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=ros2_ws/src/bringup python3 ros2_ws/src/bringup/scripts/export_runtime_configuration \
  --profile ros2_ws/src/bringup/config/operational_profile.yaml \
  --tool-actuator physics --tool-grasp physical \
  --output /tmp/arm-cell-physics-runtime.json
bash infra/isaac_sim/scripts/isaac_arm_cell.sh --runtime-config /tmp/arm-cell-physics-runtime.json
```

Script Editor 직접 실행은 위 파일을 `ARM_CELL_RUNTIME_CONFIG`로 지정하고 기존 Before Play/After Play 순서를 따른다.
이미 export한 설정은 실행 중 수정하지 않는다. 선택을 바꿀 때 원본 Profile에서 다시 export하고 실행을 재시작한다.
원본 generated USD를 다시 저장하지 않는다. Physics + Fixed Attach는 `--tool-grasp fixed_attach`를 지정한다.

## 검증 범위와 사용자 라이브 확인

자동 검증은 Logical 동작·Fixed Attach/Release 회귀, Physics target/read/STOP, unsupported 조합,
URDF 연동 파생·configuration export, 두 Robot의 HOME·Profile 보존을 확인한다. 물리 보유 추정·Session lifecycle·참조 퇴역과 ROS Holding/Attachment 분리는 Mock 및 transport 자동 테스트로 확인한다.
설치된 Isaac Sim 5.1 API/schema 검토와 USD 조립 검사는 물리 엔진 실행을 대신하지 않는다.
테스트 결과와 기존 기준선 실패는 PR 검증 항목에 기록한다.

사용자 라이브 확인 항목:

- 두 Robot의 기존 Logical PnP 성공과 HOME·Arm Drive 보존.
- Physics에서 85 mm 실제 개방 geometry·joint limit·속도, mimic 오차와 접촉 정지·OPEN·STOP·재시작.
- 빈 CLOSE와 물체/환경 걸림의 진단 구분; 고착을 보유로 보고하지 않는지.
- 실제 payload 질량·형상별 마찰, 토크, 손가락 접촉·유지·미끄러짐/낙하. 현 tuning은 성공 보장이 없다.
- Physical 모드에서 Fixed Joint 생성·자재 순간이동·충돌 무시가 없는지, 유효한 추정 HELD/근거 소실 시 UNKNOWN, HELD 이후 참조 퇴역과 후퇴·PLACE 진행.
- Physics + Fixed Attach의 실제 접촉 정지 이후 Capture Qualification 가능 여부는 별도 확인.

실제 Isaac Physics tick을 실행한 파지/접촉/하중 유지 검증은 이번 자동 검증의 완료 주장에 포함하지 않는다.

## Holding 대기 시간 예산

운영 Motion 설정의 기존 `holding_confirmation_timeout_ms`는 **4000 ms wall clock**이다.
CLOSE 이후 손가락 이동과 Physics Step dt로 누적하는 정지 관측이 모두 이 예산 안에 들어야 한다.
시뮬레이션이 실제 시간보다 느리면 동일한 물리 이동·dwell에도 더 긴 wall time이 필요하다.
이는 추가 dwell이 아니며 fresh HELD/RELEASED가 확인되면 즉시 진행한다. 같은 예산은 세 모드와
OPEN 후 RELEASED 확인에도 적용되므로 실패 시 최대 대기도 늘어난다. Validation Profile의 별도
예산은 변경하지 않는다. Object Detection·폭·반력·신선도 조건은 완화하지 않는다.
4초는 관측된 실행을 위한 초기 운영 조정값이며 임의의 simulation slowdown을 보장하지 않는다.
실제 라이브 테스트로 CLOSE→HELD와 OPEN→RELEASED, 보유·운반을 확인해야 한다.

## 물리 파지 하중 진단

초기 Force 설정은 0.8로 주 관절 drive 토크 상한은 0.82 N·m다. 이는 손가락 파지력(N)이 아니다.
Tool 접촉 정지/동마찰은 1.0/0.8로 조정했으며 미보정 simulation 값이다. Material Supply의 contact_material도 1.0/0.8로 명시하며 각 생성 자재의 Collider에 Physics Material을 바인딩한다.
자재의 마찰 결합은 average다. 양쪽 설정이 일치할 때 유효 계수도 1.0/0.8이며 실물 재질 보정값이 아니다.
Real 접촉 진단은 Physics dt와 robot/body별 좌우 반력을 남기며 한쪽 하중도 0으로 숨기지 않는다.
기존 양손가락 Qualification과 별개이며 Real Held 조건을 강화하거나 완화하지 않는다.
Actuator 진단의 max_drive_effort_nm은 구동 상한이고 joint_effort_nm은 중력 등을 포함한 관측값이다.
실제 유지력·마찰·관통/진동은 라이브 검증이 필요하다.
