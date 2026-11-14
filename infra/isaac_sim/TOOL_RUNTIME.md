# Tool 구동·파지 구현

기본 Tool은 Robotiq 2F-85이며 `Logical Actuator + Fixed Attach`를 유지한다.
Physics 모드는 명시적 실험 선택이다. 물리 접촉·자재 유지를 라이브 검증한 구현으로 간주하지 않는다.

## 선택과 책임

Tool Profile의 `runtime`은 모델 구현, `actuator`, `grasp`, Capability와 제어 설정을 소유한다.
Bringup은 선택을 Runtime Configuration에 고정하고 지원 조합을 검사한다. 실행 중 전환·실패 시 자동 fallback은 없다.

| Actuator | Grasp | 동작 |
|---|---|---|
| logical | fixed_attach | 기존 시간 보간·DOF 상태 쓰기, Capture Qualification과 Fixed Joint Attach/Detach |
| physics | fixed_attach | 물리 Drive로 손가락 구동, 기존 Qualification과 논리 Attach/Detach |
| physics | physical | 물리 Drive·접촉·마찰만 사용, 자재 이동/충돌 예외/Attach 없음 |

`logical + physical`과 알 수 없는 구현은 설정 오류다. Physics Actuator만 선택했다고 Physical Grasp가 되는 것은 아니다.

`gripper_runtime/`은 Cell 밖 `scripts/`의 재사용 Tool 구현이다. `tool_plugin.py`의 기존 선택 경계가
Actuator와 파지 구현을 조립한다. `GraspManager`는 기존 Fixed Attach를, `PhysicalGraspManager`는
접촉 진단과 보수적인 물리 상태를 소유한다. 새 Tool은 이 선택 경계에 구현과 Tool Profile을 등록한다.
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
현재 시뮬레이션은 position error·저속 지속 시간으로 유사한 obstruction 상태를 추정하고 원시 `gOBJ`나 모터 전류를 합성하지 않는다.

## 설정 소유권과 단위

| 정보 | Owner / 근거 |
|---|---|
| 링크·관절 연결·mimic·위치/속도/effort 한계·collision mesh·질량/관성 | 기존 URDF/USD. Bringup이 URDF에서 `joint_model`을 파생해 export; 별도 수작업 관절 목록 없음 |
| 85 mm stroke | Tool Profile / 제조사 사양, 기존 호환 추상화 |
| force/speed request와 가용 Capability | Tool Profile / 제어 설정 |
| Drive stiffness/damping·토크 예산·정지 판정 허용오차/dwell | Tool Profile `runtime.physics` / 미보정 simulation tuning |
| fingertip 물리 material·마찰·반발 | Tool Profile `runtime.physics.contact_material` / 실험 보정 필요 |

Profile의 0.1–1.0 N·m 토크 예산과 0.8/0.6 마찰은 보정되지 않은 초기 시뮬레이션 값이다.
제조사 force request를 이 토크 범위에 선형 대응하는 것은 **시뮬레이션 제어 추상화**이며 실물 rFR→토크/N 보정 곡선이 아니다.
회전 joint의 Drive `maxForce`는 N·m 토크 한계이고 실제 손가락 접촉력(N)과 동일하지 않다.
DC gain은 radian 기준, USD angular gain/velocity는 degree 기준으로 변환한다. Arm 관절은 Tool 설정 대상에서 제외한다.
기존 mesh collision은 유지하며 URDF의 Gazebo/ODE surface 태그가 PhysX material을 정의한다고 가정하지 않는다.
Physics 모드에서 Tool 접촉 링크에만 명시적인 material과 contact reporting을 적용한다.

## 명령과 관측

기존 GRASP / RELEASE / STOP 및 width 호환 경로를 유지한다. ROS Action·Motion·Orchestration은 변경하지 않는다.
Physics Tool의 `/tool/intent` JSON에는 선택적 `parameters`를 사용할 수 있다.

```json
{"intent":"GRASP","execution_id":"<applied-motion-reference-id>","parameters":{"target_position":1.0,"speed_fraction":0.5,"force_setting":0.5}}
```

세 값은 normalized [0,1]이며 speed는 0보다 커야 한다. 파라미터 없는 GRASP/RELEASE는 현재 speed/force 설정으로 CLOSE/OPEN한다.
STOP에는 추가 파라미터를 받지 않는다. Logical 구현은 추가 제어 Capability를 광고하지 않으며 파라미터 명령을 거부한다.
추가 필드를 모든 Tool에 필수화하지 않는다. `ToolRuntime.observation()`은 제공하는 구현에서만 position/velocity,
target, motion status, inferred object status와 설정을 반환한다.

Physical Grasp에서는 OPEN 완료의 실제 위치 관측 이후 RELEASED를 보고한다. CLOSE·motor stall·양쪽 접촉만으로
하중 유지와 미끄러짐 부재를 증명할 수 없으므로 현재 구현은 **HELD를 발행하지 않고 UNKNOWN을 유지**한다.
따라서 현재 정식 PnP는 Physical Grasp로 성공할 수 없다. 기존의 fresh HELD 요구조건을 완화하거나 새 Mission 중단 조건을 추가하지 않는다.
보유 판정의 향후 보정은 별도 관측 근거를 요구한다.

Physics contact report의 actor pair·접촉점·법선·impulse와 실제 관절 상태는 선택적 진단이다.
`physical_grasp` 기록은 기존 bounded gripper diagnostic writer에 10 Hz로 제출한다. impulse(N·s)는 force(N)가 아니다.
Contact LOST/관측 부재는 보유 성공이 아니며, contact buffer는 tick마다 교체한다. 지속 보유/미끄러짐 검증은 아직 완료하지 않았다.

## 실행

저장소 root에서 기존 Profile을 복사해 선택한다. RB5는 `rb5_850_profile.yaml`을 대신 사용한다.

```bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
from pathlib import Path
import yaml
root = Path.cwd()
out = root / '.run_records'
out.mkdir(exist_ok=True)
tool = yaml.safe_load((root / 'ros2_ws/src/motion/config/tools/robotiq_2f85.yaml').read_text())
tool['runtime'].update(actuator='physics', grasp='physical')
(out / 'physics-tool.yaml').write_text(yaml.safe_dump(tool))
profile = yaml.safe_load((root / 'ros2_ws/src/bringup/config/operational_profile.yaml').read_text())
profile['profiles']['tool'] = str(out / 'physics-tool.yaml')
(out / 'physics-profile.yaml').write_text(yaml.safe_dump(profile))
PY
ARM_CELL_PROFILE="$PWD/.run_records/physics-profile.yaml" bash infra/isaac_sim/scripts/isaac_arm_cell.sh
```

Launcher가 조립한 Runtime Configuration을 사용한다. Script Editor 직접 실행은 기존 export 명령으로 같은 Profile을 export한 뒤
`ARM_CELL_RUNTIME_CONFIG`를 지정하고 기존 Before Play/After Play 순서를 따른다.
원본 generated USD를 다시 저장하지 않는다. Physics + Fixed Attach를 조사하려면 위 선택의 `grasp`만 `fixed_attach`로 지정한다.

## 검증 범위와 사용자 라이브 확인

자동 검증은 Logical 동작·Fixed Attach/Release 회귀, Physics target/read/STOP, unsupported 조합,
URDF 연동 파생·configuration export, 두 Robot의 HOME·Profile 보존을 확인한다.
설치된 Isaac Sim 5.1 API/schema 검토와 USD 조립 검사는 물리 엔진 실행을 대신하지 않는다.
테스트 결과와 기존 기준선 실패는 PR 검증 항목에 기록한다.

사용자 라이브 확인 항목:

- 두 Robot의 기존 Logical PnP 성공과 HOME·Arm Drive 보존.
- Physics에서 85 mm 실제 개방 geometry·joint limit·속도, mimic 오차와 접촉 정지·OPEN·STOP·재시작.
- 빈 CLOSE와 물체/환경 걸림의 진단 구분; 고착을 보유로 보고하지 않는지.
- 실제 payload 질량·형상별 마찰, 토크, 손가락 접촉·유지·미끄러짐/낙하. 현 tuning은 성공 보장이 없다.
- Physical 모드에서 Fixed Joint 생성·자재 순간이동·충돌 무시가 없는지와 UNKNOWN 피드백.
- Physics + Fixed Attach의 실제 접촉 정지 이후 Capture Qualification 가능 여부는 별도 확인.

실제 Isaac Physics tick을 실행한 파지/접촉/하중 유지 검증은 이번 자동 검증의 완료 주장에 포함하지 않는다.
