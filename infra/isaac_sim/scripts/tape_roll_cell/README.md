# Tape Handling Cell

[공급·이송 계약](../../../../docs/design/components/simulation/fds-tape-cell-supply.md)을 따른다.

Cell Profile은 `ros2_ws/src/bringup/config/cells/tape_roll_cell.yaml`이다.
Cell이 설비 배치, 카트리지 수량·간격, 수령 위치, Tool별 기준 티칭과 지지면 이탈 높이를 소유한다.
롤 치수·질량·마찰은 별도 Material `config/materials/tape_roll/17mm.yaml`이 소유하며
기존 중공 convex-sector Collision과 C10 `internal_radial`/Physical Tool Runtime을 재사용한다.
`experiment.yaml`은 같은 Cell을 참조하는 단일 롤 시험 선택만 남긴다.

## 지원 실행 범위

| 선택 | 지원 범위 |
|---|---|
| `cell_executions/m0609_hand_e_c10_long_tape.yaml` | M0609 + C10 Long, 3개 롤, native 공통 Task `GraspCycle` BT 물리 시험 |
| `cell_executions/m0609_hand_e_c10_tape.yaml` | M0609 + 기본 C10, 같은 공정의 물리 시험 |
| 기존 `model_experiments/m0609_hand_e_c10*.yaml` | Cell 설정을 재사용하는 단일 롤 실험 |
| ROS Mission / Safety / 운영 Motion Backend | 미지원. ROS launcher는 fixture Profile을 명시적으로 거부한다. |

시험용 MotionPort는 티칭을 실행하는 native BT fixture Adapter다. 운영 ROS Backend로 등록하지 않는다.
Vision을 호출하거나 검출 성공을 합성하지 않는다. 기존 CNC Cell과 공통 Runtime Workflow는 유지한다.
HCR-14, HOME에서 셀까지의 이동, 동적 컨베이어 구동은 이 실행 범위에 포함하지 않는다.

## 실행

Launcher에서도 별도 Tape Cell 물리 시험 버튼으로 실행할 수 있다. ROS Mission 버튼은 활성화하지 않는다.

ROS overlay와 `BUILD_TESTING=ON`으로 빌드한 Task Execution fixture bridge를 준비한다.
저장소 README의 빌드 절차를 따른 후:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
infra/isaac_sim/scripts/tape_roll_cell/run.sh \
  ros2_ws/src/bringup/config/cell_executions/m0609_hand_e_c10_long_tape.yaml \
  "$PWD/ros2_ws/build/arm_cell_task_execution/libgrasp_cycle_fixture_bridge.so" \
  --headless --cycles 1
```

`--cycles 1`은 카트리지 하나를 소진할 때까지 처리한다. 기본형은 Profile 이름에서 `_long`을 뺀다.
`ISAAC_SIM_ROOT` 기본값은 `~/isaac_sim`이다. `--headless`를 생략하면 화면을 볼 수 있다.

## 동작과 판정

기본 카트리지는 폭 17 mm 롤 3개를 +X 축으로 40 mm 간격에 배치한다.
로봇 쪽 슬롯부터 선택하고, 수령 측은 먼 위치부터 채워 다음 삽입 통로를 보존한다.
기준 티칭을 각 슬롯의 Cartesian 위치로 옮긴 뒤 선택 URDF의 제한 내 IK를 풀고 FK 오차를 확인한다.
모든 슬롯의 Extract는 카트리지 전면 밖까지 이어지며, 슬롯 사이에는 위쪽 복귀 경로를 사용한다.
관절값에 일정 값을 더하지 않는다. 설정 export에는 SciPy·NumPy·Xacro가 필요하다.
별도 Tool 소유 물리 시험 구동 설정으로 PhysX 관절 속도 제한값을 URDF 제한의 절반으로 설정한다.
기존 C10 모델 실험의 drive 기본값과 CNC·2F-85 설정은 바꾸지 않는다.

각 슬롯은 Approach → Insert → 물리 Grasp/Holding 확인 → Relation 확정 → Lift → 수평 Extract →
상부 Clearance → 수령 위치 → Release 확인 → Relation 해제 → Retreat를 수행한다.
Lift는 카트리지 바닥에서 이탈해 앞쪽 빈 크래들의 마찰 충격을 피한다.
Cartesian 중간점마다 IK를 풀고 관절 보간 중간점의 FK 오차도 검사한다.
실행 중 모든 Collision을 유지한 Isaac 접촉 검사로 실제 경로를 검증한다.
설정의 FK·IK 검사는 경로의 무충돌을 보장하지 않는다. 배치·간격·수량 변경에는 새 라이브 검증이 필요하다.
첫 physics step 전 Approach 초기화는 시험 준비에 한정된다.

Cell 공급 관리가 슬롯 점유와 처리 완료 수량을 소유한다. 실제 이송 거리·수령 위치,
물리 Release와 후퇴 후 안착을 모두 확인한 슬롯만 정확히 한 번 차감한다.
실패하면 다음 슬롯을 실행하지 않고 미완료 슬롯과 이미 완료된 수량을 보존해 기록한다.
Timeline Stop은 종료 전 상태를 기록한 후 초기 롤 위치·속도와 카트리지 공급 상태를 복원한다.
Playing 중 물체 위치·속도 재설정, 강제 Attach, Fixed Joint, 중력/Collision 해제는 사용하지 않는다.

C10은 실제 prismatic drive와 세 핑거의 접촉 반력·측정 obstruction·dwell로 Holding을 판단한다.
STOP은 측정 위치를 유지하며 성공 관측을 합성하지 않는다.
접촉 누락·Holding 소실·미선택 롤 접촉·비핑거 Robot/Roll 접촉·Robot/설비 접촉은 실패한다.
수령 위치 오차·선속도·각속도·수평 축도 판정하며, BT 반환만으로 완료 처리하지 않는다.

`.run_records/tape-roll-*/`의 configuration/result JSON에 입력 hash, 소스 hash, native library/BT hash,
위상별 TCP/Roll 변환, 접촉·핑거·Holding·Relation, 슬롯별 수량 변화와 Stop 복원 결과를 보존한다.
실패한 실행은 FAIL을 유지한다. 공급 상태의 `occupied`는 공급 슬롯의 미완료 여부이며,
실패 후 실제 물체 위치를 추정하거나 자동 재시도를 허용하는 센서 관측이 아니다.
