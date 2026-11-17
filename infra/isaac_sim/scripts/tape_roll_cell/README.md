# Tape Handling Cell

Launcher에서 Robot **M0609**, Tool **Hand-E C10** 또는 **C10 + 100 mm 핑거**,
Cell **Tape Handling Cell**, Grasp Mode **Physical Real**을 선택한다.
일반 **Isaac 시작 → ROS 2 시작** 순서로 실행하고, Cell 창의 **남은 카트리지 처리**로 Mission을 요청한다.
모델 실험·별도 native fixture 버튼과 실행기는 제공하지 않는다.

Cell Profile은 `ros2_ws/src/bringup/config/cells/tape_roll_cell.yaml`, 운영 배포는
`tape_m0609_c10_profile.yaml` / `tape_m0609_c10_long_profile.yaml`이다.
설비·슬롯 수량·간격·티칭은 Cell, 중공 형상·질량·마찰은 Material,
세 핑거 구동·접촉 판정은 Tool, HOME·관절 제한은 Robot이 소유한다.

## 실제 실행 경로

Configured-source Mission → Cell 슬롯 예약 → `/task_execution/execute_configured_grasp`
→ 공통 `GraspCycle` BT → 동일 프로세스의 운영 `MotionCore`·`GripperPort` → Isaac ROS Backend다.
Task server는 등록된 슬롯 작업만 수락한다. 시험용 Callback MotionPort나 fixture library를 불러오지 않는다.
Vision 노드를 실행하거나 슬롯 좌표를 Vision Acquisition으로 만들어 보내지 않는다.
CNC Cell의 Vision Mission과 2F-85 모드 선택은 유지한다.

Robot은 정지 상태에서 Robot 소유 HOME으로 초기화한다. HOME에서 Cell 진입점까지는 MoveIt이
실제 Planning Scene으로 전역 계획하며, 이후 Cartesian 티칭은 선택 URDF의 IK·구간 FK와 Motion의
관절 제한·속도·가속도·충돌 검사를 통과해야 한다. 실행 완료는 실제 관절 상태와 Simulation Clock을 사용한다.
배치·간격·수량 변경에는 새 경로·라이브 검증이 필요하다.

## 물리 파지와 공급 회계

기본 카트리지는 폭 17 mm 롤 15개를 +X 방향으로 40 mm 간격에 배치한다.
로봇 쪽부터 선택하고 수령 측은 먼 위치부터 채운다. Insert → GRASP/Holding → Lift →
카트리지 전면 밖으로 수평 Extract → 상부 Clearance → 수령 위치 → RELEASE → Retreat를 반복한다.
슬롯 사이에는 높은 복귀 경로를 사용한다.

`Physical Real`은 세 핑거의 예약 대상 접촉 반력, 측정 팽창 obstruction 및 dwell로 HELD를 판단한다.
RELEASED에는 실제 수축과 접촉 소실이 필요하다. 실행 ID·슬롯·예약 token·epoch가 맞아야 하며,
결측·오래된 관측·STOP·잘못된 참조는 성공으로 처리하지 않는다. 폭 API는 제공하지 않는다.
실시간 콜백에서는 시작 때 검증한 기하 해를 재사용한다.

Cell이 점유·예약·완료를 소유한다. Task의 확인된 해제 사실에 더해 실제 수령 위치·속도·축 방향과
현재 물리 RELEASED를 확인한 경우만 정확히 한 번 완료 처리한다. 실패·불확실 terminal에는 자동 재시도하지 않는다.
Timeline Stop은 공급 epoch를 교체하고 초기 롤 위치·속도와 수량을 복원해 이전 예약을 무효화한다.

모든 롤의 중력·충돌·마찰을 유지하며 Playing 중 강제 Attach나 물체 pose/velocity 변경을 하지 않는다.
MoveIt의 중공 Planning Geometry와 논리적인 AttachedCollisionObject는 충돌 검사용이며 PhysX Joint가 아니다.
예약 대상과 핑거의 접촉·기존 지지면 접촉만 계획상 허용한다. 실제 물리 충돌은 계속 켜져 있으며
비핑거/미선택 롤/설비 접촉은 공급을 invalid로 만들고 Mission을 중단한다.

## 빌드와 CLI

저장소 README의 ROS 빌드 절차를 따른다. 운영 실행에는 테스트 라이브러리가 필요하지 않다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ARM_CELL_PROFILE="$PWD/ros2_ws/src/bringup/config/tape_m0609_c10_long_profile.yaml" \
  infra/isaac_sim/scripts/isaac_arm_cell.sh
# Isaac 초기화 후 별도 터미널에서 같은 export Profile로 ROS를 실행한다.
ros2 launch arm_cell_bringup arm_cell.launch.py \
  profile:="$PWD/.run_records/runtime_configuration.json"
ros2 action send_goal /orchestration/execute_cycle arm_cell_interfaces/action/ExecuteCycle \
  '{target_id: TapeRoll17, quantity: 3}' --feedback
```

설정 export는 NumPy·SciPy·Xacro를 사용한다. 원본 실행 기록은 `.run_records/`와 ROS/Kit 로그에 남긴다.
HCR-14는 정적 모델 검증 범위이며 C10 운영 배포로 등록하지 않는다. 동적 컨베이어 구동은 포함하지 않는다.

C10 조립 모델은 M0609 Robot 설정의 `radial_precision` drive profile을 선택한다. 중력 처짐을 줄여 좁은 내경 삽입과 0.003 rad 관절 도달 기준을 만족시키기 위한 Isaac 구동 설정이며, 기본 M0609·2F-85 조립에는 적용하지 않는다.

장면은 초록색 에폭시 바닥, 로봇 지지대, 주황 카트리지, 청록 수직 Place 수령대와 슬롯 번호를 표시한다. GUI는 Cell 설정의 조감 카메라로 시작한다. 바닥·지지대·카트리지 지지판·수령 패드는 PhysX와 MoveIt에 같은 형상으로 등록하며 프레임·다리 외관은 충돌 Scene에 발행하지 않는다.

기본 카트리지는 30 mm 피치로 17 mm 롤 15개를 한 줄 보관한다. Pick은 수평이며, 이탈 후 높은 위치에서 Tool을 아래 방향으로 회전하고 3×5 수령대에 수직 하강하여 Place한다. Release 후 수직 상승한다. 수령대는 내경 아래가 열린 외주 지지 패드이며, 완료 판정은 평평하게 안착한 실제 위치·자세·속도와 Release 관측을 함께 요구한다.

SDK default Ground Plane은 생성하지 않고 기존 기본 prim도 제거한다. Cell 소유 초록 에폭시 바닥과 로봇 pedestal은 물리·계획 충돌 Scene에 함께 등록한다. Robot base frame은 지지대 상면에 유지한다. 프레임·다리 외관은 표시용이며 기존 자재 물리 파지를 대체하지 않는다.
