# Conveyor PnP Cell 설비

[Cell 프로필](../../../../ros2_ws/src/bringup/config/cells/conveyor_pnp_cell.yaml)이 벨트 속도, 스토퍼 이동, 자재 치수와 감지 창을 소유한다. 고정 설비는 같은 디렉터리의 `conveyor_static.json`을 Isaac과 planning scene이 함께 읽는다. 센서는 월드 좌표계의 자재 중심을 측정한다. PhysX 벨트 표면 속도로 정지를, 스토퍼 강체의 횡방향 이동 끝점으로 고정을 판정한다. Pick 및 배출 로봇 영역은 PhysX 충돌체 overlap으로 확인한다. PLC가 Commit 후 배출을 허가하면 트레이에서 확인한 해당 자재 prim을 제거한다.

## 실행

ROS 2 Humble overlay에 `arm_cell_bringup`, `arm_cell_interfaces`, `arm_cell_plc_adapter`를 빌드하고 시스템 Python에 `python3-pymodbus` 2.x를 설치한다. CODESYS에는 Conveyor 공정 프로그램을 사용한다. 설비 Sidecar는 기존 Adapter 프로필의 Modbus 접속 설정을 읽고 HR16–25만 쓴다. HR0–7은 별도 Robot Adapter의 영역이다. 시험용 Mock과 Robot Adapter 중 하나만 해당 영역을 소유해야 한다. [PLC 실행 및 매핑](../../../../ros2_ws/src/plc_adapter/README.md#conveyor-pnp-cell-공정-프로그램)

저장소 루트에서 ROS 환경으로 스냅샷을 export한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ARM_CELL_PROFILE="$PWD/ros2_ws/src/bringup/config/conveyor_pnp_profile.yaml"
PYTHONPATH="$PWD/ros2_ws/src/bringup:$PYTHONPATH" python3 ros2_ws/src/bringup/scripts/export_runtime_configuration \
  --profile "$ARM_CELL_PROFILE" --output /tmp/conveyor-runtime.json
```

Isaac 환경에서 `infra/isaac_sim/scripts/isaac_arm_cell.sh --runtime-config /tmp/conveyor-runtime.json`으로 실행한다. ROS Python 경로는 별도 Sidecar가 설정한다. 다른 overlay를 사용할 때는 `ARM_CELL_ROS_SETUP`에 그 setup.bash 경로를 지정한다. 두 프로세스는 같은 ROS 도메인을 사용한다.

- 새 장면은 빈 설비와 PLC의 자재 ID·단계·마지막 Commit이 모두 0인 표본에서 동기화한다. UI에서 자재 종류·가용 수량을 설정하고 자동운전을 켠다. PLC의 식별된 공급 명령이 와야 자재가 생성된다.
- 설비 허가 생산자는 `/cell/conveyor/equipment_permitted`에 `std_msgs/Bool`을 주기적으로 제공한다. 이 값은 시뮬레이션 설비 동작 허가다. 미수신·만료는 설비 동작을 보류한다. 운영 사용에는 해당 허가 생산자를 연결한다.
- UI 자동운전 일시 정지, Timeline Pause, TCP 재접속은 같은 자재와 생성 영수증을 유지한다. Timeline Stop은 설비를 보류한다. 프로세스 재기동으로 생성 이력이나 예약을 잃은 경우에는 PLC RETAIN과 현물을 대조해야 한다.

## 고정 Pick과 Robot BT

공급자는 PLC의 위치 확정과 실제 도착 ID를 대조해 고정 Pick 슬롯에서 현재 자재의 품종과 개체 ID를 공개한다. Task는 유효한 예약을 확인하고 고정 Pick/Place 티칭 한 개를 M0609와 지름 30 mm 흡착컵으로 실행한다. Tool은 예약의 개체 ID와 투입 시 기록한 품종으로 흡착할 바디를 확인한다. 로봇 베이스는 월드 X=0.30 m에 설치한다. TCP는 컵의 접촉면 중앙이며 +Z가 자재 상면을 향한다. Pick은 월드 (0.754, 0.38, 0.332) m, Release는 (0.46, −0.38, 0.38) m다.

Tool은 Isaac 5.1 Surface Gripper의 상태와 Gripped Objects를 매 물리 스텝 읽는다. HELD에는 현재 공급·epoch·예약·슬롯으로 확인된 단일 자재 prim과의 결합이 필요하다. 목적지에서 기존 GraspCycle이 확인한 Release는 `material_placed`로 전달된다. 설비는 별도로 트레이 점유를 관측하고 기존 PLC가 Commit과 배출을 결정한다.

Robot BT 연속 생산 시운전은 같은 프로필·ROS 도메인에서 다음 두 프로세스를 각각 시작한다. 새 시험 스트림에서는 전용 PLC 인스턴스와 기록 경로를 사용한다. 설비 허가를 공급하고 UI 가용 수량을 3으로 설정한 뒤 자동운전을 켠다.

```bash
ros2 launch arm_cell_bringup arm_cell.launch.py profile:="$ARM_CELL_PROFILE"

ros2 run arm_cell_plc_adapter plc_task_adapter --ros-args \
  --params-file ros2_ws/src/plc_adapter/config/conveyor_pnp_codesys.yaml \
  --params-file ros2_ws/src/plc_adapter/config/conveyor_process.yaml
```

전용 스트림에서 세 자재를 자동 검증할 때는 위 ROS 실행과 같은 스냅샷 및 설비 허가 생산자를 사용하고, Isaac을 아래 관측 스크립트로 시작한다. 스크립트가 UI 공급 포트로 수량과 자동운전을 설정하며 실제 결합·해제, 점유, PLC Commit 및 배출 후 관절 위치를 기록한다. ROS launch에 `diagnostics:=orchestration`을 지정하면 Task 결과를 슬롯·예약별로 대조할 수 있다.

```bash
export SFTWIN_PROJECT_ROOT="$PWD" ARM_CELL_RUNTIME_CONFIG=/tmp/conveyor-runtime.json
~/isaac_sim/python.sh infra/isaac_sim/scripts/conveyor_pnp_cell/tests/commission_robot_bt.py
```

## 실제 설비 반복 시험

전용 loopback CODESYS 인스턴스를 초기 ID 0으로 준비한다. 동일한 전용 ROS 도메인을 쓰고, 세 프로세스에 같은 빈 임시 디렉터리를 전달한다. 아래 예시는 PLC의 1502 포트와 시험 중 끊을 설비 프록시의 11502 포트를 쓴다. 경로는 저장소 루트 기준이다.

```bash
tests=infra/isaac_sim/scripts/conveyor_pnp_cell/tests
run_dir=$(mktemp -d /tmp/conveyor-commission.XXXXXXXX)
export ROS_DOMAIN_ID=109 ROS_LOCALHOST_ONLY=1
# 시스템 Python, 별도 터미널 두 개
python3 "$tests/equipment_proxy.py" "$run_dir" --listen 11502 --plc-port 1502
python3 "$tests/mock_robot.py" "$run_dir" --robot-port 1502
# Isaac Python, 같은 run_dir 및 ROS 도메인
export SFTWIN_PROJECT_ROOT="$PWD" ARM_CELL_RUNTIME_CONFIG=/tmp/conveyor-runtime.json
~/isaac_sim/python.sh "$tests/commission_isaac.py" "$ARM_CELL_RUNTIME_CONFIG" "$run_dir" --equipment-port 11502
```

Mock은 설비 허가와 로봇 I/O를 제공하고 실제 ROS 공급 서비스에서 예약한다. 예약한 자재를 Isaac 트레이로 옮긴 뒤 실제 점유를 확인해 로봇 결과를 반환한다. 테스트는 투입 센서 지연, 자동운전 일시 정지와 TCP 단절 중 동일 자재 유지, 두 자재의 개별 예약·Commit·배출을 검사한다. 두 프로세스의 PASS와 종료를 확인하고 프록시와 전용 PLC를 종료한다.
