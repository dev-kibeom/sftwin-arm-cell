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


## 지속 생산 이력과 품질 검사

Cell 측 Sidecar는 `conveyor.history_db_path`의 SQLite에 실제 생성 개체·배정·예약·PLC Cycle/Commit 관측과 품질 판정을 남긴다. 제어의 Source of Truth는 기존 Isaac Equipment·Task·PLC이며 DB는 독립 이력이다. 다른 검사 설비는 `/cell/conveyor/record_inspection` (`RecordConveyorInspection.srv`)에 **알고 있는 material_instance_id**와 검사 ID, 설비 ID, 실제 판정(`PASS`/`FAIL`/`INCONCLUSIVE`), 증거 참조를 제출할 수 있다. 이 호출은 PLC Commit 또는 로봇 동작을 승인하지 않는다. 검사 ID는 설비별·개체별 번호다. 카메라와 치수기가 같은 개체에 각각 1번을 제출할 수 있다. 같은 개체·설비·번호의 재전송은 동일 판정·근거일 때 멱등이다.

`/cell/conveyor/query_history` (`QueryConveyorHistory.srv`)에 특정 `material_instance_id`를 보내면 해당 개체의 자재·예약·PLC 관측·검사·변경 기록 JSON을 반환한다. 응답의 `robot_execution.attempts`에는 같은 개체에 연결된 각 PLC 요청 ID와 Mission UUID, 예약 ID, Task UUID·결과가 들어간다. Task terminal이 없으면 `task_outcome`은 null이다. `status`는 조회된 연결(`linked`), 기록된 연결 없음(`no_link_observed`), 영수증 경로 미설정 또는 읽기 실패(`source_not_configured`, `source_unavailable`)를 구분한다. 구버전 영수증에는 개체·Task 연결이 없을 수 있다. 배포 시 Sidecar 스냅샷의 PLC 프로필과 실행 중인 Adapter의 `receipt_file`이 같은 영속 파일을 가리키도록 설정한다. 빈 ID는 보존된 전체 스트림의 생산 집계를 반환한다. 여러 소비자는 읽기 서비스를 이용하거나 동일 호스트의 SQLite WAL 읽기 연결로 보고서를 생성할 수 있다. SQLite 원본 파일은 Git·진단 스냅샷과 독립적으로 유지하고 운영 백업 정책을 별도로 설정한다.

장면이 살아 있고 Sidecar 프로세스가 종료된 경우, Timeline을 Stop하지 않고 Supply UI의 **Start automatic**을 누르면 기존 Equipment에 새 Sidecar를 연결한다. 기존 투입 수량·생성 번호·스트림을 유지하며, 새 허가와 PLC 명령을 받은 뒤 설비를 진행한다. 이 조작은 자동공급도 켜므로 작업자가 운전을 재개할 때 사용한다.

재시작 후 동일 Equipment 스트림이면 이전 예약 Snapshot과 ID를 읽되, 현재 실물과 PLC의 위치 확정이 재관측되기 전에는 해당 예약을 실행 가능한 것으로 공개하지 않는다. 이미 배치한 자재는 기존 예약과 일치하는 COMPLETE 또는 ABANDON 및 Release 확인으로 퇴역시킬 수 있다. 재연결할 스트림의 Snapshot이 누락되면 설비 관측을 계속하며 공급 예약은 보류한다. Stage를 새로 생성했거나 PLC RETAIN과 실제 자재가 불일치하면 과거 이력만 복원하며 물리 작업 자동 복귀는 하지 않는다. Sidecar 복원만으로 Robot Goal, PLC 단계 또는 물리 바디를 재생성하지 않는다.

원장 쓰기가 실패하면 설비 Sidecar는 센서 피드백을 계속 전송한다. 배정·예약 변경은 저장에 성공한 뒤 확인하며, 관측 쓰기는 다음 유효 표본에서 다시 시도한다. 장애 동안 지나간 중간 PLC 단계는 이력에서 누락될 수 있다. 새 자재가 첫 로봇 요청 전인 단계 10·20·21·22에서는 결과·사유·배치를 0(미확정)으로 조회한다. 기존 v1 원장은 처음 열 때 이 귀속을 보정하고 해당 이벤트의 원래 값은 `retained_registers`에 남긴다.

실행 중 Sidecar가 종료되어 유효한 공급 표본이 0.5초 동안 도착하지 않으면 Mission은 진행 중인 Task를 취소한다. Pick 접근 중에 취소되고 배치가 확인되지 않은 경우 PLC는 단계 39(현장 대조)에서 마지막 Commit을 유지한다. 이때 같은 예약의 ABANDON 처리는 Task의 중복 실행이나 다음 자재 공급을 승인하지 않는다.

실행 전에 새 ExecuteCycle 정의로 arm_cell_interfaces 및 이 Action을 사용하는 Mission·Adapter와 클라이언트를 함께 재빌드하고 재시작한다. SQLite 원장은 Python 표준 라이브러리를 사용한다. Pure DB 회귀는 `python3 -m pytest -q infra/isaac_sim/scripts/tests/test_conveyor_production_ledger.py`로 확인할 수 있다. 외부 CODESYS + Isaac 통합 및 재시작 시운전은 별도 live 증거로 확인해야 한다.

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
