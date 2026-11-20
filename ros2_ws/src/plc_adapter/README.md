# CODESYS SoftPLC와 단일 로봇 작업 연결

CODESYS Control for Linux SL을 Modbus TCP Server로, ROS 2 Humble Adapter를 Client로 사용한다. PLC 프로그램은 요청 ID를 출력하고 결과 블록을 입력으로 읽는다. Adapter가 연결을 복원하고 결과를 반복해서 쓴다. 부팅 시 자동으로 Mission을 실행하는 구성은 필요하지 않다. [CODESYS Modbus TCP Server Device](https://content.helpme-codesys.com/en/CODESYS%20Modbus/_mod_edt_slave_device_tcp.html) · [요청·응답 ICD](../../../docs/design/interfaces/icd-plc-task.md)

### PLC 측 구성

1. CODESYS Development System에서 대상 Control for Linux SL 장치의 Ethernet 아래에 **Modbus TCP Server Device**를 추가한다. 시험 프로필은 로컬 TCP 1502, Unit ID 1이다. 별도 호스트에서는 시험망 인터페이스에 바인딩하고 YAML의 주소와 포트를 맞춘다.
2. Input register 2 WORD(%QW), holding register 6 WORD(%IW)를 구성하고 두 영역의 overlay와 holding register의 Writable 옵션을 끈다. 시작 주소는 각 데이터 영역의 0 기준 PDU 주소다. 한 주기 안에서 블록을 갱신·샘플링한다.
3. 예시 프로필은 아래 표의 순서를 사용한다. 다른 배치는 YAML의 `request_fields`·`reply_fields` 순서와 시작 주소에 맞춰 PLC 변수를 I/O Mapping에 연결하고 프로젝트를 다운로드해 Run 한다. 요청 ID는 재기동 후에도 유지한다. 일반 PLC 통신과 별도로 설비 운전·안전 허가를 적용한다.

| Modbus 블록 | 시작 주소 기준 WORD 순서 |
|---|---|
| FC04 read input registers (PLC → Adapter) | 요청 ID 상위 WORD, 하위 WORD |
| FC16 write multiple registers (Adapter → PLC) | 응답 ID 상위 WORD, 하위 WORD, 상태, Mission 종료 사유, 배치 사실, heartbeat |

### ROS 측 구성

Ubuntu 22.04의 `python3-pymodbus`(2.x)와 ROS Humble을 사용한다. 이 패키지는 독립 실행형이다. Configured Mission과 해당 Cell의 Supply·Task 서버가 같은 ROS 도메인에 있어야 한다. `target_id`는 Mission에 등록된 자재와 일치시킨다. 동작 중인 시스템에 새 Action 정의를 적용할 때는 `ExecuteCycle` 생산자와 소비자를 같은 인터페이스로 재빌드하고 재시작한다.

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/interfaces src/orchestration src/plc_adapter --ignore-src -y
colcon build --packages-up-to arm_cell_orchestration arm_cell_plc_adapter
source install/setup.bash
ros2 run arm_cell_plc_adapter plc_task_adapter --ros-args \
  --params-file src/plc_adapter/config/conveyor_pnp_codesys.yaml
```

`receipt_file`은 요청 스트림마다 전용 영속 경로를 지정한다. 동일 스트림에는 Adapter 하나만 연결한다. 초기 시운전은 PLC 요청 ID 0과 빈 기록에서 시작한다. heartbeat의 변화와 유휴 응답을 확인한 뒤 ID 1을 유지하고 응답 ID·상태·사유·배치 값을 관찰한다. 같은 ID를 유지한 채 TCP를 끊었다 복원하면 같은 작업의 기록이 재전달된다. 실제 운전 중인 스트림의 기록을 지우거나 다른 PLC에 연결하면 상관관계를 잃는다.

ConveyorPart는 예시 자재 키다. 같은 Pick 위치에 연속 도착하는 자재는 Cell 공급자가 도착 건마다 현재 epoch의 서로 다른 미완료 논리 슬롯을 제공하고 같은 티칭 위치에 대응시킨다. PLC 요청만으로 점유나 새 슬롯을 생성하지 않는다. 실제 컨베이어 공급자·티칭 registry와 배출/Commit 프로그램은 Cell 통합 범위다. 패키지 통합 시험은 루프백 Modbus 서버, 실제 Configured Mission, 시험용 슬롯·Robot Task를 사용한다.
