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

### CODESYS 4 로컬 실통신 시험

[시험용 PLC_PRG](tests/codesys/PLC_PRG.prg.st)는 위와 같은 FC04·FC16 블록을 CODESYS의 `ModbusServer` 함수 블록으로 제공한다. CODESYS 4에서 Linux SL 프로젝트를 생성하고 기본 주기 Task의 `PLC_PRG`를 이 소스로 교체한다. 애플리케이션의 `Libraries.json`에서 `Standard` 참조를 유지하고 `ModbusTCP Server, 4.6.0 (CODESYS)`를 `MB` 네임스페이스로 참조한다. `placeholderOverrides`에서 `IoDrvModbus`와 `IoDrvModbusBase`는 각각 동명의 4.6.0 (CODESYS) 라이브러리로, `CommFB`는 `CommFB, 3.5.21 (CODESYS)`로 지정한 뒤 라이브러리를 resolve하고 빌드한다. Control for Linux SL 4.23.0.0에서 실행했으며 서버는 `lo`의 TCP 1502에 바인딩한다. [CODESYS ModbusServer](https://content.helpme-codesys.com/en/CODESYS%20Modbus/_mod_lib_modbusserver.html)

별도의 시험 클라이언트가 FC06으로 holding register 6에 작은 요청 ID를 쓰면 PLC가 다음 주기에 해당 ID를 게시한다. FC04로 input register 0부터 8 WORD를 읽으면 요청 ID(0–1), PLC가 관측한 응답 ID의 하위 WORD·상태·사유·배치·heartbeat(2–6), 시험 프로그램 식별값 `0xC04E`(7)를 확인할 수 있다. Holding register 7에 1을 쓰면 PLC가 서버를 3초 동안 내린다. 시험을 마치면 holding register 6에 0을 쓴다. 이 추가 레지스터는 로컬 시험 프로그램 전용이다.
