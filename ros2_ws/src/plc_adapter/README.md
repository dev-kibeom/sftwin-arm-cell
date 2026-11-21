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

Adapter가 영수증을 연 동안에는 같은 디렉터리의 `.writer.lock` 파일 잠금으로 단일 writer를 유지한다. 운전 중 이 파일을 교체하거나 제거하지 않는다. 같은 호스트의 History 조회는 WAL 읽기 연결에서 개체에 연결된 실행만 읽는다.

`receipt_file`은 요청 스트림마다 전용 영속 경로를 지정한다. 동일 스트림에는 Adapter 하나만 연결한다. 초기 시운전은 PLC 요청 ID 0과 빈 기록에서 시작한다. heartbeat의 변화와 유휴 응답을 확인한 뒤 ID 1을 유지하고 응답 ID·상태·사유·배치 값을 관찰한다. 같은 ID를 유지한 채 TCP를 끊었다 복원하면 같은 작업의 기록이 재전달된다. 실제 운전 중인 스트림의 기록을 지우거나 다른 PLC에 연결하면 상관관계를 잃는다.

Configured Mission은 ROS 2 Humble `ActionServer`의 기본 `result_timeout=900`초를 사용한다. 기준 시각은 해당 Goal이 terminal에 도달한 Mission 서버 시각이다. Adapter가 중단된 동안 완료된 작업은 같은 Mission 서버가 결과를 보유하는 동안 원래 UUID로 조회해야 한다. 재시작·ROS 탐색·재조회에 걸리는 시간도 이 창에 포함된다. Mission 서버가 재시작하면 창 안에서도 미저장 결과를 잃을 수 있다. Adapter가 이미 SQLite에 기록한 terminal은 서버 보존 기간이 지나도 같은 PLC ID로 읽을 수 있다. [Humble ActionServer 구현](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/action/server.py)

ConveyorPart는 예시 자재 키다. 같은 Pick 위치에 연속 도착하는 자재는 Cell 공급자가 도착 건마다 현재 epoch의 서로 다른 미완료 논리 슬롯을 제공하고 같은 티칭 위치에 대응시킨다. PLC 요청만으로 점유나 새 슬롯을 생성하지 않는다. 실제 컨베이어 공급자와 티칭 registry는 Cell 통합 범위다. PLC 공정 프로그램과 모의 설비 시험은 아래에서 설명한다. 패키지 통합 시험은 루프백 Modbus 서버, 실제 Configured Mission, 시험용 슬롯·Robot Task를 사용한다.

### CODESYS 4 로컬 실통신 시험

[시험용 PLC_PRG](../../../infra/plc/common/commissioning/PLC_PRG.prg.st)는 위와 같은 FC04·FC16 블록을 CODESYS의 `ModbusServer` 함수 블록으로 제공한다. CODESYS 4에서 Linux SL 프로젝트를 생성하고 기본 주기 Task의 `PLC_PRG`를 이 소스로 교체한다. 애플리케이션의 `Libraries.json`에서 `Standard` 참조를 유지하고 `ModbusTCP Server, 4.6.0 (CODESYS)`를 `MB` 네임스페이스로 참조한다. `placeholderOverrides`에서 `IoDrvModbus`와 `IoDrvModbusBase`는 각각 동명의 4.6.0 (CODESYS) 라이브러리로, `CommFB`는 `CommFB, 3.5.21 (CODESYS)`로 지정한 뒤 라이브러리를 resolve하고 빌드한다. Control for Linux SL 4.23.0.0에서 실행했으며 서버는 `lo`의 TCP 1502에 바인딩한다. [CODESYS ModbusServer](https://content.helpme-codesys.com/en/CODESYS%20Modbus/_mod_lib_modbusserver.html)

별도의 시험 클라이언트가 FC06으로 holding register 6에 작은 요청 ID를 쓰면 PLC가 다음 주기에 해당 ID를 게시한다. FC04로 input register 0부터 8 WORD를 읽으면 요청 ID(0–1), PLC가 관측한 응답 ID의 하위 WORD·상태·사유·배치·heartbeat(2–6), 시험 프로그램 식별값 `0xC04E`(7)를 확인할 수 있다. Holding register 7에 1을 쓰면 PLC가 서버를 3초 동안 내린다. 시험을 마치면 holding register 6에 0을 쓴다. 이 추가 레지스터는 로컬 시험 프로그램 전용이다.


### Conveyor PnP Cell 공정 프로그램

[Conveyor.fbsdev](../../../infra/plc/conveyor_pnp_cell/Conveyor.fbsdev)에는 Linux SL 장치, 라이브러리 버전과 [공정 PLC_PRG](../../../infra/plc/conveyor_pnp_cell/Conveyor.fbsdev/Application.iecapp^/PLC_PRG.prg.st)를 실행하는 주기 Task가 들어 있다. 새 체크아웃에서는 프로젝트가 지정한 장치 정의를 설치하고 저장소 루트에서 `c4-cli library install infra/plc/conveyor_pnp_cell/Conveyor.fbsdev/Application.iecapp`로 라이브러리 캐시를 복원한다. PLC에 접속할 때는 IDE에서 로컬 게이트웨이를 선택한다. Linux SL의 RETAIN 저장소를 구성한다. PLC는 활성 단계와 자재 ID, 로봇 요청 ID, 원래 terminal 결과 및 마지막 Commit ID를 유지한다. ID는 1부터 연속 증가하며 최댓값에서 멈춘다. 마지막 Commit ID는 그 ID까지 연속 인수가 끝났음을 뜻한다. 새 프로그램 다운로드·콜드 리셋·전원 상실 시 RETAIN 복원 특성은 대상별로 검증한다. PLC·Adapter 기록 또는 설비 생성 이력이 유실되면 현물과 대조해 동일 스트림을 복원한 후 자동운전을 시작한다.

아래는 loopback 프로필의 0 기준 WORD 주소다. ID는 상위 WORD 먼저, 명령과 개별 BOOL은 0/1이다. 설비는 일관된 새 관측마다 HR16–25를 한 번의 FC16으로 쓰고 IR16–29를 한 번의 FC04로 읽는다. HR의 각 블록에는 지정한 생산자 하나만 쓴다.

| 영역 | 생산자 → 소비자 | 순서와 의미 |
|---|---|---|
| IR0–1 / HR0–5 | PLC → Adapter / Adapter → PLC | 기존 단일 작업 요청 / 결과 블록 |
| HR6–7 | Adapter → PLC | 새 Mission 수락 표본의 순번(1~65535), 수락 가능 여부. 미수신·만료는 (0, 0) |
| HR16–20 | 설비 → PLC | 표본 순번, 전체 관측 유효, UI 자동운전, 자재 코드(1 = ConveyorPart), 남은 공급 가능 수량 |
| HR21 | 설비 → PLC | bit 0부터 투입 감지, Pick 감지, 벨트 실제 정지, 스토퍼 실제 고정, 트레이 점유, 이송로 비움, 드라이브 가동 가능, 로봇 Pick 영역 이탈, 로봇 배출 영역 이탈, 설비 동작 허가, 이번 자재 이송로 위 확인 |
| HR22–23 / 24–25 | 설비 → PLC | 마지막으로 실제 생성한 자재 ID / 현재 Pick 자재의 Configured Supply 슬롯 연결 확인 ID(근거 없으면 0) |
| IR16–18 | PLC → 설비·관측기 | 자재 ID, 단계(0·10 공급, 20·21·22 이송·고정, 23 미실행 재수락 대기, 30 로봇 응답 대기, 39 배치 미확정 현장 대조, 40 terminal 판정·인수 대기, 50·51 배출·재사용 대기) |
| IR19–23 | PLC → 설비 | `feed_part_command`, `conveyor_run_command`, `stopper_extend_command`, `pick_position_confirmed`, `discharge_tray_command` |
| IR24–29 | PLC → 관측기 | 마지막 Commit ID 상·하위, 최근 요청에 저장한 terminal 상태·사유·배치, 앱 식별값 0xC051 |

설비와 로봇 링크의 갱신 감시 시간은 각각 1초다. 설비는 현재성이 확인된 관측을 써야 하며 입력 일부가 불명이면 전체 관측 유효를 내린다. 통신 중단 시 설비 측에서도 명령 적용을 중지한다. 로봇 영역 및 설비 동작 허가는 설비·안전 제어계가 관측한 값이다. P4는 정지 중에도 인수를 기록할 수 있다.

**실행.** Configured Mission에는 [공정 파라미터](config/conveyor_process.yaml)를, Adapter에는 기존 [Modbus 프로필](config/conveyor_pnp_codesys.yaml)과 공정 파라미터를 차례로 전달한다. 같은 ROS 도메인에 해당 공급 service 및 Robot Task가 필요하다. Conveyor 프로필은 `plc_admission_enabled`를 켠다. Mission은 유휴·예약·하위 서비스·공급·Safety와 1초 이내의 Motion 상태를 평가해 100 ms마다 `/orchestration/mission_admission_available`을 발행한다. 기존 PR의 `robot_ready_topic`·`ready_holding_offset` 설정은 `mission_admission_topic`·`admission_holding_offset`으로 갱신한다. HR6–7 주소와 로봇 작업 블록은 동일하다. Adapter는 500 ms 넘게 새 표본이 없으면 거짓을 전달한다. PLC는 준비 표본과 별도로 결과 블록의 heartbeat를 감시한다.

**실제 런타임에서 Mock 시운전.** 다른 I/O 생산자가 없는 전용 loopback Linux SL을 새 공정 스트림(ID 0)으로 기동한다. 위 ROS 패키지를 빌드하고 overlay를 source한 뒤 저장소 루트에서 실행한다.

```bash
python3 ros2_ws/src/plc_adapter/tests/codesys/commission_conveyor.py --port 1502
```

시험은 실제 Adapter와 Mission을 별도 프로세스로 시작하고 Conveyor 공급자에 Mock 도착을 연결하고 설비·Motion·Safety 입력과 Robot Task를 모의한다. PLC 단계·Commit은 FC04로 읽는다. 기동 시 공급 가능 수량을 5로 설정한다. 잘못된 대상의 수락 거부에서 Pick 슬롯을 유지하고 수락 가능 표본을 반복한다. 수락 상태 복원 또는 설정 수정 후 자동운전 재개에서만 같은 자재를 재요청한다. BT 진입 전 실행권 Busy에는 1초 재시도 간격을 적용한다. 배치 후 GO_HOME 실패와 Safety 선점이 겹치면 Mission 종료 사유와 배치 사실을 함께 확인한다. 입력 지연·구동 불가·로봇 대기, Adapter 재시작과 terminal 재전달, 인수·배출 보류, 5회 Commit과 재고 소진을 확인한다. 재보충 후 여섯 번째 자재의 배치 전 실행 실패에서 Commit과 재공급이 보류되는지 확인한다. 구 통신 시험용 앱(0xC04E)은 공정 시험 전에 교체해야 한다.
