# Conveyor PLC

런처에서 **PLC 시작 시 CODESYS GUI 열기**를 선택하면 현재 체크아웃의 [Conveyor PnP Cell 프로젝트](projects/Conveyor.fbsdev)가 열린다. 실행 중인 SoftPLC는 마지막으로 배포된 부팅 앱과 보존된 공정 상태를 사용한다.

| 목적 | 열어 볼 파일 | 확인할 내용 |
|---|---|---|
| Conveyor 공정 읽기 | [PLC_PRG.prg.st](projects/Conveyor.fbsdev/Application.iecapp^/PLC_PRG.prg.st) | `CASE phase OF`에서 투입, 이송, 픽 확인, 로봇 요청, 배출 순서와 전이 조건을 따라간다. 공정 앱 식별값은 IR29의 `0xC051`이다. |
| CODESYS 빌드 구성 확인 | [주기 태스크](projects/Conveyor.fbsdev/Application.iecapp^/TaskConfiguration.json), [장치 트리](projects/Conveyor.fbsdev/Devices/DeviceTree.json), [라이브러리 잠금 파일](projects/Conveyor.fbsdev/Application.iecapp^/Libraries.lock.json) | CODESYS가 여는 애플리케이션은 `Application.iecapp`이고, 그 소스와 설정은 짝을 이루는 `Application.iecapp^/`에 있다. |
| Modbus 연결만 시운전 | [시험용 PLC_PRG.prg.st](commissioning/PLC_PRG.prg.st) | 요청 반향과 연결 중단을 시험하는 별도 ST 소스다. IR7의 식별값은 `0xC04E`이며 HR6·HR7도 시험 입력으로 사용한다. |

라이브러리 복원과 PLC/ROS 레지스터 매핑은 [어댑터 안내](../../ros2_ws/src/plc_adapter/README.md#conveyor-pnp-cell-공정-프로그램), Isaac·PLC·ROS의 기동 순서는 [Conveyor 실행 절차](../isaac_sim/scripts/conveyor_pnp_cell/README.md#실행)에 있다.

## 실패 사례로 공정 읽기

2026년 11월 21일의 초기 진단 한 건과 공급 판단을 기록한 후속 실행 네 건이다. 각 기록에서 당시 값을 먼저 보고 PLC가 다음 단계로 넘어가는 데 필요한 입력을 찾아볼 수 있다.

| 라이브에서 보인 증상 | 처음 비교할 값 | 사례 |
|---|---|---|
| 공급 관측 지연만 기록됨 | ROS 기동 조건과 공급 판단 기록 유무 | [16:40 · 당시 기록으로 알 수 있던 범위](../../docs/reports/live-test/20261121-164025.md) |
| 자동운전, 재고 1개, 첫 공급 대기 | 설비 허가 수신 여부 | [17:07 · 설비 허가 미수신](../../docs/reports/live-test/20261121-170706.md) |
| 발행자를 추가했는데 첫 공급 대기 | 허가 값과 마지막 수신 후 경과 시간 | [17:30 · 정상 Safety와 거절된 허가](../../docs/reports/live-test/20261121-173021.md) |
| 픽 위치에 자재가 있는데 로봇 대기 | 단계 22의 `confirmed`, `boundSlot`, `admissionAvailable` | [17:46 · Mission 요청 전 대기](../../docs/reports/live-test/20261121-174628.md) |
| 새 장면에서 첫 공급 대기 | PLC의 `phase/cycle/committed`와 Isaac의 `last_generated` | [18:24 · 이전 자재를 기다리는 PLC](../../docs/reports/live-test/20261121-182438.md) |

각 사례에는 로컬 원본의 위치, 당시에 저장한 값과 코드 기준이 있다. 예를 들어 17:30 실행에서 사용한 판정 코드는 저장소 루트에서 다음 명령으로 비교할 수 있다.

```bash
git show 95deac8:ros2_ws/src/plc_adapter/arm_cell_plc_adapter/simulation_permit.py
git diff 95deac8 3cef7b7 -- ros2_ws/src/plc_adapter/arm_cell_plc_adapter/simulation_permit.py
```
