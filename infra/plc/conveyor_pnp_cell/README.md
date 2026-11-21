# Conveyor PnP Cell PLC

런처에서 **PLC 시작 시 CODESYS GUI 열기**를 선택하면 현재 체크아웃의 [Conveyor PnP Cell 프로젝트](Conveyor.fbsdev)가 열린다. 실행 중인 SoftPLC는 마지막으로 배포된 부팅 앱과 보존된 공정 상태를 사용한다.

| 목적 | 열어 볼 파일 | 확인할 내용 |
|---|---|---|
| Conveyor 공정 읽기 | [PLC_PRG.prg.st](Conveyor.fbsdev/Application.iecapp^/PLC_PRG.prg.st) | `CASE phase OF`에서 투입, 이송, 픽 확인, 로봇 요청, 배출 순서와 전이 조건을 따라간다. 공정 앱 식별값은 IR29의 `0xC051`이다. |
| CODESYS 빌드 구성 확인 | [주기 태스크](Conveyor.fbsdev/Application.iecapp^/TaskConfiguration.json), [장치 트리](Conveyor.fbsdev/Devices/DeviceTree.json), [라이브러리 잠금 파일](Conveyor.fbsdev/Application.iecapp^/Libraries.lock.json) | CODESYS가 여는 애플리케이션은 `Application.iecapp`이고, 그 소스와 설정은 짝을 이루는 `Application.iecapp^/`에 있다. |

라이브러리 복원과 PLC/ROS 레지스터 매핑은 [어댑터 안내](../../../ros2_ws/src/plc_adapter/README.md#conveyor-pnp-cell-공정-프로그램), Isaac·PLC·ROS의 기동 순서는 [Conveyor 실행 절차](../../isaac_sim/scripts/conveyor_pnp_cell/README.md#실행)에 있다.

## 실패 사례 PLC

아래 폴더는 각각 독립된 CODESYS 프로젝트다. 원하는 프로젝트를 열고 `Application.iecapp → PLC_PRG`의 `CASE phase OF`를 현재 [공정용 PLC_PRG](Conveyor.fbsdev/Application.iecapp^/PLC_PRG.prg.st)와 비교한다.

| 프로젝트 | 보존한 ST 원본 | 살펴볼 분기 |
|---|---|---|
| [FirstFeedWaitsForRobot.fbsdev](examples/FirstFeedWaitsForRobot.fbsdev) | `3accc53` | 단계 0에서 설비와 재고가 준비돼도 `robotReady=FALSE`이면 첫 자재 번호를 발급하지 않는다. 현재 코드의 `feedAllowed` 및 첫 공급 조건과 비교한다. |
| [RetryOnSequenceChange.fbsdev](examples/RetryOnSequenceChange.fbsdev) | `205b69a` | 단계 40에서 거절 직후 `rejectedSample`을 저장한다. 단계 23에서 픽·슬롯·수락 조건이 유지되면 HR6의 순번만 달라져도 새 요청이 나간다. 현재 코드의 `retryArmed`와 비교한다. |
| [ConveyorLive20261121.fbsdev](examples/ConveyorLive20261121.fbsdev) | `7de4dff` | 11월 21일 라이브 실행의 소스 기준. PLC에 `phase=22, cycle=1, committed=0`이 남고 새 Isaac 장면에서 `generated=0`이면, 단계 22의 `generated = cycle`이 성립하지 않는다. `VAR RETAIN` 선언과 해당 분기를 함께 본다. |

각 예제에는 해당 리비전의 ST를 그대로 담았다. 장치·라이브러리·주기 태스크는 저장소에 보존된 Conveyor 프로젝트 구성(`5608579`)을 사용한다. 마지막 예제에서 중단 당시의 값은 실행 입력이다. 소스만 열면 RETAIN 메모리와 Isaac 장면까지 복원되지는 않는다.
