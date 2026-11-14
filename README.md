# SF-Twin ARM Cell

[한국어](README.md) | [English](README.en.md)

Doosan M0609 로봇과 Robotiq 2F-85 그리퍼를 대상으로 하는 시뮬레이터 기반 ROS 2 로봇 셀 프로젝트입니다. 비전, 모션 계획, 작업 오케스트레이션, 소프트웨어 안전 감독, 자재 인계, 운영자 UI를 공통 ROS 인터페이스로 연결합니다.

현재 공식 배포 버전은 v0.3.4입니다. v0.3.4는 Vision/PICK 후보 선택과 PLACE 복구, 실행별 진단 기록, 로컬 D455 자산과 런타임 로그를 개선했습니다. 엔지니어링 및 시뮬레이션 프로젝트이며, 생산용 셀이나 안전 등급을 인증받은 시스템은 아닙니다.

## 주요 기능

- Isaac Sim에서 Doosan M0609·Robotiq 2F-85 로봇 셀과 RGB-D 카메라 실행
- 깊이 영상 기반 직육면체 자재 인식과 MoveIt 2 기반 PICK·PLACE·복귀 동작
- 자재 공급, 수량 기반 작업 흐름, 소프트웨어 Safety 감독
- GUI 런처로 시뮬레이터와 ROS 프로세스 시작·종료
- Isaac Sim UI Hub에서 카메라·Vision 결과 확인과 운영자 요청

외부 AMR/VDA, PLC, 안전 하드웨어 상태는 mock으로 표현합니다. 실물 로봇 운전,
AMR 내비게이션 및 안전 등급 E-stop/STO 하드웨어는 지원 범위에 포함하지 않습니다.

[시연 영상 보기](https://youtu.be/6eOcrNFdf2g)

## 준비 사항

- Linux 호스트: 아래 명령은 Ubuntu 22.04 기준입니다.
- ROS 2 Humble과 Isaac Sim 5.1 및 Isaac Sim을 실행할 수 있는 GPU·드라이버
- GUI 런처: Python 3.10 이상, X11 또는 XWayland 디스플레이
- 호스트 빌드: colcon, 초기화된 rosdep, `nlohmann-json3-dev`
- Docker 실행을 선택하는 경우: Docker Compose v2

```bash
git clone https://github.com/dev-kibeom/sftwin-arm-cell.git
cd sftwin-arm-cell
```

아래 명령은 별도 안내가 없으면 저장소 루트에서 실행합니다.

## ARM Cell 실행 방법

다음 중 한 경로를 선택합니다. GUI 런처는 호스트에 빌드한 ROS workspace를 사용합니다.
Docker로 ROS를 실행하려면 [수동 실행](#수동-실행-호스트-또는-docker)을 사용합니다.
GUI와 수동 실행으로 같은 셀을 동시에 시작하지 마세요.

### GUI 런처로 실행

**1. ROS workspace를 빌드합니다.** 새 터미널에서 실행합니다.

```bash
bash ros2_ws/build-local.bash
```

빌드가 완료되면 `ros2_ws/install/setup.bash`가 생성됩니다.

**2. 런처 의존성을 설치합니다.** Python 의존성은 PySide6, PyYAML, xacro입니다.
설치와 실행에는 같은 Python 환경을 사용합니다.

```bash
sudo apt-get install python3-pip libegl1 libxcb-cursor0 xterm
```

가상환경 사용 여부와 위치는 사용자가 선택합니다. 사용할 경우 Python 의존성을
설치하기 전에 활성화합니다. 다음은 저장소 안의 `.venv`를 사용하는 예시입니다.

```bash
sudo apt-get install python3-venv
python3 -m venv .venv
source .venv/bin/activate
```

```bash
python3 -m pip install -r infra/launcher/requirements.txt
```

기존 설치를 갱신할 때도 같은 requirements 명령을 사용합니다.

**3. 런처를 실행합니다.**

```bash
python3 launcher.py
```

Isaac Sim 설치 위치는 기본 `~/isaac_sim`입니다. 다른 위치나 ROS domain을 사용한다면
실행 시 지정합니다.

```bash
ISAAC_SIM_ROOT=/path/to/isaac_sim ROS_DOMAIN_ID=0 python3 launcher.py
```

**4. 셀을 시작합니다.**

1. Robot·Tool·Cell·Backend를 선택합니다. 현재 지원 조합은 M0609 + Robotiq 2F-85 +
   `isaac_m0609` Cell입니다.
2. `Isaac 시작`을 누릅니다. 씬이 로드되고 Timeline과 After Playing 초기화가 끝날 때까지 기다립니다.
3. `ROS 2 시작`을 누릅니다. 상세 로그가 필요하면 시작 전에 `ROS 2 디버그 모드`를 선택합니다.
4. Isaac Sim UI Hub의 카메라·Vision 화면과 두 실행 터미널의 오류를 확인합니다.
   런처의 실행 표시는 프로세스가 실행 중이라는 뜻입니다. 시스템 준비나 PnP 성공을 보장하지 않습니다.

Isaac과 ROS마다 별도 `xterm` 창이 열립니다. 출력과 오류는 해당 터미널에서 확인합니다.
종료는 런처의 `모두 종료` 또는 창 닫기를 사용합니다. 실행 터미널의 `Ctrl+C`나 창 닫기도
종료로 처리됩니다. 종료 처리가 끝날 때까지 기다려 주세요.

### 수동 실행: 호스트 또는 Docker

**1. ROS 실행 환경을 준비합니다.** 호스트를 사용한다면 먼저 빌드합니다.
Docker를 사용한다면 호스트 workspace 빌드는 필요하지 않습니다.

```bash
bash ros2_ws/build-local.bash
```

**2. 첫 번째 터미널에서 Isaac Sim을 시작합니다.**

```bash
source /opt/ros/humble/setup.bash
./infra/isaac_sim/scripts/isaac_arm_cell.sh
```

설치 경로가 다르면 스크립트 앞에 `ISAAC_SIM_ROOT=/path/to/isaac_sim`을 지정합니다.
씬·Timeline·After Playing 초기화가 완료된 뒤 다음 단계로 진행합니다.

**3. 두 번째 터미널에서 ROS를 시작합니다.** 아래 둘 중 하나를 선택합니다.

호스트 실행:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch arm_cell_bringup arm_cell.launch.py log_level:=info
```

Docker 실행:

```bash
export ARM_CELL_UID="$(id -u)" ARM_CELL_GID="$(id -g)"
docker compose up --build
```

Isaac과 ROS는 같은 `ROS_DOMAIN_ID`와 RMW 설정을 사용해야 합니다. Compose 기본값은
`ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`입니다.
Docker 로그는 `docker compose logs -f arm-cell`로 확인합니다.

종료할 때는 ROS 터미널에서 `Ctrl+C`를 누릅니다. Docker는 Compose 터미널의 `Ctrl+C`
또는 `docker compose down`으로 종료합니다. ROS 종료 후 Isaac을 닫습니다.

## 기본 사용

Isaac Sim의 **UI Hub**는 카메라·Vision 결과와 운영자 조작을 제공합니다.
런처는 프로세스 시작·종료용이며, UI Hub는 시뮬레이터 안에서 사용합니다.

- `External Handoff Trigger`에서 `Material type`과 `Quantity`를 선택하고
  `Random Drop`으로 공급 자극을 발행합니다.
  공급 완료 후 Safety·준비 조건에 따라 작업이 시작됩니다. 버튼의 발행 성공은 공급이나 작업 성공을 뜻하지 않습니다.
- 카메라 영상과 Vision Snapshot에서 관측 결과를 확인합니다. 전체 시스템 상태와 작업 오류는
  ROS 터미널 및 진단 기록에서 확인합니다. UI Hub는 종합 상태 대시보드를 제공하지 않습니다.
- 잠시 멈출 때는 Isaac 표준 Timeline의 Pause를 사용합니다. Play로 같은 실험을 재개합니다.
- 새 실험은 가장 간단하게 런처의 `모두 종료` 후 다시 시작합니다. 같은 Isaac 프로세스를 유지하려면
  ROS 종료 → Timeline Stop → `1_before_play.py` → Play → `2_after_play.py` → 새 ROS 시작 순서를 따릅니다.
  Stop은 시뮬레이션 상태 초기화이며 Safety Reset이나 실장비 비상정지를 대신하지 않습니다.

자세한 조작과 세션 동작은 [UI Hub 안내](docs/design/components/ui-hub/fds.md)와
[시뮬레이션 세션](docs/design/components/simulation/fds-sim-session.md)을 참고하세요.

## 설정과 로그

- GUI는 선택한 설정을 Isaac과 ROS에 함께 적용합니다. 설정 파일을 수정했다면 실행을 종료하고 다시 시작합니다.
- 수동 실행 예시는 기본 운영 프로파일을 사용합니다. Isaac 시작 스크립트가 생성하는
  설정 파일의 기본 경로는 `.run_records/runtime_configuration.json`입니다.
- 자재 정의는 `ros2_ws/src/bringup/config/materials/`에 있습니다. UI Hub 목록은 실행 시 읽은 설정을 사용합니다.
- 진단 기록은 기본 `.run_records/` 아래에 저장됩니다. 런처 터미널 출력은 창이 닫히면 사라집니다.
- 호스트 ROS 상세 로그는 `log_level:=debug`로 켭니다. 기본 `info`는 선택적 진단 수집을 끄고,
  `debug`는 모든 ROS 진단 수집을 켭니다. `diagnostics:=none`, `all`, 또는
  `motion,vision,safety,orchestration` 중 쉼표 구분 목록으로 수집 범위를 선택합니다.

Docker에서는 시작할 때 환경변수로 로그와 진단을 선택합니다. 변경하려면 기존 실행을 종료하고 다시 시작합니다.

```bash
ARM_CELL_LOG_LEVEL=debug docker compose up --build
ARM_CELL_LOG_LEVEL=info ARM_CELL_DIAGNOSTICS=vision,motion docker compose up --build
```

설정 구조·컴포넌트 책임은 [구성요소 아키텍처](docs/design/architecture/components.md),
기록 해석은 [Observability Records ICD](docs/design/interfaces/icd-observability-records.md)를 참고하세요.

## 문제 해결

| 증상 | 확인할 사항 |
|---|---|
| 런처에서 Python 모듈을 찾지 못함 | 런처 실행에 쓰는 Python으로 requirements를 설치했는지 확인합니다. 가상환경을 사용한다면 활성화합니다. |
| `ros2_ws/install/setup.bash`가 없음 | 호스트 빌드를 완료합니다. GUI 런처는 Docker 이미지의 빌드 결과를 사용하지 않습니다. |
| Isaac 실행 파일을 찾지 못함 | `ISAAC_SIM_ROOT`가 `isaac-sim.sh`를 포함한 디렉터리인지 확인합니다. |
| 실행 터미널이 열리지 않음 | `xterm` 설치와 X11/XWayland 디스플레이를 확인합니다. |
| ROS 시작이 차단되거나 이미 실행 중이라고 표시됨 | 기존 런처·CLI·Docker 실행을 먼저 종료합니다. 다른 checkout의 프로세스는 자동 정리하지 않습니다. |
| 영상이나 로봇 상태가 갱신되지 않음 | Timeline이 Play 상태인지, After Playing 초기화가 완료됐는지, Isaac과 ROS의 domain·RMW 설정이 같은지 확인합니다. |
| 코드 수정이 실행에 반영되지 않음 | 호스트는 재빌드 후 overlay를 다시 source하고, Docker는 `docker compose up --build`로 교체합니다. Isaac Python 변경은 runtime 재로드 또는 재시작이 필요합니다. |
| Docker 기록 저장 권한 오류 | Isaac과 Compose를 종료하고 `.run_records/`, `.motion_data/`의 소유자와 `ARM_CELL_UID`·`ARM_CELL_GID`를 확인합니다. |

## 문서 탐색

- [v0.3.4 릴리즈 노트](docs/releases/v0.3.4.ko.md) / [English](docs/releases/v0.3.4.en.md): 변경·업그레이드·검증 범위
- [프로젝트 정의](docs/product/definition.md), [라이브 인수 계약](docs/product/acceptance.md)
- [시스템 아키텍처](docs/design/architecture/), [컴포넌트 설계](docs/design/components/), [인터페이스](docs/design/interfaces/)
- [알고리즘 설계](docs/design/algorithms/)
- [설계 결정과 Engineering Stories](docs/record/)
- [Acceptance 보고서](docs/reports/acceptance/)

## 라이선스 및 제3자 저작물

프로젝트 자체 코드는 [MIT License](LICENSE)를 따릅니다. 로봇 모델과 제3자 에셋에는
각각의 upstream 조건이 적용됩니다. 재배포 전 [Third-Party Notices](THIRD_PARTY_NOTICES.md)와
함께 배포된 라이선스 및 출처 정보를 확인하세요.
