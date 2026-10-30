# ARM Cell Observability Record Inventory and Design Audit

- **설계 근거:** PR #503의 초기 목록. 현재 Live Test Analysis 설계를 위해 증거 재감사
- **런타임 감사 기준:** `origin/main`의 `3eb67804e8ebb23b830e2591050bd8a1d3cbbf54`(PR #515 통합, PR #511의 Live Test Analysis FDS)
- **범위:** 해당 기준의 구현된 기록 생산자·독자·라이브 세션 연결·보고서 준비 상태
- **소유자:** 컴포넌트 간 분류·연결은 ARM Cell Observability. 각 생산 컴포넌트는 기록 내용과 결과 의미를 계속 소유

이 목록은 관측된 구현을 기록하며 FDS의 목표 계약이 모두 출시됐다고 주장하지 않는다. 아래 경로와 한계는 감사 기준을 설명하며 설정 가능한 경로와 런타임 옵션이 우선한다. 이는 설계 감사이지 마이그레이션 지시가 아니다. 기존 파일은 원래 위치에 남는다. 7절은 당시 기준의 라이브 테스트 보고서 준비 상태를 재감사하며 과거 생산자 Payload의 세부사항은 열거된 소유 계약의 범위로 제한한다.

## 1. Shared classification and ownership

여섯 분류는 필수 파일 형식이나 디렉터리가 아닌 기록의 목적을 나타낸다. 하나의 컴포넌트가 여러 분류의 기록을 생성할 수 있다.

| 분류 | 의미 | 대표 소유자 |
|---|---|---|
| 운영 Log | 사람이 읽는 운영 이벤트·이상·실패 원인 | 이벤트 소유 컴포넌트. 의미는 `internal project reference (not included in this package)` |
| Execution Record | 단일 작업의 주요 단계·식별자·Terminal 결과 | 작업 소유자(Motion·Orchestration·Vision Service 등) |
| Diagnostic Record | 실제 관측·계산·분류·거부·판단 근거 | 관측 또는 결정을 내린 컴포넌트 |
| Visual Artifact | 관측·결과를 표시하는 이미지 등 | 렌더링한 컴포넌트. 소비자는 표시 전용 |
| Replay Evidence | 관측·장면을 재현하는 데 필요한 입력과 설정 | Source 입력과 해석의 소유 컴포넌트 |
| Performance Trace | 유계 시계열 또는 자원 측정값 | 측정 컴포넌트 |

분류 자체가 기록에 실행 상태 권한을 부여하거나 결과를 변경하거나 모든 분류가 항상 활성화된다고 뜻하지 않는다. Log의 의미와 심각도는 로깅 정책이 소유한다. 이 FDS는 컴포넌트 간 분류·연결·시간 기준·수집 독립성·손실·생명주기 원칙을 소유한다. Payload·조건·경로·형식·식별자·소비자·현재 구체적 제한은 각 컴포넌트 FDS가 소유한다.

## 2. Producer and consumer inventory at baseline

`Required`는 기존 계약에서 해당 범위의 기록을 필수로 요구한다는 뜻이다. `Conditional`은 명시된 이벤트나 선택 조건에서만 생성된다는 뜻이다. 대시는 해당 생산자의 영속 파일을 발견하지 못했다는 뜻이다. ROS·Isaac 운영 Log는 각 런타임 Logging Backend를 사용하며 구조화된 결정 기록으로 취급하지 않는다.

| 생산자·기록 | 분류·조건·필수 여부 | 위치·형식·식별자·시간 | 소비자·기존 한계·실패 동작 |
|---|---|---|---|
| ROS 노드·Launch·Runtime, Isaac Kit·스크립트 | 운영 Log. 이벤트 소유자 기준 INFO/WARN/ERROR/DEBUG, Runtime 설정 심각도 | ROS Backend(`~/.ros/log` 또는 `ROS_HOME`), Isaac Kit Log 디렉터리, Text·Backend 고유 필드. 공유 Record ID 없음. Logger·프로세스 경과 Clock은 로컬 | 운영자·개발자가 읽음. 공통 보존 한도는 여기서 지정하지 않음. 출력 실패는 해당 Backend 동작을 따름. Logging 정책은 Tick별 잡음과 기본 기술 UUID 노출을 금지 |
| Vision Ingress 및 `VisionDiagnostic` / `VisionCalibration` / `SceneStability` Topic | 활성 ROS 인터페이스의 Diagnostic Record. 요청·선택에 따름. 요청 Progress Feed는 영속 파일이 아님 | ROS Message, `vision_run_id`, `acquisition_id`, 요청 로컬 Counter, Source Stamp와 프로세스 로컬 Steady 수신·처리값 | Vision·UI Hub가 읽기 전용 진단으로 소비. Ingress Queue·수신 Metadata는 설정 용량으로 제한되며 Replay Payload는 보존하지 않음. 인터페이스 실패·Timeout은 기존 Service Result·Detail로 반환 |
| Vision Terminal Detector Snapshot(`snapshot.png`, `metadata.json`) 및 Motion Occupancy Snapshot | Snapshot PNG는 Visual Artifact이며 Metadata·진단 JSON은 직렬화된 기록 | Vision·Motion PNG는 공통 `snapshots/` 영역에, 진단 JSON 및 Snapshot Metadata JSON은 `diagnostic_records/` 영역에 저장. 이미지와 진단 기록은 동일 세션 및 Acquisition·Calibration·Candidate attempt 등 작업 식별자로 연관할 수 있어야 함 | Vision은 Snapshot 생성·저장·발행을 소유하고 Motion은 Candidate attempt의 Planning Map Snapshot 및 진단 기록을 소유한다. 각 컴포넌트의 기존 비동기 출력과 실패 비간섭성을 유지한다. Vision 출력 실패는 정식 DetectTarget·Calibration·Mission 결과를, Motion Snapshot 실패는 Planning·실행 결과를 변경하지 않는다. 생성 시점·내용·개수는 [Vision Snapshot FDS](vision/fds-diagnostic-snapshot.md) 및 [Motion Execution FDS](motion/fds-motion-execution.md)를 따른다 |
| Vision Calibration 이미지·Metadata 및 Calibration YAML | Calibration 결과는 Visual Artifact. YAML은 운영 Calibration 입력이며 Replay Record가 아님 | 증거는 같은 Snapshot Tree, Runtime은 설정 Calibration 파일 사용. Calibration 식별자·원본 이미지 Metadata | UI Hub는 명시적 Calibration 완료를 표시하고 Vision은 시작 시 선택한 YAML을 로드. 실패한 Calibration은 Service Result가 있지만 성공 Snapshot은 없음. Snapshot 실패는 Calibration 결과를 변경하지 않음 |
| Vision Acquisition별 Diagnostic Record | Diagnostic Record. 항상 생성하는 Terminal Summary와 실제 계산된 Strategy별 값. 단일 관측 필드와 Stability 필드 구분 | 연결 세션: `.local_artifacts/diagnostic_records/<acquisition_id>.json`; 미연결: `.local_artifacts/observability/vision/<acquisition_id>.json`. `vision_run_id`, 기존 `acquisition_id`, 확인된 `live_test_session_id` 또는 명시적 `unassociated` | 기존 유계 Vision Snapshot Worker가 동일 Snapshot 메시지로 비동기 저장·게시. Cuboid는 단계별·Height Patch별 개수·사유와 손실·Capture 상태 기록. Hub는 로컬 경로 없이 진단 필드 표시. 저장 실패·Queue 손실·Writer 부재는 비차단이며 Acquisition ID와 함께 보고 |
| Vision Cuboid Replay Evidence | 수치 Replay 입력. 실패한 Cuboid 시도에서 실제 생성·사용된 Height Grid와 이미지 경계 Mask | 동일 위치의 `<acquisition_id>.height_grid.f64le`(Row-major IEEE-754 Little-endian Double, 무효 Cell은 NaN)과 선택적 `<acquisition_id>.image_boundary.u8`. 치수·Tray Frame·원점·해상도·Profile·Calibration·Source 관측 참조는 Diagnostic Record에 기록 | 적격 실패에서 기존 Grid로만 수집. 기존 4M Cell Grid 한도·Worker Queue로 제한. Snapshot PNG·Occupancy Rendering은 수치 Replay 입력이 아님. 센서를 다시 읽거나 진단용 알고리즘을 재실행하지 않음 |
| Motion PICK Occupancy Snapshot | Visual Artifact. Vision-derived PICK의 Candidate attempt 진단 | 생성·연결·Planning map과의 일치는 [Motion Execution FDS](motion/fds-motion-execution.md)가 소유 | Motion 조사·검증용. 기록 실패는 기존 PICK Result를 변경하지 않음 |
| Vision RGB-D 요청·Acquisition 상세 | Diagnostic Record. 모든 Terminal 요청에 대한 유계 Timing·Ingress 요약. 샘플·Pair 상세는 Vision 상세 Capture 선택 시에만 포함 | Diagnostic Record, 프로세스 로컬 요청 Counter·`acquisition_id`, Source Stamp와 Monotonic 수신·처리값 | Snapshot Worker가 Snapshot Metadata와 독립적으로 Diagnostic Record를 저장·게시. Ingress Callback은 유계 수신 Metadata와 요청 로컬 Pair 상세 유지. Capture 실패·부재는 가능한 경우 Status·Detail에 반영하며 Timeout은 Service 결과를 유지 |
| Motion Planning Record | Diagnostic Record. 선택적 Motion 진행 상세·PICK 후보 Detail·독립 Terminal Summary | 연결 시 세션 `motion/motion_planning_records/`, 아니면 설정 Motion 진단 디렉터리. 호환되는 Goal별 JSON과 `motion_planning_details/<goal>_candidate_NNN.json`, `motion_planning_summaries/<goal>.json`. Goal UUID·전달된 `vision_acquisition_id`·Terminal Result·Insertion 제출 상태 | 후보 Detail은 Goal·Candidate별로 개별 제한·합쳐 저장. Terminal Summary는 별도 유계 Queue·Byte 예산과 처분·단계 개수·Detail 참조를 가짐. 기존 Goal별 Terminal JSON은 간결·호환성 유지. Queue·크기·디렉터리·기록 실패는 Best-effort로 Motion 결과를 변경하지 않음. Summary는 수집 범위와 확인된 누락을 기록 |
| Pick Execution Record | Execution Record. 진단 선택과 독립적으로 현재 Motion VR에서 수락된 PICK의 매 Terminal마다 필수 | `ARM_CELL_PICK_EXECUTION_RECORD_DIR` 또는 `$SFTWIN_PROJECT_ROOT/.local_artifacts/pick_execution_records`(미설정 시 작업 디렉터리 대체), 유계 Background Writer의 JSON, Goal UUID·`execution_id`·Vision `acquisition_id`·Result·Path 참조 | Motion 조사·검증. 불변 Terminal 작업 요약은 간결한 선택 Candidate 요약과 Motion Planning Summary 참조 보존. 후보별 탐색 증거는 Planning Detail에 둠. Snapshot 및 선택적 Capture Record와 연결 유지. Writer 실패는 비차단·보고하며 기존 필수 요구를 약화하지 않음 |
| MotionProfileTrace | 선택적 `diagnostics:=motion`과 PICK 또는 PLACE `WAYPOINT` 전략이 함께 활성화될 때 GO_HOME과 첫 성공 GLOBAL 궤적 Profile, 실패 Capture 후 재시도 | 설정 Motion 진단 폴더의 `/motion_profile_trace`, 유계 CSV 샘플·JSON Manifest, `run_id`·Runtime·Profile 식별자·가능한 Motion Goal UUID, Monotonic 샘플링과 명시된 시간 기준 | Motion 성능 분석. 기존 비동기 Writer, 최대 200,000 샘플, Manifest에 Trace Metadata·Drop 상태 포함. `diagnostics:=none`이거나 두 전략 모두 `PLANNED`이면 비활성. 실패가 Task Result를 변경하지 않음 |
| Motion Clock 진단 Capture·Watchdog | Diagnostic Record 및 Performance Trace. 선택적 Motion 진단, Clock 이벤트·이상과 유계 Watchdog 증거 | `.local_artifacts/observability/<clock-capture-id>/<process-instance>/`, JSONL `events.jsonl` 및 Manifest. 각 Capture 인스턴스가 로컬 ID 생성, 프로세스 로컬 Sequence·Epoch, ROS `/clock` 값·로컬 Steady 수신 시각 | Motion·Clock 조사. 이벤트 전 64샘플 Ring, 256항목 비동기 Queue, 이벤트 후 샘플 구간, 기존 Record·Byte 한도. Drop·기록 실패를 드러내며 Watchdog·제어 결과에 영향 없음. 기존 공통 FDS는 존재하지 않는 프로세스 간 공유 `capture_id`를 잘못 설명했음 |
| Isaac Gripper Capture/Release 및 PhysX Observer | 현재 Runtime은 선택과 무관하게 경량 Capture/Release Writer 실행. 상세 PhysX Observer는 `SFTWIN_DIAGNOSTICS`가 `motion` 또는 `all`일 때만 활성 | `.local_artifacts/capture_diagnostic_records/`, Goal UUID 키 Capture JSON, 로컬 Monotonic Time·Command Generation 키 Release JSON, Simulation Physics Step Sequence·로컬 Monotonic 값·Stage/PhysX 관측 | Isaac Gripper·Motion 조사. Capture Writer 유계 Queue(대기 2개), 이벤트 한도(64), 디렉터리별 한도. Release는 RELEASED·128 Physics Step·다음 OPEN·Stop·Shutdown 시 닫힘. Writer 실패를 보고하나 동작은 유지. Width-only OPEN은 Motion Goal과 연결 없음. 상세 PhysX가 꺼져도 경량 결정·결과 기록은 유지되고 Physics 시계열 값은 사용 불가 |
| Safety 구조화 진단 JSONL | Diagnostic Record. `safety` 선택 시 20ms 평가 Tick이 아닌 Summary 변화와 Reset 요청·결과만 수집 | `.local_artifacts/observability/safety/`, JSONL, 로컬 Sequence·Safety 이벤트·입력 식별자, 가능한 ROS·Source·Steady 수신 시각 | Safety 조사. 구현에서 설정한 유계 Writer, Queue·Record·Directory 한도. Queue Drop·용량·Writer 실패를 Log에 남기고 Safety 결정은 변경하지 않음 |
| Orchestration 구조화 진단 JSONL | 필수 경량 Mission 시작·Terminal Record와 선택적인 `orchestration` 상세. Coordinator 대기·Retry·Cancel·Recovery 및 선택 BT 경계 | 연결 시 세션 `orchestration/`, 아니면 `.local_artifacts/observability/orchestration/`. JSONL, ExecuteCycle ID·Delivery ID·선택적 Target Episode·Motion Goal UUID·로컬 순서·시간 기준 | Orchestration 조사. 유계 Writer와 Queue·Record·Directory 제한. Drop·실패는 보고하고 Mission은 변경하지 않음. 정상 500ms Feedback과 일반 Tick은 제외 |
| Isaac RawPart Pose Recorder | Replay Evidence. Timeline Playing 중 Pose 변경·생성·제거 이벤트. 진단 선택과 독립 | `.local_artifacts/isaac_pose_recordings/<recording-session>.jsonl`, 각 이벤트의 `live_test_session_id`·생산자 Recording `session_id`·Simulation Epoch·Time·World Frame·PhysX Pose 관측 지점. Kit Update 약 200ms 샘플 | `reconstruct_layout`이 장면 복원 시 Simulation Epoch 하나를 선택. 이벤트에는 안정된 Delivery·Object 식별자·치수·질량 포함. 파일별 8MiB와 최근 20 Session. 한도 도달 시 한도 이벤트를 기록하고 자체 비활성. Best-effort이며 도메인 결정에 사용하지 않음 |
| UI Hub Snapshot·Metadata 소비자 | 소비 전용. 로컬 Vision Snapshot 저장이나 Metadata 작성 없음 | 전체 `/vision/detector_snapshot` ROS 이벤트(PNG + Metadata JSON), `acquisition_id`·Snapshot 식별자·Source Header 값 소비 | Viewer는 Vision 제공 필드를 표시하고 최신 수신값 유지. Camera·Diagnostic Topic에서 재구성하거나 Vision 로컬 경로를 읽지 않음. 알 수 없거나 없는 선택 필드는 생략·사용 불가 표시. 렌더 실패는 표시 문제만 초래 |
| Isaac Camera·Setup 검사 및 Camera Snapshot | Visual Artifact·Engineering Diagnostic Record. Runtime Acquisition과 무관한 명시적 Tool 호출 | `.local_artifacts/m0609_cell/...` 또는 명시적 출력 재정의. JSON Snapshot·Stdout 진단 | 개발자·Tool이 Camera Geometry·Depth Helper·Scene 검사에 사용. Setup 관측이지 Vision Replay 데이터나 Runtime 결정 증거가 아님. 기록·검사 실패는 Tool에 표시하며 Cell 결정은 아님 |

## 5. Live test session and activation audit

당시 구현에는 ROS·Isaac 진단을 함께 묶는 공통 라이브 테스트 `session_id`나 세션 루트 서술자가 없었다. 기존 로컬 ID는 각각 자체 생명주기만 식별했다. RawPart Pose `session_id`, Vision `acquisition_id`, Motion Goal UUID·`execution_id`, ExecuteCycle ID, Delivery ID, 컴포넌트 Capture ID는 테스트 세션 식별자를 대체하지 못한다.

| 감사 경계 | 당시 구현 증거 | 목표 대비 차이·필수 구현 경계 |
|---|---|---|
| Isaac Before/After Playing 스크립트 | `1_before_play.py`는 `shared/environment_setup.py`를 로드해 `configure_live_environment` 호출 뒤 `setup_scene.py`, Camera Snapshot, ActionGraph 설정을 실행한다. `2_after_play.py`는 `after_play` 역할로 진입하며 `runtime_workflow.py`는 `bootstrap_gripper.py` 실행 후 UI Hub를 설정한다. 하나의 장기 Kit 프로세스에서 Script Editor 호출은 독립적이다. | 기존 Script Editor 생명주기 Hook을 확장해 Timeline Play 때 세션 하나를 열고 After Playing이 진단 Writer 생성 전에 상세 진단 선택을 적용하도록 한다. 새 ROS Endpoint·Service를 만들지 않는다. |
| Isaac Gripper 진단 | `environment_setup.resolve_diagnostics_selection()`은 Before Playing에서 상속된 `SFTWIN_DIAGNOSTICS`(기본 `none`)를 읽어 정규화한다. After Playing 이후 `GripperRuntimeSession.__init__()`이 이를 읽는다. 선택값이 비어 있지 않으면 `BoundedCaptureDiagnosticWriter`와 `IsaacPhysXObserver`를 둘 다 생성하고, 비어 있으면 둘 다 생성하지 않는다. Writer는 유계·비동기다. | 당시 정상 `none` 운영에는 Capture/Release Writer가 없고 상세 PhysX 활성화와 결합돼 있었다. 경량 이벤트 Writer는 항상 만들고 After Playing 스크립트에서 Bootstrap 전에 선택적 PhysX 상세 진단의 `SFTWIN_DIAGNOSTICS`를 지정·재사용한다. Pause/Resume 동안 선택을 고정하고 새 Play에서 재평가한다. |
| Isaac Stop/Play | `setup_scene.py`는 Gripper·HOME 목표·생성 자재를 Reset하는 Timeline STOP Listener를 등록한다. `ValidationSessionOwner.teardown()`은 STOP 후 Play 재개를 위해 기존 GripperRuntimeSession과 Physics Step 구독을 유지한다. Writer 종료는 일반 STOP이 아닌 Bootstrap 교체의 `runtime_session.shutdown()`에서 수행된다. | Reset을 차단하지 않고 Stop에서 진단 세션을 종료·확정한다. 다음 Play에서는 기존 Kit 생명주기에 필요한 Runtime 객체만 유지하면서 새 세션 Root와 세션 범위 Writer 상태를 연결한다. Pause/Resume에서 Writer를 다시 만들지 않는다. |
| Isaac RawPart Pose 기록 | `RawPartPoseRecorder`는 `.local_artifacts/isaac_pose_recordings`에 UUID 이름의 독립 JSONL 세션을 약 200ms 간격으로 저장하며 한도는 8MiB·20개 파일이다. Replay Reader가 파일을 직접 소비한다. | 기존 파일과 Replay 동작을 유지한다. 새 세션 설명 또는 생산자 Metadata로 각 기록을 라이브 테스트 세션과 연결하며 자체 Recording ID를 `session_id`와 혼동하지 않는다. |
| ROS Bringup 및 산출물 | `arm_cell_operational.launch.py`는 `diagnostics`(기본값은 Log Level에서 유도)를 해석하고 `SFTWIN_PROJECT_ROOT`를 설정하며 컴포넌트별 선택·출력 경로를 전달한다. Isaac Session ID·Path는 받지 않는다. Motion·Safety·Orchestration은 `.local_artifacts/observability` 아래 로컬 경로를 사용하며 Pick Execution은 외부 설정 경로를 사용할 수 있다. | 이미 열린 Isaac 세션에서 ROS Bringup으로 로컬 파일·경로 Handoff만 추가한다. Bringup은 유효한 열린 세션에 합류할 뿐 새 세션을 만들지 않는다. 기존 경로·형식은 유지하고 외부 산출물은 `session.json`에서 찾을 수 있도록 한다. 알 수 없거나 모호한 연결은 명시적으로 비차단 처리한다. |
| 세션 생성과 프로세스 순서 | Isaac과 ROS는 독립적이었다. 당시 문서화된 순서는 Timeline Play 전에 ROS를 시작하고 ID 교환은 없었다. 공통 저장소 Root(`SFTWIN_PROJECT_ROOT` 또는 Launch가 해석한 Checkout Root)에 두 프로세스 모두 접근할 수 있으므로 기존 연동 경계는 로컬 파일이다. | 공유 Artifact Root의 Isaac 소유 `session.json`을 최소 Rendezvous로 우선한다. Isaac은 Play 때 충돌 방지 날짜·시간 폴더를 생성하고 ROS는 After Playing 초기화 뒤 유일한 열린 서술자를 찾아 양쪽 프로세스 참여를 기록한다. 정확한 폴더 충돌 처리와 경로 인자 연결은 구현 선택이다. 시간만으로 소속을 추론하거나 오래된·모호한 폴더에 자동 합류하지 않는다. |

목표 디렉터리는 `private local diagnostic artifact (not included in the public package)`이며 `session.json`과 컴포넌트별 디렉터리를 포함한다. 컴포넌트 Writer는 기존 형식과 소유권을 유지한다. 기존 파일을 이동하지 않고 세션 설명이 실제 경로를 연결하므로 하나의 라이브 테스트를 함께 식별·검토·백업할 수 있다. 세션 연결 또는 기록 실패는 관측 가능하지만 Mission·Motion·Vision·Safety·Isaac Reset·Stop 동작은 변경하지 않는다.

After Playing 스크립트가 필수 설정 경계다. 당시에는 상세 진단 선택을 지정하지 않았고 Before Playing이 상속된 환경 변수를 해석했다. `2_after_play.py`는 `role_entrypoint.run("after_play", globals())`를 호출하며, `runtime_workflow.py`는 이후 `bootstrap_gripper.py`를 실행한다. 이 스크립트가 `GripperRuntimeSession`을 생성하고 생성자는 `SFTWIN_DIAGNOSTICS`를 해석한 뒤 Writer와 PhysX Observer를 생성한다. `2_after_play.py` 시작 부분에서 Role Entrypoint 호출 전에 설정을 지정하면 두 생산자가 생성되기 전에 적용된다. 구현에서는 항상 활성화된 경량 Capture/Release 기록과 선택적인 PhysX Observer를 분리하고, 상세 진단 선택에는 기존 `SFTWIN_DIAGNOSTICS` 문법을 유지해야 한다.

Recommended implementation PR sequence:

1. `runtime_steps/setup_scene.py`의 기존 Timeline 생명주기 처리에서 Play 시 Isaac 소유 세션 디렉터리·`session.json`을 열고 Stop 시 종료하도록 확장한다. 기존 `SFTWIN_PROJECT_ROOT`·Checkout 경로를 재사용한다. 서술자 기록 실패에도 현재 Reset Callback 동작을 보존한다.
2. `2_after_play.py`와 `runtime_steps/bootstrap_gripper.py`에서 Runtime Bootstrap 전에 기존 상세 진단 선택을 지정하고, 모든 세션에서 경량 Gripper Writer를 생성하며 선택된 경우에만 상세 PhysX Observer를 연결한다. Stop·다음 Play에는 세션 범위 Writer 상태만 종료·재연결하며 Pause/Resume에는 상태와 Isaac에 필요한 현재 런타임 객체를 유지한다.
3. `arm_cell_operational.launch.py`에는 Bringup이 이미 열린 Isaac 세션에 합류하고 ID·루트를 컴포넌트 Writer에 전달할 수 있도록 로컬 서술자·경로 입력 또는 Resolver를 추가한다. After Playing이 세션 설명을 노출한 뒤 Bringup을 시작한다. Handoff가 누락·오래됨·모호함·읽기 불가이면 미연결로 기록하며 작업 Gate가 돼서는 안 된다(MUST NOT). ROS Endpoint나 세션 Service를 새로 만들지 않는다.
4. 기존 생산자 경계에서 세션 연결을 추가한다. Snapshot·Occupancy·Replay·MotionProfileTrace 및 외부 설정된 Pick Execution 경로를 유지하고 실제 경로·ID를 `session.json` 또는 동등한 영속 세션 참조에 등록한다. 과거 이력을 변경하거나 이동하지 않는다.

구현 검증에는 하나의 세션에 여러 Delivery·Acquisition·Goal·Cycle 식별자가 존재하는 Play, Pause/Resume 연속성, 저장 실패 중 Stop Writer 종료와 Stage Reset, Stop→Play 후 별도 세션 식별자, 중복 세션 생성 없는 ROS 연결, 프로세스·시작 실패 후 명시적인 미확인 연결 상태가 포함돼야 한다. 또한 `SFTWIN_DIAGNOSTICS=none`에서도 경량 Capture/Release 증거가 존재하고, 상세 PhysX 증거가 Observer 필요 시점 전에 시작되며, 기록·세션 실패가 Mission·Motion·Vision·Safety·Isaac Reset 결과를 바꾸지 않음을 보여야 한다.

## 3. Audit findings: implementation versus target

1. **분류가 암묵적이고 중복됨.** 기존 FDS 절에는 생산자 설정, 기록 Payload 의미, 산출물 생명주기, 작업 요구사항이 섞여 있다. 위 여섯 분류는 공유 Writer·Schema·Directory를 강제하지 않고 기존 목적에 이름을 부여한다.
2. **Vision Snapshot Metadata의 책임 과다.** Source Image 식별자와 표시 Annotation뿐 아니라 Cuboid 후보 생성·거부 세부정보, Occupancy 배열, Stability 요약, 요청 샘플·Pair·Timing 세부정보까지 전달한다. UI 계약은 Metadata 필드를 소비하므로 이동하려면 명시적인 호환성 계획이 필요하다. 현재 Snapshot은 Replay Evidence로 충분하지 않다. 원본 RGB-D와 실제 Cuboid Height Grid·전체 알고리즘 입력은 기본적으로 보존되지 않는다.
3. **후보 0개 증거 계약 미완성.** 기존 Cuboid 결과 데이터는 계산된 생성·거부 값을 보고하지만 후보 객체가 생기기 전에 제거된 후보까지 포함한 생성·Partition·Geometry·Qualification 각 단계의 후보 수량을 별도 영속 기록으로 보증하지 않는다. Snapshot은 표시 산출물이므로 누락 단계 추론을 위해 요구해서는 안 된다.
4. **실패 보존 기능이 균일하지 않음.** 많은 상세 기록은 컴포넌트 진단 선택으로 제어되며 기존 Snapshot과 필수 Pick Execution 출력은 독립적으로 실행되고 유계 Writer가 손실을 보고한다. 일부 생산자에는 유계 사전 이벤트 결정 Buffer나 실패 시 자동 보존 기능이 없다. 설계는 알고리즘 재실행·재샘플링 없이 실패 당시 이미 관측한 결정 증거를 보존해야 한다.
5. **공통 FDS가 프로세스 간 상관관계를 과장함.** 당시 생산자가 컴포넌트 로컬 식별자만 갖는데도 프로세스 간 진단 전용 `capture_id`를 규정했고, Isaac 폭 전용 Release는 Motion Goal과 연결할 수 없다. 실제 인과 연결이 있을 때만 기존 ID를 사용하고 아니면 로컬 식별자와 누락 연결을 명시한다. Clock 값 일치가 상관관계를 입증하지 않는다.
6. **시간 의미가 존재하나 분산됨.** Vision은 이미지 Source Stamp와 프로세스 로컬 수신·처리 시간을, Motion Clock Capture는 수신한 `/clock`과 자체 Steady 수신 Clock을, Isaac은 로컬 Monotonic·Physics Step·Simulation 값을 보유한다. 독립 프로세스의 Steady Clock은 비교할 수 없다. 공통 시간 표는 컴포넌트 간 소유자 역할을 유지하고 개별 기록은 관측자와 시간 기준을 명시한다.
7. **저장 경로와 손실 동작은 Writer마다 다름.** 대부분의 런타임 데이터는 `.local_artifacts` 아래에 있지만 Motion Pick 출력은 다른 위치로 설정할 수 있고 작업 디렉터리 Fallback도 있다. 샘플·Queue·Record·Directory·Session 한계는 공유 디스크 할당량 하나로 볼 수 없다. 공통 계약은 손실·Coverage를 표시하되 정확한 보존 기간과 전체 용량은 시험 운영 이후 검토한다.
8. **소비자 경계는 대체로 명확함.** UI Hub는 게시된 Snapshot을, Isaac Replay는 자체 Pose JSONL을 소비한다. Motion 기록은 Vision·Capture 출력을 참조한다. 주된 호환성 위험은 Viewer Metadata가 별도 Vision Diagnostic Record로 옮기려는 필드를 현재 포함한다는 점이다.
9. **Isaac Capture 활성화가 필수 기록 조건을 위반함.** Motion `VR-MOT-TASK-10`은 매 시도마다 Goal 연결 Capture Diagnostic Record를 요구하지만 당시 `SFTWIN_DIAGNOSTICS=none`에서는 Capture Writer와 PhysX Observer를 모두 생성하지 않았다. 필수 Capture 결정·결과 기록은 항상 활성 상태로 두고 상세 PhysX 샘플링만 선택적으로 하며 관측되지 않은 값은 사용할 수 없다고 표시한다.

## 4. Target contracts and Vision split

### Snapshot Metadata

Snapshot Metadata는 원본 Frame·Pair 및 수집 시각, Frame과 좌표 해석, 이미지·렌더 설정, Annotation, 렌더 이미지 판독에 필요한 범례를 식별한다. Detection 상태와 후보 수량은 보이는 Annotation을 설명하는 데 필요한 범위에서만 유지한다. Metadata는 후보별 거부 이력, 알고리즘 계산, Stability 집계 의미, 요청 처리 Timeline, RGB-D Ingress·동기화 측정을 소유하지 않는다. 단계적 변경 중 게시 Metadata 소비자와의 호환성을 유지한다. UI Hub는 인식 가능한 필드를 계속 표시하고 알 수 없거나 누락된 선택 필드를 사용 불가로 취급한다. 구현 PR에서는 기존 필드를 제거하기 전에 Metadata 소비자를 조사하고 마이그레이션해야 한다.

### Vision Diagnostic Record

각 Acquisition에 대해 Vision 소유 Diagnostic Record는 기존 `acquisition_id`를 사용해 실제 Source 관측을 선택 Strategy의 결정·Terminal 결과와 연결한다. 단일 관측 판단과 다중 관측 Stability 집계를 구분한다. Cuboid는 Height Patch 생성, Contour·Line Fitting, 가설 생성, Geometry 검증, Partitioned/Isolated 분류, 후보 거부·적격성, 최종 순서·선택, 실패 단계·사유에 대해 실제 계산한 값을 기록한다. 최종 후보 목록이 비어 있어도 각 단계 수량을 보고한다. 계산하지 않은 필드는 사유와 함께 `not_computed`로 명시하며 기록을 위해 추가 알고리즘을 실행하지 않는다. Strategy 중립 Envelope 필드는 다른 Detector Strategy에 Cuboid 필드를 강제하지 않는다.

### Visual Artifact and Replay Evidence

기존 Detector·Occupancy 이미지는 Visual Artifact 책임을 유지한다. Height Patch·Contour Overlay 같은 알고리즘 분석 시각화는 별도 Sidecar일 수 있으며 수치 Replay 입력은 아니다. Replay Evidence는 실제 소비한 Height Grid, 좌표계·원점·해상도·치수·Calibration·Profile 출처·식별자, 알고리즘에 필요한 유효성 Mask·Source 관측 참조를 보존할 수 있다. 이미 구성된 입력이어야 하며 나중에 센서를 재샘플링하거나 알고리즘을 재실행한 결과를 원본 증거로 표현해서는 안 된다. 기존 산출물에 무손실 보존된 데이터를 중복하지 않는다. Replay Capture는 정상 운영 중 명시적으로 선택하거나 적격 실패 후 이미 있는 유계 값을 보존한다.

### Activation, preservation, and cost

- DEBUG와 선택적 진단 Capture는 독립적인 설정으로 유지한다.
- 정상 운영에서는 의미 있는 요청·Task의 결정·결과에 대해 경량 Diagnostic Record를 유지하고 현재 필수 Execution Record를 활성화한다. 매 프레임·매 Tick 출력이 아닌 이벤트 수준 Capture다.
- 상세 계산 및 큰 Replay Evidence는 정상 운영에서 명시적 선택에 따라 수집한다. 실패 시 소유자는 해당 시도에서 이미 수집한 관측과 결정을 유계 메모리나 불변 결정 Snapshot에 존재하는 범위만큼 보존한다. 보존되지 않은 데이터를 복원했다고 주장하거나 재샘플링·재실행하지 않는다.
- Capture 및 Serialization은 Vision·Motion·Safety 결정을 차단하거나 변경하지 않는다. 기존 유계·비동기 Writer를 우선한다. 생산자 Snapshot, Queue 대기·복사, 직렬화, 이미지 인코딩, 디스크 I/O 비용을 따로 평가한다. 생산자가 원인을 관측할 수 있는 범위에서 Queue Full, 저장·기록 실패, 불완전한 증거를 구분해 보고한다.
- 기존 Writer 한계는 2절의 관측값을 유지한다. 새 보존 기간·전체 산출물 할당량·Database·Collector·OpenTelemetry SDK·중앙 Bus·공통 Logging Framework는 이 설계에서 도입하지 않는다.

## 5. Identity and time rules

실제 관계를 나타내는 기존 식별자를 재사용한다. Vision `acquisition_id`·`vision_run_id`, Motion Goal UUID·`execution_id`, ExecuteCycle ID, Delivery ID, Capture Reference·Command Generation, MotionProfileTrace `run_id`가 이에 해당한다. 같은 검출 시도의 Snapshot Metadata·Vision Diagnostic Record·Replay Evidence는 `acquisition_id`를 공유한다. 시도 내 여러 관측은 기존 Source Stamp와 Frame 식별자, 필요 시 로컬 순번을 사용한다. 전역 Trace ID나 상관관계 Service를 새로 만들지 않는다. 타임스탬프 일치나 일반 Log로 인과 연결을 추론하지 않는다.

모든 Timestamp는 출처·관측자·목적을 명시한다. Camera·Source Stamp, ROS 노드가 관측한 ROS Time, 로컬 Monotonic 수신·결정 시각, 측정 처리 시간, Isaac Simulation Time·Physics Step을 구분한다. Monotonic 값은 동일 프로세스 인스턴스·Clock Epoch 안에서만 비교한다. 독립 프로세스의 Steady Clock 값을 비교해 지연이나 전체 순서를 도출하지 않는다. 시간 기준을 알 수 없으면 사용할 수 없는 상태로 유지한다.

## 6. Recommended implementation PR sequence

1. **Vision 기록 분리(우선):** 이미 구성된 알고리즘 입력을 사용해 Acquisition 키의 Diagnostic Record와 Replay Evidence Payload를 추가하고 Snapshot Metadata의 범위를 좁히되 마이그레이션 중 UI Hub 호환성을 유지한다. 해당 시도의 Diagnostic Record와 Replay 입력만으로 `candidates=0` 원인을 입증한다. 기존 Snapshot·Occupancy 이미지와 Detection 결과 의미를 보존한다.
2. **정상 경량 기록 활성화와 실패 보존:** 적격 생산자별로 결정·결과 수준 경량 기록과 유계 메모리 보존 Envelope를 추가한다. 첫 Acceptance 사례가 Vision 실패이므로 Vision부터 시작하고 각 컴포넌트가 해당 관측을 소유하는 곳으로만 확장한다. 상세 원본·대형 Capture는 선택 사항으로 유지한다.
3. **기록 간 손실·Coverage·시간 정렬:** 기존 Writer에서 완료·사용 불가·Queue 거부·드롭·기록 실패를 명시적으로 보고하고 시간 기준 Label과 실제 인과 ID를 확인한다. 작업 Result 계약은 변경하지 않는다.
4. **다른 컴포넌트 계약 검토:** Motion·Safety·Orchestration·Isaac Gripper·RawPart Replay 기록을 이 FDS와 대조한다. 현재 결과·필수 기록·경로·설정된 제한을 보존한다. 생산자별 변경은 별도의 유계 PR로 수행한다.

각 구현은 진단 활성·비활성 실행의 결정·결과 동등성을 비교하고 대표 속도에서 생산자 측 복사·직렬화 비용, Queue 점유·Drop, 이미지 인코딩 시간, Writer 지연, 제어·센서 Callback 지연을 측정한다. 포화나 저장 실패가 작업 결과를 변경하지 않고 증거 손실이 드러나는지 확인한다. 정확한 총 보존 기간과 디스크 할당량은 새 체계의 시험 운영 후까지 의도적으로 미룬다.

## 7. Current live-session implementation audit

감사 기준 `3eb67804` 구현에는 세션 생명주기, Vision Diagnostic Record, 유계 보고서 Reader, Stop 시 시작되는 오프라인 보고서 Process가 있었다. 이후 Reader·Process가 제거됐고 라이브 테스트 분석은 사용자 요청 시 수행한다. Simulation Session FDS가 Play·Pause/Resume·Stop을 소유하며 이 절은 당시 구현 사실과 남아 있던 증거 수집 차이를 기록한다.

| 경계 | `3eb67804` 당시 구현 | 라이브 테스트 분석에서의 의미 |
|---|---|---|
| Isaac 세션 생성·종료 | `infra/isaac_sim/scripts/m0609_cell/shared/live_test_session.py`는 UUID 세션, 날짜·시간 디렉터리, `session.json`, 컴포넌트 디렉터리, 시작·종료 필드, 프로세스 식별자, 선택값, Artifact 위치 맵을 생성한다. `setup_scene.py`는 Play 때 시작하고 Stop 때 닫는다. 같은 초의 경로 충돌에는 접미사를 붙인다. | 서술자는 분석 범위를 식별한다. 서술자 종료는 세션 경계일 뿐이며 생산자의 Flush·Coverage 상태가 산출물의 완전성을 별도로 입증해야 한다. |
| Pause/Resume 및 Stop→Play | 기존 세션이 있으면 Timeline Play는 이를 유지하고 Stop은 닫는다. 다음 Play는 새 UUID·Root를 생성한다. Isaac Stop은 생성 자재와 Gripper·Attachment 상태를 Reset한다. | 종료된 서술자 하나씩 분석하며 세션 ID가 다른 산출물을 연결하지 않는다. |
| ROS 소속과 진단 경로 | `arm_cell_operational.launch.py`는 유일하게 읽을 수 있는 열린 Isaac 서술자에만 합류한다. 연결에 성공하면 Motion·Vision·Safety·Orchestration이 세션 ID·경로를 받으며 실패하면 기존 미연결 경로로 대체하고 Bringup을 막지 않는다. Pick Execution 경로 재정의도 유지한다. | 명시적 세션 연결을 재사용한다. 보고서는 서술자 또는 생산자 식별자가 세션과 연결한 파일만 주장할 수 있다. 디렉터리 인접성이나 타임스탬프는 소속 증거가 아니다. |
| Vision | 진단 기록과 Snapshot Metadata는 RGB·Depth Source Stamp·Frame, Acquisition 로컬 후보 식별자, 추정 Pose·치수, 선택·거부 후보 정보를 보존한다. Candidate ID가 Isaac RawPart를 식별하지는 않는다. 당시 세션 증거에서는 Camera Source Stamp가 Isaac Physics Step이나 RawPart 상태 Snapshot과 매핑되지 않았다. | 실제 Source Frame·Candidate 데이터·Source Pair를 재사용한다. 해당 관측 시점의 동일 RawPart Pose·Geometry가 명시적으로 연결되기 전에는 객체 단위 수치 오차를 사용할 수 없다. 200ms RawPart Recorder는 생명주기 참고에는 유용하지만 프레임 판정 기준은 아니다. |
| Motion | 필수 Pick Execution Record와 선택적 Planning Record가 Goal·`execution_id`·Vision Acquisition 연결, Terminal 결과, 최종 측정 관절·Task Accuracy, 계획 TCP 목표를 제공한다. Accuracy는 Motion 프로세스가 측정 Feedback으로 계산하며 별도 Timestamp를 가진 Isaac 측 Terminal 샘플은 아니다. MotionProfileTrace는 선택 사항이고 실제 관절 궤적을 포함하지 않는다. | 기존 Plan·Terminal Accuracy를 원본·기준과 함께 재사용한다. Motion과 Isaac의 직접 Endpoint 비교에는 Goal 및 Simulation Time Epoch와 연결된 유계 Isaac Terminal Joint·TCP 관측이 필요하다. 시간 정렬된 실제 관절 샘플 없이는 전체 경로 오차를 구할 수 없다. |
| Gripper | Capture/Release 진단은 Stage 관절·Prim 관측과 선택적 PhysX Observer 값을 구분한다. PhysX 읽기에는 Step 순서·Physics Step·Delta Time 문맥이 있으며 기록에는 수집한 경우 Goal·Capture Reference, RawPart·TCP 식별자, 동작 전후 상대 관측이 포함된다. | 동일 Source·관측 지점과 일치하는 식별자만 비교한다. 선택된 PhysX 기록은 Capture·관절·상대 Pose 검사를 지원한다. 상세 PhysX가 꺼졌거나 증거가 없거나 손실됐으면 Stage 값에서 추정하지 않고 물리 상태를 `UNAVAILABLE`로 보고한다. |
| Material Supply / RawPart | `MaterialSupplyRun`은 요청·공급·대기 수량의 Terminal 결과를 만들고 기존 Sidecar에서 Log·게시하지만 Terminal 결과는 세션 서술자에 영속 목록으로 기록되지 않는다. RawPart Recorder는 안정된 Delivery·Object 식별자, Simulation Time, 치수, 변경된 PhysX Pose를 약 200ms마다 보존한다(8MiB/파일, 20개). 서술자의 디렉터리는 `unconfirmed`이고 Stop은 Stage 자재를 Reset한다. | 기존 산출물은 샘플링한 최종·생명주기 상태를 보여줄 수 있지만 Stop 이후 정확한 요청·공급 수량 대조를 보증하지 않는다. Stop·Reset 전에 제공자의 Terminal 결과와 정확한 Recorder 파일을 세션 증거로 보존하며 샘플 수를 공급자 Terminal 결과로 사용하지 않는다. |
| Orchestration | 소유자는 `mission_started`를 기록하고 Coordinator 반환 후에만 `mission_outcome`을 기록한다. Delivery·ExecuteCycle 식별자, 요청·처리 수량, 결과 가용성, 정식 Terminal 결과, 관련 실패 상세가 포함된다. 이 경량 생명주기 기록은 선택적 상세 진단과 독립적이다. | 시작 기록만 있고 결과가 없으면 작업 미완료 또는 결과 증거 없음이다. 서술자 종료는 Mission 완료를 뜻하지 않는다. Outcome은 Mission 결과로만 사용하고 Cycle 로컬 처리량을 공급자 공급량과 구분한다. 컴포넌트 이벤트나 객체 수에서 성공 결과를 합성·추론하지 않는다. |
| MotionStatus / Orchestration Action 결과 | Motion은 주기적인 `/motion/state` 게시와 실제 실행·Holding 상태를 소유한다. Orchestration은 제출한 Motion Action마다 요청·수락·Feedback·취소·통신 관측·Terminal 결과를 소유한다. Action 성공 여부를 결정하려고 Action Terminal 결과와 MotionStatus를 비교하지 않는다. | Backend 실행·비활성 관측, Stop·Recovery Ready, Holding, 통신 불확실성에 MotionStatus를 사용한다. 진단 설명에 필요하면 원본 Header Stamp와 Orchestration 수신 경과시간을 보존하되 수신된 Action Terminal 결과를 변경하지 않는다. |
| Safety | 선택적으로 세션 연결된 JSONL은 Safety 전이·입력·상태 관측·Reset·Stop 관련 증거를 수집한다. Motion·Orchestration 기록에서도 후속 상태 변화가 보일 수 있지만 Safety 결정 기록을 대체하지 않는다. | 수집된 Safety 결정과 대응되는 사용 가능한 Motion 관측만 제시한다. 프로세스 로컬 Steady Clock을 비교해 Stop 응답 순서를 추론하지 않는다. 공유 Simulation Time이나 명시적 이벤트 연결이 없으면 시간적 대응을 사용할 수 없다. |
| 사용자 요청 분석 | Codex 또는 다른 분석 작성자가 선택된 종료 서술자와 세션 연결 생산자 기록을 읽고 `report.md`와 지원되는 차트를 `report_assets/`에 작성한다. | 분석은 요청 시 수행하며 Stop·Reset·세션 종료·다음 Play에 참여하지 않는다. Live Test Analysis FDS를 적용하고 기존 경로의 원본 기록을 수정 없이 유지한다. |

### 7.1 Audited comparison boundary and minimal additions

| 비교 | 재사용 가능한 현재 증거 | 정확히 누락된 증거·처리 |
|---|---|---|
| Vision 추정 대 RawPart | 후보 추정값, RGB-D Source Header, 후보 수량, Isaac이 샘플링한 RawPart ID·Pose·치수 | 정확한 Source Frame↔Simulation Time 매핑과 모호하지 않은 후보↔RawPart 연결이 없다. 기존 Camera Source 관측(RGB·Depth Stamp·Frame)을 키로 해당 RawPart Stable ID, PhysX Pose, 치수·형상, Simulation Time·Epoch, 좌표계 출처를 담은 유계 Isaac 관측 Snapshot을 추가한다. 연결 가능한 Source 관측에서만 수집하며 이미지를 복사하거나 인접 Recorder 샘플을 Ground Truth로 취급하지 않는다. 기존 Camera 계약에서 안전한 관측 Callback·Clock 매핑을 노출할 수 없다면 해당 경계가 수립될 때까지 `UNRESOLVED`·`UNAVAILABLE`로 보고한다. |
| Motion Endpoint 대 Isaac | 계획 Target, Motion 측정 최종 Joint·TCP Accuracy, 실행 결과, Goal UUID | 독립적 물리 비교를 위해 Stop Reset 전에 Simulation Time·Epoch, Frame, Goal·Execution 식별자와 연결된 유계 Isaac Terminal Joint State·TCP Pose 하나를 보존한다. 전체 궤적 비교에는 시간 정렬된 실제 Joint가 필요하므로 미룬다. Terminal 한 점에서 경로 오차를 도출하지 않는다. |
| Gripper 결정 대 PhysX | Capture/Release 결과, Stage Attachment 상태, 선택적 PhysX Pair 관측과 Physics Step 문맥 | 사용 가능한 동일 Step의 Capture/Release 관측을 재사용한다. 관측 출처(`USD Stage` 대 `PhysX`), Physics Step, Object Path·ID, Goal·Capture Reference, 생명주기 Phase를 명시해 보존한다. 대표 세션 감사에서 구체적인 누락 비교 시점을 입증하지 않는 한 기본 샘플링 추가를 정당화하지 않는다. |
| 공급·Mission 대 RawPart | Isaac Log·Sidecar의 제공자 결과값, 샘플링한 객체 ID·최종 상태, 선택한 Orchestration 기록의 Mission 결과 | 기존 제공자 Terminal 결과와 Mission Terminal 기록을 명시적 세션 산출물로 만들고 작업별 식별자·결과·수량·소유 시간 기준을 포함한다. Stop 전에 정확한 RawPart Recorder 파일·ID를 세션에 등록한다. 제공자·Mission·관측 객체 수량은 독립적으로 보고한다. 객체만으로 Mission 성공을 재분류하지 않는다. |
| Safety 대 Motion | 선택한 Safety 결정·입력 이벤트와 Motion 정지·상태 관측 | 원본 프로세스·Clock 기준 및 존재하는 명시적 Simulation·Sequence 데이터와 함께 기록을 재사용한다. 연결되지 않은 프로세스 간 순서는 사용할 수 없다. Safety 정책·제어 동작은 이 보고서 변경 범위 밖이다. |

추가 Isaac 관측은 검증 전용·이벤트 유계 기록이다. 기존 Isaac 상태 접근을 통해 일치하는 Camera 관측 및 선택된 Terminal 작업 시점에서 Stop·Reset 전에 수집한다. ROS Topic을 추가하거나 Camera Image·Depth 발행을 변경하거나 런타임 결정을 내리게 하거나 Tick별·고주파 수집을 활성화해서는 안 된다. 기존 Camera 관측 경계에 Source Frame 출처를 연결하는 것을 우선한다. 먼저 Source Header Clock이 Isaac Simulation Epoch에 매핑됨을 증명하고, 그렇지 않으면 병렬 Sensor Pipeline을 추가하지 말고 Vision 비교를 사용 불가로 유지한다. 각 값에 객체 Stable ID·Session ID·Source Stamp·Simulation Time/Epoch·Frame·Geometry·관측 출처를 기록한다. Capture 구현 수락 전 기존 Recorder 경로 대비 Record 수·Byte를 제한하고 Callback 비용·RTF·ROS Bridge 영향을 측정한다.

### 7.2 Report outcome and implementation sequence

보고서의 기본 열람 순서는 **A. 요약**, **B. 실행 Timeline**, **C. Ground Truth 비교**, **D. 실패 분석**, **E. 컴포넌트 진단**, **F. 성능**, **G. 증거 및 한계**다. 요약에는 소유자 기록으로 뒷받침되는 세션·환경, 요청·공급·처리·성공·실패·잔여 수량, Mission 결과, 주요 실패 단계, 비교 Coverage, 증거 충분성을 포함한다. Timeline은 Delivery와 Mission·ExecuteCycle 시도를 묶고 각 생산자 Clock·순서 기준이 허용하는 범위에서만 재시도·복구 순서를 보존한다. 프로세스 간 Steady Timestamp는 전체 순서를 확정하지 않는다. 컴포넌트 세부사항은 원본 파일·이미지를 복제·변경하지 않고 연결한다. 실패 분석은 기존 `CONFIRMED`, `NARROWED`, `INSUFFICIENT EVIDENCE` 수준을 사용해 관측 사실·소유자가 보고한 사유·확정 원인·미해결 가설을 구분한다.

Implement as small sequential outcomes:

1. **현재 감사·설계 PR:** 현행 증거 계약과 유계 추가 사항을 기록하며 런타임·보고서 구현은 변경하지 않는다.
2. **Isaac 증거 PR:** 기존 Camera·Time 경계를 검증한 뒤 연결된 Camera 관측 Snapshot과 Terminal Motion Endpoint 기록만 추가한다. Material Supply·Mission 결과 산출물과 Recorder 소속 관계를 각 생산자 경계에서 정확히 등록한다. 유계 Capture와 결정·RTF·Bridge 동등성을 검증한다.
3. **오프라인 비교·보고서 PR:** 보수적 비교 Helper, `UNAVAILABLE`·`UNRESOLVED` 동작, 보고서 A~G절, 묶은 Timeline, 요약·실패 분석을 추가한다. 기존 Fixture를 재사용하고 원본 산출물을 보존한다.
4. **라이브 검증:** 종료된 복수 Delivery 세션 보고서를 생성·검토하고 Stop·Reset·다음 Play를 시험해 Capture 비용을 측정한다. 대표 종료 세션이 없으면 런타임 검증 미완료를 별도로 보고하고 완료로 주장하지 않는다.
