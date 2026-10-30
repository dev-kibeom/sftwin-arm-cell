# [SF-Twin] ARM Cell Vision Service 및 진단 공유 계약

- **범위:** `Vision DetectTarget ↔ Orchestration; Vision diagnostics → Hub`
- **소유자:** `ARM Cell 공유 인터페이스`

## 1. Endpoint와 요청

| Endpoint | ROS Type | Producer | Consumer |
|---|---|---|---|
| `/vision/detect_target` | `arm_cell_interfaces/srv/DetectTarget` | Vision | Orchestration 및 개발 전용 Hub 관측 Client |
| `/vision/test_scenario/mislocalize_next` | `std_srvs/srv/SetBool` | Vision | Hub의 시뮬레이션·시험용 교란 요청 |
| `/vision/scene_stability` | `arm_cell_interfaces/msg/SceneStability` | Vision | Hub 진단 관측(선택) |
| `/vision/diagnostics` | `arm_cell_interfaces/msg/VisionDiagnostic` | Vision | Hub 읽기 전용 요청·인지 진단 |
| `/vision/calibration_status` | `arm_cell_interfaces/msg/VisionCalibration` | Vision | Hub 관측 및 Orchestration의 읽기 전용 준비 상태 입력 |
| `/orchestration/calibrate_vision` | `std_srvs/srv/Trigger` | Orchestration | Hub의 단일 운영자 Calibration 요청·결과 |
| `/vision/calibrate` | `std_srvs/srv/Trigger` | Vision | Orchestration의 명시적 Calibration 요청 경유 |
| `/vision/detector_snapshot` | `arm_cell_interfaces/msg/VisionDetectorSnapshot` | Vision | Hub의 완전한 읽기 전용 Snapshot |
| `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Camera ingress | Vision 인지 및 선택적 Hub Live Camera 보기 |
| `/vision/tray_aligned_depth/image_raw` | Tray frame 높이 이미지 | Vision | Hub의 선택적 지지면·작업 영역 기준 |

실행 가능한 ROS 타입 선언은 각 `.srv`·`.msg` 코드가 SSOT이며, 이 ICD는 여러 Consumer가 공유할 데이터 의미와 인계 조건을 정의한다.

**Material Candidate**는 한 Acquisition의 최종 STABLE Depth 관측에서 Vision이 Geometry를 검증한 물리적 자재다. `cuboid_top_face`는 SELECTED HIGH·MEDIUM 정상 PICK 적격 후보 전체를 단일 `material_candidates` 목록으로 인계하고 LOW·REJECTED 및 미해결 근거는 진단에 보존한다. **Grasp Pose Candidate**는 Motion이 Material Candidate 하나에 대해 생성한 TCP Pose이며, **Planning Attempt**는 해당 Pose에 대한 IK 및 Path Planning 평가다. Vision은 Material Candidate 검출·관측/Geometry Qualification·단일 목록의 안정적인 열거를 소유한다. Orchestration은 Vision 등급을 재계산하지 않고 전체 목록을 변경 없이 전달하며 Mission Progress/Retry를 소유한다. Task Execution은 전달받은 전체 목록 안에서 Material Candidate 우선순위·후보 전환과 PnP 내 작업 실행을 소유한다. Motion은 선택된 Material Candidate의 Grasp Pose Candidate 생성·Planning Attempt 및 단일 movement 실현을 소유한다.

요청은 `target_id`, 상대 `timeout`, 대소문자를 구분하는 `algorithm` 문자열을 포함한다. Recipe `vision.algorithm`이 알고리즘 선택의 SSOT이고, 생략 시 기본 선택은 현재 Recipe/Service 계약에서 정한다. 지원 가능한 알고리즘 집합과 실행 설정은 Vision의 운영 Profile 및 연결된 Algorithm 설계가 소유하며 이 ICD는 특정 알고리즘으로 선택 범위를 제한하지 않는다. 명시적으로 비어 있거나 미지원인 값은 Acquisition 수락 전에 `DETECT_RESULT_INVALID_RESULT`로 거부한다. Orchestration은 일반 요청·Retry·새 Acquisition·PLACE Lookahead에 선택을 전달하고 Vision은 Acquisition 안에서 다른 Algorithm Strategy로 재선택·혼합·실패 Fallback을 수행하지 않는다. 선택 Strategy 내부 복구 절차의 허용 여부는 해당 기능 FDS가 소유한다. 이 공유 계약은 새 알고리즘을 추가해도 변경할 필요가 없으며 실행 가능한 Service 기본값과 생성된 ROS 타입은 코드 선언을 따른다.

Development Hub는 Recipe를 소유하지 않으며 현재 Service/개발 설정의 기본 알고리즘 선택을 사용한다. Fixed Validation의 Fixed Target 계약과 지원 여부는 해당 검증 Adapter의 독립된 기능 계약을 따르며, 개발용 선택이 Production 알고리즘의 지원 범위를 변경하지는 않는다.

`/vision/detect_target`는 요청 하나의 전체 Material Acquisition 결과를 전달한다. `SceneStability.msg`와 `/vision/scene_stability`는 Hub용 진단 관측이며 Orchestration의 별도 Acquisition Gate가 아니다. 기존 `/vision/scene_stability_active` Endpoint는 현재 코드 선언·소비처가 없어 정식 계약에서 제외한다.

`/vision/test_scenario/mislocalize_next`는 기존 ROS `use_sim_time` 설정이 `true`일 때만 사용할 수 있다. Vision 소유 Simulation/Test Stimulus Endpoint이며 DetectTarget Control이나 Production Perception Adapter가 아니다. `data=true`는 Arm Request 수락 이후 새로 Admission된 Acquisition 중 첫 성공 DetectTarget Result를 대상으로 한다. 그때 이미 활성화된 Acquisition은 대상이 아니다. Pose Perturbation은 [DetectTarget FDS](../components/vision/fds-detect-target.md)의 Owner 정의를 따른다. Arm Pending 중 중복 요청은 거부한다. `data=false`는 Pending Stimulus를 해제하며 이미 소비된 경우에도 No-op Success다. 응답은 Arm/Disarm 요청 수락 여부만 보고한다. 적격 Result의 실제 소비 여부 및 시점은 Vision Diagnostic이 보고한다. 이 Endpoint는 DetectTarget Request·TTL·Acquisition Identity·Frame·Calibration·Detector Algorithm 계약을 바꾸지 않는다.

## 1.1 Calibration 상태 및 명시적 요청·결과

`/vision/calibration_status`는 기존의 정규 Calibration Readiness Surface다. 상태 의미는 다음과 같다.

| State | 의미 |
|---|---|
| `CALIBRATING=0` | Vision이 명시적인 운영자 Calibration 요청 하나를 실행 중이다. Startup 또는 정상 Production 작업이 아니다. |
| `READY=1` | `vision.calibration_file`이 선택한 파일을 성공적으로 로드했고 현재 Camera 설정과 호환된다. Live Depth Freshness를 보증하지는 않는다. |
| `NEEDS_CALIBRATION=2` | 선택 파일이 없거나 읽을 수 없거나 무효하거나 비호환이다. `diagnostic_detail`은 원인을 설명하며 Fallback·자동 Calibration은 없다. |

Vision은 기존 1초 Freshness 한계 안에서 현재 Readiness 상태를 반복 발행한다. 이는 Status/Liveness만 나타내며 Sensor Observation이나 Geometry Verification을 유발하지 않는다. `tray_from_camera`는 선택 파일이 로드된 `READY`에서만 채우고 그 외에는 비운다. `accepted_frames`·`required_frames`·`median_depth_residual_m`는 활성 명시적 Calibration 요청만 설명한다. `diagnostic_detail`은 운영자용 문구이지 Machine-parsed Reason Code가 아니다. 필수 Depth 누락/Stale 또는 비호환 CameraInfo는 DetectTarget의 Sensor Input Failure이며 단독으로 Calibration Readiness를 바꾸지 않는다. 현재 제공된 CameraInfo가 선택 파일과 충돌하면 `NEEDS_CALIBRATION`이 된다.

Calibration 요청·Mission 수락 여부·Vision 작업 슬롯 경합·RGB-D 측정·파일 산출·운영자 선택·재시작의 **시간적 처리 절차**는 [Installation Calibration FDS](../components/vision/fds-installation-calibration.md)가 소유한다. 이 ICD는 `/orchestration/calibrate_vision`, `/vision/calibrate`의 요청/응답과 상태값·산출 파일 경로의 공유 의미를 정의한다. 성공은 새로운 파일 생성이고 선택된 파일의 활성화나 READY를 뜻하지 않는다. 별도 자동 보정·승인·장기 작업 요청 인터페이스는 없다.

## 1.2 Calibration 상태와 Mission의 소비

`READY`는 Vision이 선택된 Calibration 파일을 로드했고 현재 Camera 설정과 호환됨을 뜻한다. 이는 Live Depth Freshness나 현재 Target 관측의 유효성을 보장하지 않는다. Orchestration이 이 상태로 Mission을 승인·보류·재평가하는 조건과 절차는 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다.

DetectTarget Service 경합으로 반환되는 `DETECT_RESULT_BUSY`는 새 요청이 기존 Vision 작업 슬롯을 얻지 못했다는 공유 결과다. 빈 `acquisition_id`는 해당 거부에서 새 Acquisition이 생성되지 않았음을 뜻한다. BUSY는 활성 요청의 취소·교체·재시작이나 Vision/PICK Retry를 뜻하지 않는다. `VisionDiagnostic.state=IDLE`은 발행 시점에 활성 DetectTarget 요청이 없음을 뜻하며, 최신 Terminal Observation의 `valid`·`result_code`와 공존할 수 있다. `DETECTING`은 요청 처리 중 상태이고 Terminal 결과를 뜻하지 않는다. Message에는 Generation 필드가 없으므로 순서 비교용 Generation은 Consumer가 수신 경계에서 관리하는 로컬 순번이다. Mission의 IDLE 대기·요청 재진입 절차는 Mission Cycle FDS, Vision 측 원자적 수락·Result 생성은 DetectTarget FDS가 소유한다.

`VisionDiagnostic`은 Acquisition의 처리 상태·결과·Source RGB Header·Support 영역·진단 문구를 전달한다. 대표 후보의 Centroid·Object Region·Yaw Availability는 제공하지 않으며 후보별 관측 근거는 Snapshot과 Diagnostic Record에 남는다.

## 1.3 Vision Detector Snapshot 이벤트 계약

기존 `VisionDiagnostic`·`VisionCalibration`·선택적 `SceneStability`는 요청·Calibration·Stability 진단 계약을 유지한다. 렌더링된 이미지와 일치하는 전체 Metadata가 없으므로 Hub가 RGB-D를 매칭해 Vision 출력을 조립하도록 해서는 안 된다. Hub 내부 `DetectionSnapshot`은 ROS Interface가 아니다. 새로운 최소 `VisionDetectorSnapshot` 메시지와 `/vision/detector_snapshot` Topic은 Vision이 완성한 Snapshot을 전달하며 새로운 Service 또는 Processing Request를 추가하지 않는다.

`VisionDetectorSnapshot.msg`의 필드는 다음과 같다.

| 필드 | 의미 |
| --- | --- |
| `snapshot_id` | 변경할 수 없는 고유 Snapshot Identity |
| `revision` | Vision Run 내부에서 증가하는 Revision |
| `event_kind` | `0=CALIBRATION_COMPLETE`, `1=DETECT_TARGET_PROGRESS`, `2=DETECT_TARGET_SUCCESS`, `3=DETECT_TARGET_FAILURE`; Calibration Progress Snapshot 없음 |
| `vision_run_id` | Vision Process Run Identity |
| `calibration_id` | Calibration Request의 Diagnostic Identity; 다른 Event에서는 비움. 승인·활성화 권한 없음 |
| `acquisition_id` | DetectTarget Acquisition Identity; Calibration에서는 비움 |
| `image_png` | 완성된 렌더링 Image, `format=png`; `data` 길이 0이면 이미지 생성/인코딩 불가 |
| `metadata_json` | Snapshot·Source Observation Identity/Time, Image/Coordinate 해석, 보이는 Annotation 의미 |
| `diagnostic_record_json` | Terminal DetectTarget의 Acquisition별 독립 Diagnostic Record. Progress/Calibration에서는 비움. Hub는 Snapshot Topic으로만 소비하며 Vision 로컬 경로를 읽지 않음 |
Snapshot Message의 Payload·Metadata와 소비자가 해석하는 보장·비보장은 아래와 같다. 생성·렌더링·저장·발행 및 실패 처리는 [Diagnostic Snapshot FDS](../components/vision/fds-diagnostic-snapshot.md)가 소유한다.

| 공유 항목 | 보장 및 비보장 |
|---|---|
| Metadata Schema | 현재 Producer Version은 `19`. Consumer는 제공된 Schema Version을 기준으로 선택적 필드의 존재 여부를 해석한다. 이전 버전의 파일은 재작성하거나 Backfill하지 않는다 |
| `metadata_json` | Snapshot/Acquisition 및 실제 관측 Source Identity·시각, terminal `scene_generation`, Annotation 및 Image/Coordinate 해석을 전달한다. 교란 중심의 이미지 투영 상태는 Candidate ID·Category·RGB Pixel 또는 `unavailable`로 연결한다. Per-candidate Algorithm Reasoning·상세 Geometry·Request Timing·Stability Aggregate는 `diagnostic_record_json`의 책임이다 |
| `diagnostic_record_json` | Terminal DetectTarget Event의 독립적인 전체 진단 근거를 전달한다. 현재 Diagnostic Record Schema Version은 `2`다. Acquisition 공통 결과와 `single_observation.material_candidates`의 후보별 Identity·Pose·Geometry·Confidence를 구분하고 대표 Target을 투영하지 않는다. Profile-specific Detail은 선택적이며 계산되지 않은 값은 생략한다. Hub는 이를 읽기 전용으로 소비하며 후보를 재계산하지 않는다 |
| `cuboid_stage_images` / `cuboid_pipeline_stage_images` | 선택적 Stage Image Manifest다. 현재 Sidecar Key는 `height_patches`·`geometry_candidates`·`final_candidates`; 개별 Stage는 `height_patch_extraction`·`contour_line_extraction`·`rectangle_hypothesis_generation`·`geometry_validation_resolution`·`candidate_eligibility_enumeration`으로 구분한다. Entry는 Status·Observation-local ID·저장에 성공한 경우만 Relative Path를 가진다 |
| Stage Image 근거 | 동일 Acquisition·Source Depth Observation 및 사용 가능한 호환 RGB Header에 연관된다. Stage Image 누락은 Empty Success가 아닌 Unavailable이다. 중간/거부 Hypothesis는 Detection Result로 간주하지 않으며 완전한 Reject Evidence는 Diagnostic Record가 보존한다 |
| Candidate Identity | `PickCandidate.candidate_id`는 해당 `acquisition_id` 안에서만 고유하다. 목록은 Vision의 Stable Enumeration이며 Grasp Priority가 아니다. 정상 Mission의 단일 대표 Target은 없으며 Candidate ID·Pose·Geometry는 각 목록 항목에만 속한다 |
| Observation Evidence | Terminal Success는 Qualified Material Candidate 전체 목록을 보존한다. Terminal Failure는 Result·마지막 실제 관측 Stage·대표 Source Identity를 전달한다. RawPart 전용 Geometry/Quality 및 Reject Detail은 선택적 Profile-owned Diagnostic Object에 속한다 |
| 선택적 Quality Detail | 측정 Side·Nominal Residual·`orientation_symmetry_order`·유효한 Observed Yaw·실제 계산한 Inlier/Coverage/Edge-support·`max_normalized_residual`·Match/Reject Reason 등을 포함할 수 있다. 미계산 값은 임의 추정하지 않는다 |
| RGB 및 Depth | Snapshot RGB는 Detector 완료 후 독립 선택된 시각화 입력이다. 사용 여부와 미사용 Reason, 선택 RGB Source Header 및 처리 Depth Source Header를 기록한다. 두 Timestamp의 차이는 Acceptance 조건이 아니다 |
| `snapshot_directory` | 로컬 저장을 시도한 Snapshot 이미지 디렉터리의 선택적 절대 경로. Stage Image의 Relative Path는 이 디렉터리를 기준으로 해석한다. Metadata JSON은 별도 Diagnostic Record 영역에 저장한다. 필드 부재 또는 저장 실패는 정식 DetectTarget 결과에 영향을 주지 않으며, 경로 존재가 저장 성공 또는 운영 결과를 보장하지 않는다 |
| `snapshot_style` | 현재 RGB Annotation의 실제 Style을 전달하는 Presentation Metadata로 Detector 또는 Service Result를 변경하지 않는다. Confidence별 시각 표현은 [Diagnostic Snapshot FDS](../components/vision/fds-diagnostic-snapshot.md)가 소유하며, 이 필드는 실제 적용 Style을 전달한다. Disturbance Marker 색상은 Vision 운영 Profile의 `snapshot_style.disturbance`에서 설정한다. Priority 기반 강조·Rank 표시는 금지한다 |
| Calibration Snapshot | `CALIBRATION_COMPLETE`는 새 결과 파일 생성 의미이며 선택 파일 활성화 또는 `READY`가 아니다. 선택적 `calibration_summary`는 실제 Input·Validation Evidence·Transform·Success Reason을 전달한다. 미계산 값은 생략한다 |
| Diagnostic 비간섭 | Topic/Content는 Read-only이며 Safety·Motion·Orchestration·DetectTarget Result·Mission Decision의 Admission 입력이 아니다 |
| 전송 | Reliable/Transient-local 및 Keep-last-one으로 최신 Complete Snapshot을 전달한다. Hub 부재에도 Vision의 발행 계약은 유효하며 Hub는 다음 Snapshot이 올 때까지 마지막 수신 내용을 보존한다 |

### VR-ICD-VIS-ORCH-18 — Snapshot Payload와 Source 상관관계
Snapshot은 고유 Identity, 해당 Event에 맞는 Acquisition/Calibration Identity, 실제 Source Observation Provenance 및 Image/Metadata의 의미상 대응 관계를 보존해야 한다(SHALL). Snapshot Payload는 생성·발행 후 변경되지 않으며, 관측 부재·Image Encoding 불가·선택적 필드 미계산은 성공 근거를 꾸며내지 않고 명시적으로 해석한다. 저장·발행 실패의 처리 순서는 Diagnostic Snapshot FDS가 소유한다.

### VR-ICD-VIS-ORCH-19 — Snapshot의 읽기 전용 의미
Snapshot 및 관련 진단은 읽기 전용 근거다. 이를 Safety·Motion·Orchestration·DetectTarget 결과 또는 Mission Admission의 입력으로 재해석해서는 안 된다(SHALL NOT). Subscriber 부재와 Snapshot Lifecycle의 비간섭성은 [Diagnostic Snapshot FDS](../components/vision/fds-diagnostic-snapshot.md)의 `VR-VIS-DETECT-17~18`이 소유한다.

### VR-ICD-VIS-ORCH-20 — Acquisition evidence correlation
DetectTarget Request마다 비어 있지 않은 불변 `acquisition_id` 하나를 부여해야 한다. Terminal Service Response와 Vision Snapshot Metadata는 같은 ID를 가져야 한다. Orchestration은 진단 상관관계를 위해서만 해당 PICK Goal에 ID를 변경 없이 전달해야 하며 어떤 참여자도 이를 Freshness·Acceptance·Retry·Execution Gate로 사용해서는 안 된다.

### 개발 전용 Retry Detection 공유 경계

Hub의 Development Retry Detection도 기존 `/vision/detect_target`을 사용하며 Vision Result·Snapshot은 새 `acquisition_id`에 연결한다. 이 읽기 전용 요청은 Delivery·Mission을 만들거나 Motion·Gripper·Material Supply를 호출하지 않는다. 동일 Acquisition이 점유 중이면 `BUSY` 의미를 따르며 진행 중 요청을 취소·교체하지 않는다. Hub의 버튼 활성 조건, Caller Timeout 및 지연 응답 처리는 [UI Hub FDS](../components/ui-hub/fds.md)의 `VR-INT-UI-07`을, Mission의 BUSY 대기와 Calibration 준비 판정은 Mission Cycle FDS의 `VR-ORCH-MISSION-24` 및 `VR-ORCH-MISSION-25`를 따른다.

### VR-ICD-VIS-ORCH-25 — Calibration 결과의 공유 의미
`/orchestration/calibrate_vision`의 Terminal `success/message`는 새로운 Calibration 산출 파일의 생성 결과와 경로를 나타낸다. 성공은 선택된 `vision.calibration_file`을 변경·활성화하거나 `/vision/calibration_status`를 `READY`로 만드는 뜻이 아니다. 요청·거부·생성 순서는 [Installation Calibration FDS](../components/vision/fds-installation-calibration.md)가 소유한다.


## 1.4 요청별 Stable Material Acquisition 공유 경계

요청과 응답은 하나의 `acquisition_id`로 상관관계를 가진다. `target_id`, `algorithm`, 상대 `timeout`, 결과 코드, Source Identity 및 단일 Material Candidate List의 **공유 필드 의미**는 이 ICD에서 소유하며, 필드 유효성·해석·비보장은 후술하는 응답·관측·인계 계약을 따른다. Cuboid에서 Vision은 내부 평가한 SELECTED HIGH·MEDIUM 정상 PICK 적격 후보 전체를 하나의 안정된 `material_candidates` 목록으로 반환하고 LOW·REJECTED는 일반 PICK 응답에서 제외한다. 등급은 Action Candidate 필드가 아니며 실행 우선순위도 아니다. 반환 후보 Geometry와 관측 provenance는 인계 중 불변이다. Orchestration은 전체 후보를 변경 없이 전달한다. Task Execution은 전체 목록의 후보 우선순위·전환을 소유하고 Motion은 선택된 후보의 Grasp Pose·계획·movement를 수행한다. Confidence 의미와 진단 근거는 RawPart FDS·Algorithm 및 Vision Snapshot 진단 책임에 따른다.

요청의 수락/경쟁, 단일 Monotonic Deadline, Fresh Depth 관측과 Scene Stability, Detector의 1회 실행, 부재 판단, 실패·중단 및 관측의 종료 시점은 [DetectTarget FDS](../components/vision/fds-detect-target.md), [Frame Ingress FDS](../components/vision/fds-frame-ingress.md), [RawPart FDS](../components/vision/fds-rawpart-depth-perception.md)가 소유한다. 이 ICD는 그 동작을 별도 Gate로 중복 규정하지 않는다.

Legacy `scene_generation`, `SceneStability.msg`, `/vision/scene_stability`는 호환성과 읽기 전용 진단을 위한 공유 식별자·상태 Surface이며 Orchestration의 추가 수락 Gate가 아니다. 공유 Payload가 누락되거나 무효하면 이를 정상 부재 또는 확인된 Free Space로 해석하지 않는다.

## 2. 응답 및 관측 Pose

| Response Field | 의미 |
| --- | --- |
| `result_code` | 정규 결과 |
| `diagnostic_detail` | Acquisition 전체의 비규범 진단 문구 |
| `scene_generation` | 진단용 Legacy Vision Acquisition Identity이며 Orchestration Acceptance Gate가 아님 |
| `acquisition_id` | Vision Snapshot 및 하위 PICK Evidence를 연관시키는 요청 범위의 불변 Identity |
| `material_candidates` | 동일 Acquisition의 SELECTED HIGH·MEDIUM 정상 PICK 적격 후보 전체. 안정적 열거 순서이며 PICK 우선순위가 아님 |
| `disturbance_candidates` | 동일 관측의 미해결 영역 근거. 정상 PICK 적격 후보가 아님 |

각 `PickCandidate`는 Acquisition-local `candidate_id`, 관측 Frame·Timestamp가 있는 `target_pose`, Orientation Symmetry Order, 자체 두께·Rectangle 치수와 실측 여부를 보존한다. 정상 Mission에는 단일 대표 Target Pose·ID·Geometry 투영이 존재하지 않는다. 직접 지정 PICK Pose, PLACE Desired Object Pose, 개별 Motion 목표, Task Execution이 선택한 실행 후보 Pose 및 Calibration 측정 Pose는 별도 기능 계약을 유지한다.

| 후보별 필드 | 공유 의미 |
| --- | --- |
| `candidate_id` | 해당 Acquisition 안에서 고유하며 비어 있지 않은 관측 후보 ID. 영속 객체 Identity가 아님 |
| `target_pose` | 관측 상판의 Object Reference. `base_link`와 원본 Depth 관측 Timestamp를 보존하며 실행 TCP Pose가 아님 |
| `orientation_symmetry_order` | `0=UNOBSERVABLE`, `1=UNIQUE`, `2=TWOFOLD`, `4=FOURFOLD`; 해당 후보의 `target_pose.orientation`을 지배하는 Class |
| `estimated_height_m` / `has_estimated_height` | 보이는 상판 법선 방향 자체 두께. 측정 또는 형상 근거 추론이며 Rectangle 성공 후보에서 필수·유한·양수. Tray 위 상판 높이가 아님 |
| `has_observed_rectangle` | 상판 변 길이의 외곽선 실측 여부. L자 복원의 Recipe 추정 치수 및 Rectangle Geometry 미제공 Profile에는 거짓 |
| `rectangle_long_side_m` / `rectangle_short_side_m` | 상판의 직교 변 길이(m). 실측 또는 복원 추정값인 경우 유한 양수 쌍이며 실측 여부는 별도 provenance로 구별 |
| `has_non_target_occupancy` / `non_target_occupancy` | Legacy wire field이며 Vision-derived PICK의 Planning 의미가 없다. 해당 경로의 소비자는 값과 배열을 무시해야 하며, False/Empty는 이웃 점유가 없다는 근거가 아니다 |


`DETECT_RESULT_SUCCESS`일 때 SELECTED HIGH·MEDIUM 전체를 단일 `material_candidates` 목록으로 전달하고 목록은 비어 있지 않아야 한다. LOW·UNRESOLVED·ALTERNATIVE·REJECTED는 일반 PICK 목록에 넣지 않는다. 유효한 최종 관측에서 개별 Geometry가 해결되지 않은 경우 기존 `DETECT_RESULT_GEOMETRY_ERROR`를 유지하며, 해당 관측의 신뢰 가능한 미해결 영역 근거가 있으면 별도 `disturbance_candidates`를 함께 전달할 수 있다. `DETECT_RESULT_TEMPORARY_INVALID_TARGET`은 일시적인 관측 Evidence 부족에 사용하며 정상적인 미해결 복합 Geometry와 구별한다. `DETECT_RESULT_OBJECT_NOT_FOUND`는 기존 Valid Absence Coverage와 Support Geometry가 부재를 확인할 때만 반환한다. 단일 목록은 안정된 Detector Enumeration, 같은 Acquisition ID, 상호 고유·비어 있지 않은 Candidate ID를 유지해야 한다. Confidence 등급 및 판정 근거는 Action Candidate Payload에 추가하지 않는다. Consumer는 `material_candidates` 전체만을 정상 PICK 후보 출력으로 사용한다.

각 `PickCandidate.top_height_tolerance_m`에는 해당 Acquisition에 적용한 Vision Profile의 기존 Observed Height 비교 Tolerance가 기록되며 모든 Candidate에 같은 값이 변경 없이 반복된다. Cuboid는 설정된 `cuboid.height_tolerance_m`를 제공하고 Observed Height Tolerance 계약이 없는 Detector는 0을 제공한다. 이 필드는 Vision의 관측 메타데이터로 유지되며 Task Execution의 PICK 후보 우선순위에는 사용되지 않는다. PICK 순서는 기존 `base_link` Pose의 Tray Y, 관측 상판 Z, Candidate ID로 결정한다.

Producer별 필수 출력은 RawPart FDS가 소유한다. Vision은 Stable 단일 목록 Enumeration을 구성하고 Task Execution은 전달된 Material Candidate List의 Priority와 Transition을 소유하며 Orchestration은 전체 목록을 변경하지 않고 전달한다. Motion은 선택된 Material Candidate에 대한 Grasp Pose Candidate 생성·계획·개별 movement를 소유한다. 성공 외 결과에는 일반 PICK `material_candidates`가 비어 있어야 한다. 유효 관측에서 미해결 Geometry가 생긴 경우 기존 `disturbance_candidates`는 별도 관측 근거 Payload로 보존할 수 있으며 이를 PICK Success로 해석하지 않는다. Service는 수치 Clearance Score를 노출해서는 안 된다.

Vision–Orchestration 경계는 검출된 Candidate의 관측 Pose·Geometry·Acquisition provenance를 전달한다. LOW·미해결 가설 및 `disturbance_candidates`는 별도 진단/교란 근거이며 Material Candidate Geometry로 승격되지 않는다.

Orchestration은 Vision 등급을 재계산하거나 개별 Geometry를 재판정하지 않는다. 단일 `material_candidates` 전체를 검증한 뒤 변경 없이 기존 `ExecutePnp.material_candidates`에 담아 한 Action으로 전달한다. Confidence 또는 위치로 재정렬하거나 개별 후보를 선택하지 않는다. Task Execution은 전체 목록의 PICK 우선순위·후보 전환을 소유하며 [Task Execution FDS](../components/task-execution/fds-pnp-execution.md)의 기존 거리 우선 정책을 적용한다. Confidence는 PICK 비교 기준이 아니다. Task Execution은 Motion에 선택된 후보의 Grasp Pose 생성·계획·movement를 요청한다. 별도 Vision Candidate Service는 없다. PLACE Lookahead Result는 PLACE 성공 및 Fresh `RELEASED` 전까지 격리 상태를 유지한다.

### 교란 후보의 공유 데이터

`DetectTarget`은 정상 PICK 후보와 별개의 `disturbance_candidates`를 같은 Acquisition에서 반환할 수 있다. 교란 후보는 PICK 적격성이 아니며 기존 PICK 적격성·Depletion·Freshness 판정을 바꾸지 않는다. 동일 높이의 복합 연결 외곽선에서 개별 배치·형상을 확정하지 못한 관측 영역은 기존 `OTHER_UNRESOLVED` 범주로 전달할 수 있다. 교란 후보는 HIGH/MEDIUM로 PICK에 인계한 객체를 중복 대상으로 삼지 않는다. Orchestration은 HIGH/MEDIUM 후보가 모두 없고 기존 미션 조건도 충족할 때에만 유효 교란 후보를 사용한다. `DETECT_RESULT_SUCCESS`는 HIGH 또는 MEDIUM 후보가 있을 때 적용하며, 유효 최종 관측에서 미해결 Geometry와 점유 근거가 남으면 기존 `DETECT_RESULT_GEOMETRY_ERROR`와 disturbance 의미를 적용한다. `DETECT_RESULT_TEMPORARY_INVALID_TARGET`은 일시적인 관측 Evidence 부족에 한정한다. `OBJECT_NOT_FOUND`는 관측으로 확인된 부재이므로 교란 후보와 함께 반환하지 않는다. Sensor·TF·Calibration·Stale·Timeout 오류에는 관측 근거 없는 교란 후보를 만들지 않는다.

| 공유 필드 | 의미·존재 조건 | 비보장·무효 해석 |
|---|---|---|
| `disturbance_candidates` | 동일 Acquisition의 미확정 관측 목록, 없으면 빈 배열 | 확정 PICK 후보 아님 |
| `candidate_id` | Acquisition 내부 유일 ID | 영속 물리 객체 ID 아님 |
| `category` | `INCLINED_SURFACE`, `ELEVATED_UNRESOLVED`, `OTHER_UNRESOLVED` | 적층·기하 객체 정체성 확정 아님 |
| `center` | 실제 미해결 영역을 이루는 서로 다른 유효 Height Grid 셀의 3D 중심 좌표 산술 평균, `base_link` 기준 metre 단위 및 해당 Acquisition의 관측 시각 | 물체 전체 중심·숨겨진 형상 추정·가상 분할선·확정 접촉점 아님; 유효 셀이 없으면 후보 미생성 |
| `observed_height_m` | 같은 유효 셀 집합의 Tray 기준 높이 산술 평균, metre 단위 | 자체 두께·미관측 영역 높이 아님 |
| `acquisition_id`·`scene_generation` | 관측 출처의 동일성 | 서로 다른 관측 혼합 금지 |

기존 `inclined_surface`의 긍정적 높이 변화 근거를 재사용하며 `mixed_or_inclined`만으로 기울어짐을 확정하지 않는다. 교란 후보에는 면적·공칭 치수·완전한 직육면체 복원을 요구하지 않는다. Motion이 우선순위·접촉점·밀기 방향을 결정한다.

중심 계산은 영역에 속한 유효 Height Grid 셀을 중복 없이 한 번씩 반영한다. 누락 셀 보간, 잘린 영역의 외삽, 공칭 직육면체 중심 또는 객체 전체 중심으로 대체하지 않는다. 높이별로 독립된 부분 관측 영역은 각각의 셀 집합에서 중심을 계산한다. 세부 영역 형성 및 분류 절차는 [Cuboid top-face grasp selection Algorithm](../algorithms/cuboid-top-face-grasp-selection.md)이 소유한다.

## 3. Freshness 및 인계 경계

DetectTarget Response의 `acquisition_id`, 결과·후보 목록·Source Identity는 하나의 요청에 고정된다. 추가 관측이나 경과 시간만으로 이미 반환된 후보 목록을 수정하지 않으며, 공유 계약은 Handoff 이후 별도의 Source Age TTL을 정의하지 않는다. 그 결과를 새 Request나 Mission에 재사용하는 것은 허용되지 않고 새 요청은 새 Acquisition Identity를 갖는다. Request Deadline, Caller 취소 후 결과의 사용 가능성, PLACE Lookahead와 폐기 조건은 DetectTarget 및 Mission Cycle FDS가 소유한다.

## 4. Scene Stability 공유 의미

`/vision/scene_stability` 및 `SceneStability.msg`는 Acquisition에 연결된 Depth 안정화의 진단 상태·Source Identity다. 이는 DetectTarget의 정규 Terminal Result를 대체하거나 Orchestration에 추가 수락 Gate를 제공하지 않는다. `OBJECT_NOT_FOUND`는 유효한 부재이며 불충분·무효·불안정·모호한 관측과 구별된다. Stability 판정·필요 관측·Detector 실행 시점은 [DetectTarget FDS](../components/vision/fds-detect-target.md), Profile별 Geometry·Coverage는 [RawPart Depth-perception FDS](../components/vision/fds-rawpart-depth-perception.md)가 소유한다.

## 5. 검증 요구사항

### VR-ICD-VIS-ORCH-DISTURB — 미확정 관측 인계

유효한 기울어진 상판 관측은 일반 PICK 후보에 포함되지 않으면서 교란 후보의 중심·유형·관측 ID로 인계될 수 있어야 한다(SHALL). 교란 후보는 면적 하한을 적용하지 않으며, Sensor 오류나 미관측 영역을 유효한 교란 좌표로 꾸며서는 안 된다(SHALL NOT).

각 후보의 `center`는 해당 미확정 영역의 유효 관측 Grid 셀을 중복 없이 산술 평균한 3D 점이어야 하고(SHALL), 누락 영역을 보간하거나 전체 물체 중심으로 추정해서는 안 된다(SHALL NOT).


### 인접 의미 Owner

Vision Acquisition·Calibration·Snapshot 내부 처리는 각 Vision FDS가 소유한다. Safety Permission 및 Envelope는 Safety 설계가, Motion Goal의 Geometry·Path·Trajectory 판단은 Motion FDS가, ExecuteCycle Admission·종료·수량은 Mission Cycle FDS가 소유한다. ICD는 Vision과 Consumer 사이의 공유 Payload·Result·Identity·비보장 및 상호운용 조건만 규정한다. Handoff 뒤 별도 Sensor Age TTL이나 Live Scene Stability 조건을 추가하지 않는다.

### VR-ICD-VIS-ORCH-01 — Detector-profile Observation Pose
Orchestration은 Vision Observation Pose의 Detector Profile 기준 Geometry Reference를 재해석하거나 TCP/Tool Correction을 적용하지 않고 Motion으로 전달해야 한다.

### VR-ICD-VIS-ORCH-04 — Orientation Class 인계
Order 0은 `target_pose.orientation`을 사용 불가로 해야 한다. Order 1은 유효한 Zero를 포함한 고유 Object Orientation을 제공해야 한다. Order 2는 정사각형이 아닌 Rectangle의 Long Axis를 180° Modulo 대표값으로 제공해야 하고 Order 4는 정사각형 Edge를 90° Modulo 대표값으로 제공해야 한다. Orchestration은 Order를 보존하고 0에서 Orientation을 무시한다. Missing/Unsupported Value는 Fail-closed다. Generated Service Type 변경에는 Vision·Orchestration 쌍의 갱신이 필요하며 Mixed Binary는 지원하지 않는다.

### VR-ICD-VIS-ORCH-05 — Observation Position 및 Estimated-height Authority
Orchestration은 Vision Observation Pose Position을 PICK Motion Goal에 변경 없이 전달하고 `estimated_height_m`/`has_estimated_height`를 `estimated_object_height_m`/`has_estimated_object_height`로 변경 없이 매핑해야 한다. `estimated_height_m`는 관측된 Top Reference Position을 변경해서는 안 된다. Motion은 이 자체 두께의 절반만큼 보이는 Top Normal을 따라 안쪽으로 Geometry Center Offset을 계산해야 한다. 적층 Fixture에서 Top Elevation 0.16m, 자체 두께 0.08m라면 Center Elevation 0.12m여야 한다. 적층 높이를 두께로 반환하거나 Target XYZ를 이동해서는 안 된다.

### VR-ICD-VIS-ORCH-02 — Depletion 구분
Object Not Found는 Vision Failure와 구분되어야 한다.

### VR-ICD-VIS-ORCH-03 — 유한 요청
Request Timeout은 Paused Simulation Clock과 무관하게 Monotonic Time으로 전체 Vision Acquisition을 제한해야 한다. Orchestration은 두 번째 Stability Deadline을 추가해서는 안 된다.

### VR-ICD-VIS-ORCH-06 — Target Profile 선택
요청 `target_id`는 Vision 소유 Detector Profile을 선택해야 하고 DetectTarget Service/Processing 경계에서 무시되어서는 안 된다.

### VR-ICD-VIS-ORCH-07 — 알 수 없는 Target ID
Unknown/Unconfigured `target_id`는 `DETECT_RESULT_INVALID_RESULT`를 반환해야 한다.

### VR-ICD-VIS-ORCH-08 — 유효 Profile에서 Target 부재
설정된 유효 Profile에서 일치하는 Observation이 없으면 Unknown/Unconfigured ID와 다른 `DETECT_RESULT_OBJECT_NOT_FOUND`를 반환해야 한다.

### VR-ICD-VIS-ORCH-09 — Reference Point 의미
성공한 `target_pose.position`은 TCP/Tool Correction 없이 `base_link`의 선택된 Detector Profile Observation Reference를 표현해야 한다. RawPart Profile의 보이는 Top Surface Centroid는 RawPart Vision FDS가 정의한다.

### VR-ICD-VIS-ORCH-10 — Observation Height 분리
`estimated_height_m`는 반환 Target XYZ/Z를 바꾸면 안 된다. Motion은 변경되지 않은 Top Observation Pose에서 선택 RawPart의 자체 두께만 사용해 Object Pose를 도출한다.

### VR-ICD-VIS-ORCH-28 — 복원 Geometry와 관측 provenance
부분 형상 RawPart 성공 응답은 Recipe에서 추정한 양수 Side Length와 선택한 추정 자체 두께를 기존 필드에 전달해야 한다. 이 경우 `has_observed_rectangle=false`는 외곽선 실측이 없다는 provenance를 유지하며 값이 제공되지 않았다는 의미로 해석해서는 안 된다. 일반 Cuboid의 실측 치수와 기존 외부 메시지 스키마는 변경하지 않는다.

### VR-ICD-VIS-ORCH-11 — Validation Provenance 격리
Validation-only Fixed Vision Result는 Public ROS Shape·Result/Frame/Object Reference 의미·Fixture Registration Stamp를 유지하면서 Production Operational Profile에서 선택할 수 없어야 한다. 그 Evidence를 RGB-D Perception Validation으로 나타내서는 안 된다.

### VR-ICD-VIS-ORCH-12 — Profile 격리 Target 선택
서로 다른 Configured `target_id`는 각각 다른 Vision-owned Profile을 선택해야 한다. Unknown/Unconfigured ID는 Generic Foreground Aggregation이 아닌 `INVALID_RESULT`를 반환해야 한다. 경쟁 Foreground Geometry가 선택 Profile Pose를 이동시켜서는 안 된다. RawPart Profile은 필수 Observation이 유효할 때만 RawPart를 선택하며 부재/무효 RawPart는 Profile 계약에 따라 거부하거나 Absence로 보고한다.

### VR-ICD-VIS-ORCH-13 — 일시적 Invalid Target 구분
Configured Valid Target Profile의 일시적 Invalid Observation은 `DETECT_RESULT_TEMPORARY_INVALID_TARGET`로 보고해야 한다. Unknown/Unconfigured Profile은 `DETECT_RESULT_INVALID_RESULT`로 보고한다. 두 Outcome을 Depletion에 매핑해서는 안 된다. 이 구분은 Mission Result에 실제 Vision Failure Classification을 보존하며 자동 Retry Class를 정의하지 않는다.

### VR-ICD-VIS-ORCH-14 — Scene Stability 진단 공유 계약
`SceneStability`의 상태·Source Identity는 요청별 Depth 안정화의 진단 표현이지 Orchestration의 추가 실행·수락 Gate가 아니다. **STABLE 도달 조건, 연속 5회 관측, 원본 Depth 비교, TF Preflight 및 Detector 호출 시점**의 실행 검증은 [DetectTarget FDS의 `VR-VIS-DETECT-20`](../components/vision/fds-detect-target.md)이 소유한다. 이 ICD는 결과·진단의 공유 해석을 보존한다.

### VR-ICD-VIS-ORCH-15 — Acquisition 경계 및 Result
Acquisition ID와 단일 Terminal Result의 상관관계는 이 ICD를 따르며 Deadline 시작·유지, Stability와 Detector의 정확한 처리 순서 및 재실행 금지는 [DetectTarget FDS](../components/vision/fds-detect-target.md)가 소유한다. 공유 Response에 별도 Source Age TTL이나 Stability Acceptance Gate를 추가해서는 안 된다.

### VR-ICD-VIS-ORCH-16 — Geometry Eligibility와 유효 Absence
반환 Candidate는 선택 Detector의 단일 Observation Geometry Eligibility를 통과하고 동일한 최종 Depth Observation에서 나와야 한다. Valid Absence는 Detector Observation의 전체 Projected Workspace Coverage와 충분한 Valid Depth를 요구한다. 이 조건은 관측된 Candidate 승인 때가 아닌 Absence 확인 때만 적용한다. Insufficient/Ambiguous Evidence는 Depletion이 아닌 Failure/Timeout이어야 한다. Vision Enumeration은 Grasp Priority가 아니고 인계 뒤 Material Candidate 우선순위는 Task Execution이 소유하며 선택 후보의 Grasp Pose·계획 가능성은 Motion이 소유한다.

### VR-ICD-VIS-ORCH-17 — Acquisition 한정 단일 Material Candidate 목록
Result는 Outstanding DetectTarget Request 및 요청 Profile과 일치해야 한다. Vision은 최종 Qualified Observation의 모든 가설을 내부 평가하고 SELECTED HIGH·MEDIUM 전체를 단일 `material_candidates`의 Stable Enumeration 순서로 반환해야 한다. LOW·REJECTED는 일반 PICK 목록에 넣지 않고 진단에 보존해야 한다. 단일 목록이 비고 유효한 최종 관측에서 Geometry가 미해결이면 기존 `DETECT_RESULT_GEOMETRY_ERROR` 및 근거 있는 disturbance 계약을 적용한다. 일시적인 Observation Evidence 부족은 기존 `DETECT_RESULT_TEMPORARY_INVALID_TARGET`이며 정상적인 미해결 Geometry와 구별한다. valid absence만 `OBJECT_NOT_FOUND`다. 경과한 Source Age나 이후 Sensor/Diagnostic Update가 반환 List를 바꾸면 안 된다. Orchestration은 Vision 등급을 재판정하지 않고 전체 목록을 변경 없이 기존 단일 PnP Action에 전달하며 Task Execution은 전달된 목록의 소비·폐기 및 후보 전환을 소유한다. Outer Retry와 다음 Object는 새 DetectTarget Result가 필요하다. Legacy Response `scene_generation`, `SceneStability.msg`, 선택적 Diagnostic Topic은 Wire Compatible을 유지할 수 있으나 Orchestration은 Generation Equality·별도 Stability Lifecycle Signal·두 번째 Source Age TTL을 요구해서는 안 된다.

### VR-ICD-VIS-ORCH-26 — 단일 검출 후보 목록 인계
Vision은 Grasp Priority 의미가 없는 Stable Enumeration 순서로 SELECTED HIGH·MEDIUM Material Candidate 전체를 단일 `material_candidates` 목록으로 반환해야 한다. LOW·REJECTED는 내부 진단에 보존하고 일반 PICK 목록에서 제외한다. Candidate ID는 단일 목록 안에서 Acquisition-local하게 고유하며 후보 Pose·Geometry·provenance는 같은 Source Observation에 속한다. Orchestration은 전체 목록을 변경 없이 단일 PnP 실행 경계에 전달하고 Task Execution은 실행 순서와 Candidate Transition을 소유한다. Candidate payload는 검출 Geometry를 제공하며 Planning Occupancy를 포함하지 않는다.

### VR-ICD-VIS-ORCH-27 — Candidate Geometry와 Legacy Occupancy 비소비

Vision이 반환한 Candidate Pose·Geometry·Acquisition provenance는 Orchestration을 거쳐 변경 없이 Task Execution에 전달되어야 한다. Vision이 Planning Occupancy 배열을 생성하지 않아야 하며, Vision-derived PICK은 legacy Candidate Occupancy 필드를 Planning 근거로 소비하지 않아야 한다.

### VR-ICD-VIS-ORCH-RAWPART-EVIDENCE — Planning Policy와 독립적인 Evidence
성공 Acquisition은 반환된 Material Candidate의 관측 Pose·Geometry·Source Frame·Timestamp·Acquisition Identity를 불변으로 유지해야 한다. Vision은 Planning에 사용할 후보별 Occupancy 배열을 생성하지 않으며, Orchestration은 Vision 검출 결과를 변경 없이 Task Execution으로 전달해야 한다. Snapshot·Diagnostic Record의 상세 근거는 기존 진단 계약을 따른다.
