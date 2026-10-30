# [SF-Twin] ARM Cell Simulated RGB-D Interface Contract

- **범위:** `Simulated RGB-D Producer ↔ Vision Consumer`
- **Owner:** `ARM Cell Shared Interface`
- **Related Architecture Record:** `[ADR-ARM-CELL-0003](../../../record/arm-cell/architecture-decision/rgbd-sensor-contract.md)`

## 1. 목적

이 ICD는 Vision이 소비하는 RGB-D Sensor 공유 계약을 정의한다. Isaac Simulation Source와 실제 RGB-D Source 모두 지원하되, Simulator에서만 성립하는 Timestamp의 완전한 일치를 일반 Vision 계약으로 격상하지 않는다.

## 2. 토픽

정규 Sensor Channel은 다음과 같다.

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

## 3. 인코딩

현재 정규 의미는 다음과 같다.

- RGB: 설정된 Vision Pipeline과 호환되는 Color Image
- Aligned Depth: Deprojection과 호환되는 미터 단위 Depth 표현
- CameraInfo: ROS의 기본 Camera Calibration 의미

현재 Simulation Profile의 RGB는 `rgb8`, Aligned Depth는 미터 단위 `32FC1`을 사용한다. 이 Profile Encoding을 모든 실제 Sensor의 요구사항으로 확장하지 않는다. 다른 Source는 설정된 Consumer와 호환되는 Encoding을 제공하고 아래의 Depth·Frame·Calibration 의미를 보존해야 한다. Producer와 Consumer의 Encoding 호환성은 Integration 시 검증해야 한다.

## 4. QoS

Vision RGB-D Subscription은 ROS SensorData 방식의 다음 QoS를 요청해야 한다.

```text
Reliability: Best Effort
Durability: Volatile
```

Producer는 호환되는 더 강한 Reliability를 제공할 수 있다. 예를 들어 현재 Isaac Profile은 Reliable/Volatile을 제공해도 Best-Effort/Volatile Vision Subscriber와 호환될 수 있다. QoS 호환성은 Integration 시 검증해야 한다.

## 5. Calibration RGB ↔ Depth 시간 연관

DetectTarget과 Snapshot은 RGB와 Depth 간 시간 연관을 요구하지 않는다. Snapshot RGB는 검출 이후 독립적으로 선택하는 시각화 Image다. 두 Source Timestamp의 차이와 로컬 수신 순서는 그 사용에 영향을 주지 않는다.

명시적 Fiducial Calibration에는 동기화된 RGB-D 입력을 계속 사용한다. 이 Pair에 정확한 Timestamp 일치는 요구하지 않는다. 유효한 Calibration RGB-D Pair는 다음 조건을 충족한다.

```text
abs(t_rgb - t_depth) <= configured_sync_tolerance
```

Tolerance는 검증 대상인 Vision Ingress 설정값이다. 현재 Isaac Profile에서 `Δt = 0`이 자연스럽게 관측될 수 있으나, 이는 Simulator의 특성이지 Backend 공통 요구사항이 아니다.

## 6. CameraInfo 연관

CameraInfo는 Calibration State다. Vision은 현재 Image Geometry와의 호환성을 검증한 뒤 CameraInfo를 Cache할 수 있다. CameraInfo와 각 RGB-D Pair의 Frame별 Timestamp 완전 일치는 이 ICD의 요구사항이 아니다. 관련 Frame·Resolution·Intrinsic Identity가 바뀌면 Cached Calibration을 무효화하거나 다시 검증해야 한다.

## 7. 좌표계

정규 Optical Measurement Frame은 다음과 같다.

```text
camera_color_optical_frame
```

Simulation Camera Pose는 Isaac Simulation Camera Tooling이 Stage Geometry로부터 계산한다. `arm_cell_bringup`은 설정된 ROS Camera TF Overlay를 발행한다. Vision은 해당 Frame 및 TF Edge를 소비하고 경쟁하는 Camera Transform을 발행하지 않는다.

## 8. Depth 의미

Depth 값은 설정된 Aligned-Depth 계약에 따른 기하학적 Range/Depth를 나타내며, 실행 중 숨겨진 Unit 재작성을 거치지 않고 소비된다. 현재 Simulation Profile은 직접 Geometric Deprojection에 적합한 미터 단위 Depth를 제공한다.

## 9. 데이터 보존

Consumer는 불일치하는 입력을 유효해 보이도록 만들기 위해 Pixel·Depth 값·Timestamp·Frame ID·CameraInfo를 임의로 변경해서는 안 된다.

### 9.1 Acceptance Sensor-Fault Stimulus 경계

Final Demo의 Invalid/Stale/Unavailable Vision 시나리오는 Production RGB-D 입력 경로의 승인된 Source-side Stimulus를 통해 재현할 수 있어야 한다. Stimulus는 Vision Subscriber 이전에 적용하며 필수 Image/Calibration 입력을 일시 중단하거나, Stale/역순 Sample을 발행하거나, Invalid/비호환 Sample 또는 Calibration을 제공할 수 있다. 활성 Stimulus와 정합성 있는 Fresh Stream으로의 복구는 정규 Sensor 경계에서 관측 가능해야 한다. 시나리오 값과 정확한 Stimulus Duration은 배포·시험 설정에 따르며, 별도 ROS Control Interface나 정상 Sensor 의미를 변경하지 않는다.

Stimulus는 Acceptance와 동일한 RGB-D Topic 및 Production Vision Operational Profile을 사용해야 한다. DetectTarget 결과를 주입하거나 Vision 내부 Buffer·Profile·Result를 변경하거나 Vision·Orchestration·Motion·Safety 정규 상태를 직접 기록해서는 안 된다. Stimulus를 Clear하면 Source의 정상 발행을 복원한다. Vision Ingress와 Result 처리는 기존 [Vision Frame Ingress FDS](../components/vision/fds-frame-ingress.md), [DetectTarget FDS](../components/vision/fds-detect-target.md), [Vision↔Orchestration ICD](icd-vision-orchestration.md)를 따른다. Validation 전용 Fixed Vision Adapter는 Sensor-Fault 경계나 Production Perception 근거가 아니다(`VR-ICD-VIS-ORCH-11`).

## 10. 검증 요구사항

### VR-ICD-RGBD-01 — Best-Effort Consumer QoS
Vision은 RGB-D 입력에 Best-Effort/Volatile SensorData 방식의 QoS를 요청해야 한다.

### VR-ICD-RGBD-02 — Calibration의 유한 시간 동기화
명시적 Calibration의 RGB/Depth Pair는 모든 Source에 완전한 Timestamp 일치를 강제하지 않고 유한한 시간 연관을 사용해야 한다. DetectTarget과 Snapshot에는 이 연관을 요구해서는 안 된다.

### VR-ICD-RGBD-03 — Calibration 연관
Vision은 현재 Image Geometry와 호환성이 검증된 CameraInfo만 사용해야 한다.

### VR-ICD-RGBD-04 — Optical Frame
RGB-D Geometry는 정규 Optical Frame 계약에 따라 해석해야 한다.

### VR-ICD-RGBD-05 — 숨은 재작성 금지
Producer/Consumer 경로는 비호환성을 감추기 위해 데이터 의미를 암묵적으로 변경해서는 안 된다.

### VR-ICD-RGBD-06 — Production Sensor-Fault 시나리오 경계
승인된 Acceptance 경계는 Unavailable, Stale/역순, Invalid/비호환 Sensor 조건이 Production RGB-D Source 경계에 드러나도록 해야 하며 Clear 후 정합성 있는 Fresh 발행을 복원해야 한다. Oracle은 Sensor 입력 경로와 Stream 상태다. Vision 거부 및 Result 분류는 Vision↔Orchestration ICD의 `VR-VIS-INGRESS-01`, `VR-VIS-INGRESS-03`~`VR-VIS-INGRESS-05`, `VR-VIS-DETECT-01`, `VR-ICD-VIS-ORCH-02`에 따라 검증한다.
