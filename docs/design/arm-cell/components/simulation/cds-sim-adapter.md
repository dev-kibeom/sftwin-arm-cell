# [SF-Twin] ARM Cell 시뮬레이터 상태 Adapter 컴포넌트 설계 (CDS)

- **범위:** `ARM Cell / Simulator Measured-State Adaptation`
- **Owner:** `ARM Cell Simulation Adapter` (`arm_cell_sim_adapter`)

## 1. 목적

`arm_cell_sim_adapter`는 시뮬레이터 고유의 Articulation 구조가 정규 ROS 로봇 상태 계약으로 유출되는 것을 방지한다. 측정 관절 상태와 Simulation Clock 관측을 연결하는 ROS Simulation Adapter이며 Isaac Scene·Camera·Gripper·Bringup 동작은 소유하지 않는다.

## 2. 정적 구조와 책임

```mermaid
flowchart LR
    Isaac[Isaac 측정 Articulation] --> Adapter[arm_cell_sim_adapter]
    Model[정규 Robot Model] --> Adapter
    Adapter --> Joint[/joint_states]
    Joint --> RSP[robot_state_publisher]
    Joint --> MoveIt[MoveIt Current State Monitor]
```

| 책임 | 보장 |
|---|---|
| 원본 입력 소비 | Simulator 경계의 실제 측정 Articulation State 사용 |
| 독립 관절 판별 | 정규 Robot Model에 근거해 필수 독립 가동 관절 결정 |
| 정규 상태 투영 | 정규 독립 관절의 실제 측정값만 상위 ROS 계층에 제공 |
| 원본 근거 보존 | 유효한 원본 측정값·Timestamp 및 존재하는 유효 Velocity/Effort 보존 |
| 무효 측정 차단 | 필수 값 누락·기형 Message 거부 |
| 표현 격리 | Simulation 전용 Entry·Mimic/종속 관절의 직접 노출 금지 |

## 3. 비책임 및 계약 경계

Adapter는 누락된 Position을 합성하거나 0으로 대체하지 않으며 Stale Timestamp를 갱신하지 않는다. 명시적 공유 계약 없이 단위를 재해석하거나 Simulation 전용 Articulation Entry를 노출해서는 안 된다. Arm/Gripper를 명령하거나 Robot Model TF를 발행하거나 Mission·Safety 의미를 소유하지 않는다.

정규 `/joint_states`의 필드·관절 의미와 Timestamp·TF Authority는 [Simulation State·Time·TF ICD](../../interfaces/icd-sim-state-time-tf.md)가 소유한다. Simulator Epoch 및 새 Session의 시간적 처리는 [Simulation Session FDS](fds-sim-session.md)를 따른다.

## 4. Verification Requirements

### VR-SIM-STATE-01 — 정규 관절 투영
정규 출력에는 필수 독립 관절이 포함되어야 하며 Simulation 전용·종속 Articulation Entry는 포함해서는 안 된다.

### VR-SIM-STATE-02 — 측정값 임의 생성 금지
기형이거나 필수 측정값이 누락된 입력은 임의 값을 생성하지 않고 거부해야 한다.

### VR-SIM-STATE-03 — 원본 의미 보존
명시적 ICD 변환 계약이 없는 한 유효한 원본 값과 Timestamp를 보존해야 한다.

### VR-SIM-STATE-04 — 선택 필드 보존
선택적 Velocity/Effort가 없다는 사실을 만들어낸 값으로 표현해서는 안 된다.
