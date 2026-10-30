# [SF-Twin] ARM Cell 시뮬레이션 상태·시간·TF 공유 계약

- **계약 Owner:** ARM Cell Shared Interface
- **소비자:** Isaac Adapter, ROS Bringup, Motion, Vision, Orchestration

## 1. 목적과 공유 계약 경계

시뮬레이션의 관절 상태, 시간 기준, TF 권한 및 Timeline 재시작 이후 관측의 공유 의미를 정한다. Play/Pause/Stop 절차·세션 정리·Bringup 순서는 [Simulation Session FDS](../components/simulation/fds-sim-session.md)가 소유한다.

## 2. 참여자와 인터페이스

| 논리적 데이터·채널 | Producer / Authority | Consumer |
|---|---|---|
| `/isaac/joint_states` | Isaac 시뮬레이터 | ARM Cell 상태 Adapter |
| `/joint_states` | 정규 관절 상태 Adapter | ROS Motion 및 상태 소비자 |
| `/clock` | Isaac 단독 | 시뮬레이션 시간 사용 ROS 노드 |
| 로봇 모델 TF | `robot_state_publisher` 단독 | TF 소비자 |
| `world → base_link` | ARM Cell Bringup 설정의 단일 발행자 | TF 소비자 |

## 3. 공유 의미 계약

- 정규 `/joint_states`는 필요한 독립 정규 관절만 전달한다. 시뮬레이터 내부·종속 관절을 정규 상태로 노출하지 않는다. 필수 관절의 누락·무효값은 만들어 채우지 않고 무효로 판단한다.
- `/clock`은 지원 프로파일에서 Isaac만 발행한다. 관측의 Source Stamp, ROS 시뮬레이션 시간, 수신자의 로컬 Steady Time은 서로 다른 의미를 갖는다. 관측·진단의 공유 의미 소유자는 [Observability Records ICD](icd-observability-records.md)다.
- Timeline Stop은 Safety Reset 명령이 아니다. 세션 전환과 Process-local Mission Accounting의 수명은 각 FDS가 소유한다.
- `robot_state_publisher`의 로봇 모델 TF와 Bringup의 `world → base_link` TF는 각각 단일 권한만 갖는다. MoveIt Virtual Joint가 추가 발행 권한을 만들지 않는다.

## 4. 상호운용 조건

| 조건 | 공통 해석 |
|---|---|
| Epoch 전환 | 이전 Play의 Stamp·TF·관측을 현재 실행의 신선한 근거로 재사용하지 않음 |
| 관측 데이터 | 실험용 오프라인 분석 결과는 Time·Joint State·TF·Safety의 새로운 Authority가 아님 |

## 5. 검증 요구사항

### VR-ICD-SIM-STATE-01 — 정규 관절 상태 발행 경로
정규 `/joint_states` 발행 경로는 하나여야 한다.

### VR-ICD-SIM-STATE-02 — 정규 관절의 범위
시뮬레이터 전용 관절 또는 종속 관절을 정규 상태로 전달해서는 안 된다.

### VR-ICD-SIM-TIME-01 — 단일 시뮬레이션 시간 권한
지원 프로파일에서 Isaac만 `/clock`을 발행해야 한다.

### VR-ICD-SIM-TIME-02 — Play 이후 관측 검증
새 작업을 승인하기 전에 현재 Play의 시간·상태·TF를 검증해야 한다. Timeline Stop의 시계 역행은 허용되며 세션·Bringup 순서는 Simulation Session FDS를 따른다.

### VR-ICD-SIM-TIME-03 — Epoch와 Safety Reset의 구분
Timeline Stop은 Safety Reset이 아니며, Safety 권한·Latch·재시작 의미는 Safety 설계를 따른다. 새 실험의 Session 전환은 [Simulation Session FDS](../components/simulation/fds-sim-session.md)가, Mission Accounting 수명은 [Mission Cycle FDS](../components/orchestration/fds-mission-cycle.md)가 소유한다.

### VR-ICD-SIM-TF-01 — 로봇 모델 TF 권한
로봇 모델 TF의 Publisher는 하나여야 한다.

### VR-ICD-SIM-TF-02 — 기준 좌표계 TF 권한
`world → base_link`의 설정된 Publisher는 하나여야 한다.

## 6. 계약 의존성과 소비 관계

- [Simulation Session FDS](../components/simulation/fds-sim-session.md): Timeline·Bringup·세션 전이
- [Observability Records ICD](icd-observability-records.md): 복수 시간 기준의 진단 근거
- [Motion CDS](../components/motion/cds.md): 정규 관절 상태 및 TF 소비 책임
