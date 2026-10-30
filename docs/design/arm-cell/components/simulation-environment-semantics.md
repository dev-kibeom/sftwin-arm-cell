# ARM Cell 시뮬레이션 환경 의미 계약

- **도메인 / 의미 관심사:** ARM Cell 시뮬레이션 환경
- **소유자:** ARM Cell Product / Architecture Authority
- **관련 HLD / CDS / FDS / ICD:** [Vision CDS](vision/cds.md), [Motion CDS](motion/cds.md), [자재 인계 FDS](gateway/fds-material-handoff.md), [UI Hub FDS](ui-hub/fds.md), [Vision ↔ Orchestration ICD](../interfaces/icd-vision-orchestration.md), [외부 인계 트리거 ICD](../interfaces/icd-external-handoff-trigger.md), [Final Demo 정의](../../../product/arm-cell/definition.md)

## 1. 목적

이 문서는 Vision, Motion, Gateway, VDA, Safety, Orchestration, 운영자 화면이 하나의 시뮬레이션 Cell을 일관되게 해석하기 위해 ARM Cell 시뮬레이션 환경이 보존해야 할 횡단 의미 불변식을 정식으로 소유한다. 이 불변식은 단일 컴포넌트·동작·인터페이스에 귀속되지 않으며 환경 구현 전반에 걸쳐 적용되고 구현을 리팩토링한 뒤에도 유효하다. 환경은 공유 의미 입력이지 새로운 런타임 권한 소유자가 아니다.

이 모델은 환경 유효성과 의미만 정의한다. 컴포넌트 책임은 CDS, 런타임 동작은 FDS, 공유 계약은 ICD, 통합 데모 결과는 [Final Demo 정의](../../../product/arm-cell/definition.md)가 소유한다.

## 2. 범위

### 포함 범위

- Cell·작업 영역 공간 관계에 대한 여러 소비자의 공통 해석
- Production 인지, Physics Drop 자재 공급, 충돌 계획, 외부 프로세스 표시, Acceptance 주장에 필요한 환경 선행 조건
- 시뮬레이션 환경을 리팩토링해도 유지되는 관측 가능한 불변식

### 제외 범위

- 선반의 정확한 배치·치수, Marker Pose·ID, Prim Path, Camera 좌표, 난수 분포, AMR 경로 길이
- Isaac/USD 구성, Simulator API, Scene Graph 구성, 런타임 구현 선택
- 컴포넌트 알고리즘, 메시지 필드, 인터페이스 Endpoint, Safety 정책, Mission 순서, 승인 판단
- 데모별 Calibration 값과 Physics 설정

## 3. 도메인 문맥

환경은 여러 런타임 소유자가 사용하는 센서 관측과 공간 문맥을 제공한다. 소비자가 Production RGB-D를 해석하든, 자재를 배치하든, 충돌을 고려한 로봇 동작을 계획하든, CNC 상태를 관측하든, 시나리오 자극을 주입하든 동일한 의도된 Cell 관계를 보존해야 한다. 환경 불변식을 만족해도 그 자체로 컴포넌트 알고리즘을 입증하거나 런타임 Motion·Mission 승인을 허용하는 것은 아니다.

## 4. 의미 모델과 개념

- **정식 Cell·작업 영역 관계:** Cell, Target 작업 영역, 자재 영역, 로봇 작업 영역, 센서 관측을 해석하는 공통 공간 관계다. Frame 이름과 Transform 계약은 해당 ICD가 소유한다.
- **센서로 관측 가능한 증거:** 시뮬레이터가 작성한 Ground Truth 없이 선언된 Vision Profile에 Production 센서 관측으로 제공되는 지지·작업 표면 형상과 Target 관련 증거다. 실제 Target 선택과 검출 결과 의미는 Vision FDS가 소유한다.
- **시뮬레이션 자재 생성:** Isaac Demo Adapter는 수락된 Delivery에 대해 요청한 RawPart 수량을 생성하고 설정된 무작위 Physics Drop을 적용한다. 기존 생성 자재는 Isaac Timeline Stop까지 남는다. 이는 시뮬레이터 구현 사실이지 물리적 공급 완료의 도메인 정의가 아니다.
- **Acceptance 범위의 물리 Profile:** 물리·중력·충돌 설정은 각 Acceptance 시나리오가 주장하는 물리적 동작에 적합해야 한다. 특정 Profile을 사용했다는 이유만으로 증거를 넘어서는 주장을 할 수 없다.
- **Fault 자극:** Scene 요소는 소유 경계를 통해 관측 가능한 조건을 만들거나 시나리오를 요청할 수 있다. 요소의 존재나 시각적 상태가 다른 컴포넌트의 정식 권한을 변경하지 않는다.

## 5. 소유권과 권한

이 문서는 아래 횡단 환경 불변식만 소유하며 기존 소유자의 권한을 이전하지 않는다.

- Vision FDS/VR은 Profile 실행, Target 선택, 검출 성공, 인지 결과 의미를 소유한다. 이 문서는 해당 결과를 주장하지 않는다.
- 외부 생산자는 Delivery 할당과 Trigger 식별자를 소유한다. Gateway만 원시 Trigger를 수락하고 수락된 이벤트를 VDA와 선택된 공급 제공자에게 전달한다. VDA는 VDA 사실을 소유한다. 공급 제공자는 환경에서 정의된 공급 완료 사실을 보고한다. Gateway는 필수 VDA 문맥과 함께 이를 검증하고 Readiness를 소유한다. Isaac Spawn·Physics Drop 진단은 시뮬레이션 구현 세부사항으로 유지한다.
- Orchestration은 Batch 승인과 Mission 진행을 소유한다.
- Safety는 Capability, 심각도, 허용 Motion Envelope를 소유한다.
- Motion은 충돌 고려 경로 계획과 실행을 소유한다.
- UI Hub는 정식 상태를 관측하고 시뮬레이션·Fault 자극을 발행한다. 데모 Handoff Trigger는 외부 계약을 사용하며 Hub에 Delivery 권한을 부여하지 않는다.
- ICD는 Frame, Sensor, Material 및 외부 프로세스 계약을 계속 소유한다.

## 6. 컴포넌트 간 사용

| 소비자 | 필요한 일관된 해석 |
|---|---|
| Vision | Production Depth 관측은 선언된 Profile에 쓸 수 있는 지지면과 Target 증거·형상을 제공한다. Profile 실행, Depth Scene Stability 관측, 검출 결과는 Vision FDS가 소유한다. |
| Gateway / VDA / Material Supply | VDA Phase와 선택된 공급 제공자의 공급 사실은 동일한 수락 Delivery를 가리킨다. Gateway는 제공자 식별자, 수량, 유효성·신선도, 필수 VDA 문맥을 검증하고 Readiness를 게시한다. Isaac Physics Drop은 데모 구현이며 성공하더라도 영역 점유, 안정성, Vision 검출, 파지 가능성을 약속하지 않는다. RGB-D 관측 가능성과 Task Pose는 Vision이 독립적으로 판단한다. |
| Motion | 로봇 작업 영역, 자재 영역, 충돌 계획 형상은 동일한 정식 Cell 공간 관계를 가리킨다. |
| Safety / UI Hub | CNC/PackML 표시는 정식 외부 프로세스 상태를 반영하며 해당 상태를 새로 만들거나 변경하지 않는다. |
| Fault 시나리오 소비자 | 환경 요소는 소유 경로로 자극을 생성하며 정식 상태 권한을 우회하지 않는다. |

## 7. 의존성과 경계

공간 Frame 이름, 센서 데이터 의미, 자재 전달 사실, 외부 PackML 상태, 충돌 Scene 소유권 및 컴포넌트 런타임 동작은 해당 ICD/CDS/FDS가 명시한다. 이 문서는 해당 계약들이 함께 소비하는 환경 수준 불변식만 정의한다.

구현과 Calibration 값은 설정·구현 권한이 소유한다. 정확한 Pose, 치수, Prim Path, Marker 식별자, Camera 좌표, Motion 거리, Simulator Metadata는 여기서 의미 요구사항이 아니다.

## 8. 도메인 불변식

1. 정식 Cell·작업 영역의 Frame 관계는 센서 관측, Target Reference, 자재 영역, 로봇 작업 영역, 충돌 계획 표현에서 의미상 일관돼야 한다(SHALL).
2. Production Vision 센서는 선언된 Production Target Profile을 지원하기에 충분한 수준으로 공칭 Target 작업 영역을 관측할 수 있어야 한다(SHALL).
3. 지지·작업 표면 기준은 시뮬레이터가 작성한 표면 식별자 없이 Production 센서 관측과 설정된 Marker·Tray 형상으로 초기 설치 또는 명시적인 운영자 Calibration 요청 시 설정할 수 있어야 한다(SHALL). 일반 Production에서는 Marker를 반복 관측하지 않고 운영 Profile이 선택한 Calibration 파일과 설정 Tray 형상을 사용해야 한다(SHALL).
4. 지지면 위의 Target 자재는 선언된 Production Vision Profile이 관측을 평가할 충분한 센서 관측 증거와 형상을 제공해야 한다(SHALL). 이 불변식이 Target 선택이나 검출 성공을 보장하지는 않는다.
5. 무관한 전경 형상이 존재해도 설정된 Production Profile에서 요청 Target을 격리할 수 없게 만들어서는 안 된다(SHALL NOT).
6. Gateway가 수락한 Trigger에 대해 Isaac Demo Adapter는 요청된 설정 RawPart 수량을 생성하고 무작위 초기 Pose 및 Physics Drop을 적용해야 한다(SHALL). 시뮬레이션 공급 성공은 실제 오류 없이 생성과 동작 실행이 완료됐음을 의미하며 영역 도착, 지속 존재, Settling, 가시성, 파지 가능성을 주장하지 않는다. Pose 정렬, 재배치, 재샘플링, 자동 복구는 금지한다.
7. 공유 공급 완료 사실은 선택된 제공자가 요청 수량을 유효하게 완료했고 Delivery와 연결됨을 표현해야 한다(SHALL). Gateway는 `MATERIAL_READY` 게시 전에 식별자, 수량, 유효성·신선도, 필수 VDA 문맥을 검증해야 한다(SHALL). Timer, VDA Phase, UI 상태, 기존 RawPart는 준비 상태를 성립시켜서는 안 된다(SHALL NOT). Isaac 영역 관측 및 Vision Scene Stability는 이 공유 완료 계약의 일부가 돼서는 안 된다(SHALL NOT).
8. 자재 선반 구조는 의도된 빈 Tray의 설치·유지보수 Calibration 관측에서 ArUco 지지 Marker가 수신·Drop Volume 밖에 장착되어 관측 가능하게 유지해야 한다(SHALL). 정상 Production 검출에는 매 프레임 Marker 가시성이 필요하지 않다. 공급 샘플링은 알고리즘으로 Marker를 회피하지 않는다. 정확한 형상은 환경 설정에 따른다.
9. 공급 무작위화는 Vision·Motion 성공 제약, Pose Filtering, Drop 후 재배치, 강제 정렬, 자동 실패 복구를 사용해서는 안 된다(SHALL NOT). 동일 Scene·Profile·Adapter·Runtime 설정에서 제공된 Seed는 샘플링한 초기 Pose를 재현한다.
10. 환경의 CNC/PackML 표시기는 정식 외부 프로세스 상태의 읽기 전용 투영이어야 하며(SHALL) 해당 상태를 게시·선택·덮어써서는 안 된다(SHALL NOT).
11. 로봇 작업 영역, 자재 영역, 충돌 계획 표현은 정식 Cell·작업 영역 관계 내에서 의미상 정렬돼야 한다(SHALL).
12. Production 센서 대상 환경은 시뮬레이터 Prim 식별자, 정확한 World 좌표, Simulator Metadata, API Ground Truth 없이 센서 관측으로 Target 증거를 노출해야 한다(SHALL).
13. Physics·중력·충돌 설정은 각 Acceptance 주장에 적합해야 하며(SHALL), 설정 Profile이 해당 주장을 무효화해서는 안 된다(SHALL NOT).
14. Fault Injection Scene 요소는 자극 대상인 Safety, Orchestration, Vision, Motion, VDA, Gateway, Material Supply의 정식 권한을 우회·덮어써서는 안 된다(SHALL NOT).
15. 새로 수락된 Delivery ID는 이전 생성 자재를 삭제하거나 활성 Mission 상태를 변경하지 않고 자재 추가 생성을 즉시 시작해야 한다(SHALL). 명시적 Isaac Timeline Stop만 Isaac 소유 생성 자재 및 검증 Fixture를 제거한다. 표준 새 실험 순서는 수동 ROS Bringup 중지 → Timeline Stop → 수동 Bringup 시작 및 Endpoint 검증 → Timeline Play·Runtime 준비다. 새 ROS 프로세스는 프로세스 로컬 작업량 계산을 빈 상태로 시작한다. Isaac Python·Extension만 Reload하면 Stage·자재 상태를 보존한다.
16. 시뮬레이션 생성은 Isaac Material Supply Adapter와 설정된 Spawn·Drop 구성을 사용해야 한다(SHALL). Mission·Motion·Vision Idle 상태를 기다리지 않고 활성 Goal이나 인계된 PICK Pose를 변경하지 않는다. 기존 Vision 안정화와 Motion·Safety Fail-safe 계약이 관측 결과를 결정하며 별도의 공급 Gate나 Safety 정책을 추가하지 않는다.

## 9. 검증 요구사항

| ID | 의미 주장·불변식 | 필수 관측 결과 |
|---|---|---|
| `VR-ARM-SIM-ENV-01` | 정식 Cell·작업 영역 Frame 관계의 일관성 유지 | 센서 관측, Target Reference, 작업 영역, 계획 형상이 서로 일관된 공간 관계로 해석됨 |
| `VR-ARM-SIM-ENV-02` | Production Vision이 공칭 Target 작업 영역을 관측 | 선언된 Target 작업 영역이 유효한 Production 센서 관측에 표현되고 설정된 Production Profile을 지원함 |
| `VR-ARM-SIM-ENV-03` | 센서로 설정 가능한 설치 지지면 기준 | 초기 설치 또는 명시적 운영자 Calibration 관측과 설정된 Marker·Tray 형상으로 시뮬레이터 작성 식별자 없이 지지 기준을 설정. 일반 Production은 Marker 반복 관측 없이 운영 Profile이 선택한 Calibration 파일을 사용 |
| `VR-ARM-SIM-ENV-04` | 지원 Target 자재가 Profile에 필요한 센서 증거·형상 제공 | 지지면의 Target에 대한 Production Depth 관측에 선언된 Profile의 센서 입력 선행 조건에 충분한 증거·형상이 있음. 선택적 RGB는 표시에 사용하고 Vision Target 선택·검출 성공까지 주장하지 않음 |
| `VR-ARM-SIM-ENV-05` | 무관한 전경이 있어도 Target Profile 격리 가능 | Target과 무관한 전경이 함께 있어도 Production 센서 관측에서 요청 Profile의 선언된 센서 제약에 따라 Target 증거를 구별할 수 있음. Target 선택 결과는 Vision FDS가 소유 |
| `VR-ARM-SIM-ENV-06` | 과거 자재 인계 연속성(외부 공급 경로에서는 대체됨) | 과거 Scene 증거는 하나의 Delivery에 대해 VDA 하역 구현에서 자재 작업 영역까지의 연속성을 보여줬으나 새로운 Physics Drop이나 Transfer Completion 주장을 입증하지 않음 |
| `VR-ARM-SIM-ENV-07` | Scene 구현은 정식 자재 권한을 우회할 수 없음 | 전달·공급 Scene 요소의 활성화·관측만으로 Gateway Readiness 또는 Orchestration Admission이 변경되지 않으며 각 소유 인터페이스에서 해당 사실을 관측 가능 |
| `VR-ARM-SIM-ENV-08` | CNC/PackML 표시의 읽기 전용성 | 표시기가 정식 외부 프로세스 상태를 따르며 표시기만 변경해도 상태는 바뀌지 않음 |
| `VR-ARM-SIM-ENV-09` | 로봇·자재·충돌 계획 영역의 의미상 정렬 | Acceptance에 사용된 영역에 대해 관측된 Cell 관계와 계획 Collision 표현이 일치 |
| `VR-ARM-SIM-ENV-10` | Simulator Ground Truth 없이 센서 관측에서 Production Target 증거 획득 가능 | 시뮬레이터 전용 식별자·Metadata·정확한 좌표·API 접근 없이 센서 관측과 선언 Profile 입력에 의미 해석에 필요한 Target 증거가 존재하며 Vision 결과까지 주장하지 않음 |
| `VR-ARM-SIM-ENV-11` | 물리 설정이 Acceptance 주장 지원 | 활성 Physics·중력·충돌 Profile이 시나리오 증거가 주장하는 물리 동작과 모순되지 않음 |
| `VR-ARM-SIM-ENV-12` | Fault 요소의 정식 권한 우회 금지 | Scene 자극은 소유 컴포넌트·인터페이스를 통해서만 정식 상태를 변경하고 Scene·UI가 직접 정식 상태를 변조하지 않음 |
| `VR-ARM-SIM-ENV-13` | 시뮬레이션 공급은 생성·동작 결과 보고 | 동일 수락 Delivery에 대해 Demo Adapter가 요청 RawPart 수량 생성 및 무작위 Physics Drop이 실행 오류 없이 적용됐는지 보고. 위치·지속 존재·안정성·가시성·파지 가능성을 증명하지 않으며 Production 공급 증거가 아님 |
| `VR-ARM-SIM-ENV-14` | 선반 구조의 설치 Calibration 기준 보존 | 의도된 빈 Tray 설치·유지보수 관측에서 ArUco가 수신·Drop Volume 밖에 장착되고 관측 가능. Production 프레임별 관측이나 자재 샘플러의 Marker 회피는 필요하지 않음 |
| `VR-ARM-SIM-ENV-15` | 공급 무작위화가 보정하지 않은 물리 결과를 노출 | 동일 Seed·설정에서 초기 Pose가 반복되고 충돌·가림·접근 불가 Pose·공급 실패는 관측 가능한 상태로 남으며 성공 조건부 샘플링이나 자동 보정이 없음 |
| `VR-ARM-SIM-ENV-16` | 공급 완료와 독립적인 RawPart 안정화 평가용 Tray 지지면 센서 증거 | 선택·로드된 Calibration 및 설정 Tray 형상으로 변환한 최신 Depth 관측은 안정된 Tray Depth 장면, 유효한 Target 부재, 장면 변화, 무효·불충분한 관측을 구분할 만큼 설정 작업 영역 증거를 보존함. 선택적 RGB는 Production 검출 선행 조건이 아님. 매 프레임 ArUco 검출, 무관한 Tray 형상, Isaac Prim·Physics 상태는 필요하지 않음. Vision Profile의 STABLE/UNOBSERVABLE 의미는 RawPart FDS 및 Vision–Orchestration ICD가 소유 |
| `VR-ARM-SIM-ENV-17` | 반복 Delivery 및 프로세스 내 Isaac 개발 Reload는 활성 실험을 보존하고 Reset 뒤 새 ROS Session 시작 | 서로 다른 수락 Delivery의 공급은 Stage에 누적. Isaac Python·Extension Reload는 자재를 보존하고 활성 Runtime Callback·Node 집합 하나만 유지. Timeline Stop은 새 Bringup 실험 전에 생성 자재·Fixture를 제거 |

## 10. 참조

- [Final Demo 정의](../../../product/arm-cell/definition.md)
- [Vision CDS](vision/cds.md), [Vision DetectTarget FDS](vision/fds-detect-target.md), [Vision ↔ Orchestration ICD](../interfaces/icd-vision-orchestration.md)
- [시뮬레이션 RGB-D ICD](../interfaces/icd-sim-rgbd.md), [Camera Overlay CDS](simulation/cds-camera-overlay.md)
- [Motion CDS](motion/cds.md)
- [자재 인계 FDS](gateway/fds-material-handoff.md), [VDA 자재 전달 FDS](vda/fds-material-delivery.md), [외부 인계 Trigger ICD](../interfaces/icd-external-handoff-trigger.md)
- [외부 상태 ↔ Safety ICD](../interfaces/icd-external-safety.md), [Safety Supervision FDS](safety/fds-supervision.md), [Simulation UI Hub FDS](ui-hub/fds.md)
- [Fault Injection FDS](vda/fds-fault-injection.md)
