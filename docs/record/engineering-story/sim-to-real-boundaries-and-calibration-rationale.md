# 시뮬레이션을 검증 환경으로 두고 실제 로봇 적용을 지향한 설계 과정

*Engineering Story — 2026-10-30 개발자의 설계 의도 회고와 저장소 문서·PR 근거를 대조한 역사적 기록. 이 문서는 현재 설계 계약이나 실기 장비 인수 결과가 아니다.*

## 목적과 선택

SFTWIN ARM Cell의 목적은 Isaac Sim에서만 성공하는 장면을 만드는 데 한정되지 않았다. 설계자는 **현실적 공장 작업 흐름, 실제 센서·로봇과 호환되는 관측·명령 계약, 환경별 구현 교체 가능성, 시뮬레이션과 현실의 차이를 분리해 다루는 것**을 함께 고려했다고 회고했다. 따라서 Sim-to-Real은 사후에 붙인 구호가 아니라 시스템 경계를 설계할 때 선택한 방향이었다. 다만 처음부터 그 목표가 동일한 우선순위였는지의 개발 시점별 변화는 이 회고만으로 단정하지 않는다.

선택한 방식은 **공통 업무·인지·실행 의미는 시뮬레이터 밖에 남기고, Isaac 고유 Scene·Physics·Articulation·Clock 및 장치 동작은 Adapter/Backend가 맡도록 하는 것**이다. 현실의 물리·센서·기구 편차는 시뮬레이션에 가능한 범위까지 반영하되, 잔여 차이는 실제 장비 Calibration과 통합 검증의 대상으로 남겼다. 이는 [현행 ARM Cell System Overview](../../design/architecture/system-overview.md)에 명시된 구조 전략과도 부합한다.

## 실제로 선택했던 경계와 이유

### 로봇 상태·모델·시간: 서로 다른 권한을 혼합하지 않음

[Model, State and Time ADR](../architecture-decision/model-state-and-time.md)는 ROS와 Isaac의 로봇 모델을 별도로 편집하다 드리프트가 발생하는 대안을 피하고, 공유 모델 자원과 Canonical JointState Adapter를 채택했다. 로봇 TF의 Authority는 ROS에, 시뮬레이션 Clock Authority는 Isaac에 둔 역사적 판단이다. 이는 특정 시뮬레이터 내부 관절 구조를 상위 Motion 판단의 계약으로 만들지 않으려는 구분이다.

### RGB-D: 논리적 센서 정체성 및 좌표 변환

[RGB-D Sensor ADR](../architecture-decision/rgbd-sensor-contract.md)는 카메라 Asset 내부 Prim 경로 대신 논리적인 센서 Identity를 유지하고, CameraInfo·Depth·TF의 출처 및 좌표 변환을 명시했다. Vision의 생산 입력은 관측 RGB-D이며, USD Prim 식별자나 Ground Truth는 검출 정답을 대체하지 않는다. 시뮬레이터는 Sensor Data를 만드는 구현 환경이지 검출 의미의 Owner가 아니다.

### 외부 자재 공급: 동일 계약, 다른 실행 주체

PR #471은 Simulation과 외부 공급 처리에 중복되어 있던 Material Handoff Coordinator를 IT–OT Gateway 경계로 일원화했다. Isaac Material Supply Adapter와 현장 공급자는 자재를 공급하는 서로 다른 실행 주체이지만, Gateway는 동일 Trigger/TransferCompletion 계약을 검증한다. IT–OT Gateway 기록은 이를 별도로 상세히 설명한다.

### 업무 처리와 환경 구현의 분리

[Simulator Realization Isolation ADR](../architecture-decision/simulator-realization-isolation.md)은 Mission·Vision·Motion·Safety의 공통 의미와 Isaac 고유 구현을 분리한다. 구체적인 ROS Interface, Camera/Gripper/Robot Backend와 시뮬레이터 Session 실현 방식은 현재 CDS/FDS/ICD의 책임이다. 동일 의미를 유지한 채 대체 Backend를 작성할 수 있도록 설계했다는 것과 **이미 실제 로봇에 적용해 검증했다는 것**은 엄격히 다르다.

## 의도적 선택과 대가

- 시뮬레이터 Ground Truth를 편의상 생산 판단에 직접 사용하면 테스트 데모는 쉬워지지만, 실제 센서 관측에서 나타나는 불확실성을 숨길 수 있다. 따라서 검출은 측정 입력과 알려진 공장 기하학으로 수행한다.
- ROS/Isaac의 내부 상태를 직접 결합하면 Adapter가 줄어들지만, 새 Backend나 실기 장비 이식 시 상위 로직까지 바뀔 위험이 있다. 대신 변환·검증 경계를 명시한다.
- 시뮬레이션에서 물리·센서 특성을 모두 똑같이 맞출 수 있다고 가정하지 않는다. 설치 좌표·카메라 Intrinsics/Extrinsics·로봇 동특성·Gripper 접촉·안전 장치 및 현장 공급 검증은 실제 장비에 맞춘 Calibration/시운전이 필요하다. 이들은 가능한 후속 작업의 예시이지 본 기록에서 검증이 완료된 항목은 아니다.
- 소프트웨어 Safety 감독과 물리적 E-Stop/STO·안전 인증은 다른 권한이다. Simulation Acceptance를 현장 안전 적합성으로 확대 해석하지 않는다.

## 현재 검증 단계와 실기 전환에 남은 작업

현재 시스템은 Isaac 기반 통합 데모를 통해 기능을 검증하는 단계이며, PR #648은 현 인수 범위가 시뮬레이션 데모임을 제품 정의와 분리하여 명시했다. 개별 Adapter·Gateway·상위 계약을 검증한 테스트만으로 실제 기구·센서·네트워크·현장 Safety 통합 검증이 완료된 것은 아니다. 이 기록에 실기 적용 성공률, 장비 호환성 수치, Calibration 정확도를 새로 주장하지 않는다.

포트폴리오에서는 **'실제 로봇 이식을 고려한 시뮬레이터 중립적 계약과 환경별 Adapter 경계를 설계·구현했다'**고 설명할 수 있지만, **'실제 로봇에서 검증된 시스템'**으로 표현하려면 별도의 실기 장비 증거가 필요하다.

## 근거와 남은 질문

- 설계자 회고: 여러 Sim-to-Real 목표를 함께 고려하고 환경 차이를 분리 관리했다는 2026-10-30 답변.
- 기존 ADR: [Simulator Realization Isolation](../architecture-decision/simulator-realization-isolation.md), [Model/State/Time](../architecture-decision/model-state-and-time.md), [RGB-D Sensor](../architecture-decision/rgbd-sensor-contract.md), IT–OT Gateway.
- 현행 설계: [System Overview](../../design/architecture/system-overview.md).
- 관련 변경: PR #471, PR #648.
- 후속 질문: 이 Sim-to-Real 판단이 최초 기획부터의 목표인지, 개발 과정에서 인수 범위가 커지며 강조되었는지; 현장 이식 시 어떤 항목을 가장 먼저 실측 검증할 계획인지. 설계자 판단과 검증 계획이 확인되기 전에는 추정으로 쓰지 않는다.
