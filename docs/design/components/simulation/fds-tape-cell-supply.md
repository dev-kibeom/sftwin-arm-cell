# Tape Handling Cell 공급·물리 이송 FDS

- **Function ID:** Tape Cell Cartridge Processing
- **Owner:** Tape Cell Simulation 공급 관리

## 목적과 동작 범위

선택된 Tape Cell의 정렬된 카트리지 자재를 한 슬롯씩 공통 Task Execution에 맡겨 수령 위치로 옮기고,
확인된 물리 이송·해제만 공급 완료 수량에 반영한다. 시작은 명시적인 Cell 실행 요청이며,
종료는 공급 소진 또는 첫 실패다. ROS Mission Admission·Vision 관측·Safety 승인·Motion 계획을 대신하지 않는다.

## 참여 책임과 계약

| 참여 책임 | 역할 | 관련 계약 |
|---|---|---|
| Cell 공급 관리 | 슬롯 배치·선택, 슬롯 점유·완료 수량, Stop 복원 | 이 FDS |
| Material | 중공 롤 치수·질량·마찰·관성·Collision 형상 | Material Profile 및 [환경 의미](../simulation-environment-semantics.md) |
| Task Execution | 단일 자재 GraspCycle 순서와 terminal 결과 | [Task Execution CDS](../task-execution/cds.md) |
| Tool Runtime | Internal Radial Grasp 구동·Holding·Release 관측 | [Motion Backend ICD](../../interfaces/icd-motion-backend.md) |
| Simulation | 중력·접촉에 의한 물리 이송, Timeline lifecycle | [Simulation Session FDS](fds-sim-session.md) |

## 핵심 시나리오

```mermaid
sequenceDiagram
    participant C as Cell 공급 관리
    participant T as 공통 Task GraspCycle
    participant P as 물리 Tool·Motion 시험 Adapter
    loop 미완료 슬롯이 있는 동안
        C->>C: 로봇 쪽 미완료 슬롯 선택
        C->>T: 선택 슬롯의 단일 물리 이송
        T->>P: Approach·수평 Insert·Grasp·Holding·Relation
        T->>P: 지지면 이탈·수평 Extract·수령 위치 이동
        T->>P: Release·Relation 해제·Retreat
        alt 실제 이송·해제·안착과 작업 성공
            T-->>C: 성공 및 물리 관측 근거
            C->>C: 선택 슬롯만 완료, 수량 한 번 반영
        else 실패
            T-->>C: 실패와 관측 근거
            Note over C: 기존 완료 수량 및 미완료 점유 보존, 반복 종료
        end
    end
```

Cell은 위치·지지면·티칭을 소유한다. Robot·Tool·Assembly 설정의 관절과 TCP 의미를 바꾸지 않는다.
슬롯 좌표에 대한 티칭은 선택 기구학의 FK·IK로 검증하고, 실제 경로의 접촉·충돌은 물리 실행으로 검증한다.
수령 위치는 이전 완료 자재가 다음 Tool 삽입 통로를 막지 않는 순서로 사용한다.

## 상태·실패·복원

미완료 슬롯은 선택할 수 있으며, 한 번에 하나만 활성화한다. 완료는 활성 슬롯의 실제 이송과 물리 Release,
수령 위치 안착 및 단일 작업 성공 뒤에만 가능하다. Tool 명령 수락·BT 반환·Relation 생성만으로 차감하지 않는다.
중복 완료는 거부한다. 공급 소진 시 새 작업을 시작하지 않는다.

실패는 물체를 원래 위치로 되돌리지 않고 실행을 종료한다. 공급 점유는 미완료 기록이며 물체의 실제 위치를
보장하지 않는다. 실패 원인과 종료 전 물리 상태·수량을 보존하며 자동 재시도하지 않는다.
Playing 동안 강제 Attach·물체 pose/velocity 재설정·중력 또는 Collision 해제로 성공을 만들지 않는다.

Timeline Stop은 종료 상태 기록 뒤 Cell의 초기 카트리지 배치·물체 속도·공급 상태를 함께 복원한다.
이 복원은 성공 이력을 새 완료 수량으로 이월하지 않는다. Pause는 초기화 조건이 아니다.

## 실행 범위

Cell 선택과 Scene Builder는 기존 Plugin 경계를 사용한다. 현재 등록된 M0609 + C10/Long 실행은
native 공통 Task BT의 물리 시험 Adapter다. 운영 ROS Motion Backend·Mission·Vision·Safety 통합은
지원하지 않는다. ROS Mission에는 정상 Vision 관측이 필요하며 슬롯 좌표를 검출 성공으로 합성하지 않는다.
CNC Cell, 2F-85, 공통 Runtime Workflow의 동작 계약은 유지한다.

## Verification Requirements

| ID | 조건·자극 | 관찰 가능한 기대 결과·금지 결과 |
|---|---|---|
| VR-TAPE-SUPPLY-01 | 기본 17 mm 롤 3개 연속 처리 | 각 슬롯에서 수평 삽입·물리 파지·지지면 이탈·이송·해제·후퇴, 수량 1→2→3 및 소진 |
| VR-TAPE-SUPPLY-02 | 슬롯별 실행 및 실패 | Robot/설비·비핑거/롤·미선택 롤 접촉은 실패, 실패한 슬롯을 완료로 반영하지 않고 이전 수량 보존 |
| VR-TAPE-SUPPLY-03 | 처리 또는 실패 뒤 Timeline Stop | 모든 롤이 초기 슬롯 배치·정지 속도로 복원, 완료 0·전체 미완료, Pause에는 보존 |
| VR-TAPE-SUPPLY-04 | 기본 C10 및 기존 Cell/Tool 회귀 | 기본 C10의 같은 물리 공정과 CNC·2F-85 관련 기존 동작 보존, 미지원 ROS Mission 실행은 명시적으로 거부 |
