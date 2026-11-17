# Tape Handling Cell 공급·물리 이송 FDS

- **Function ID:** Tape Cell Cartridge Processing
- **Owner:** Tape Cell Simulation 공급 관리

## 목적과 동작 범위

선택된 Tape Cell의 정렬된 카트리지 자재를 한 슬롯씩 공통 Task Execution에 맡겨 수령 위치로 옮기고,
확인된 물리 이송·해제만 공급 완료 수량에 반영한다. 시작은 명시적인 Cell 실행 요청이며,
종료는 공급 소진 또는 첫 실패다. ROS Mission Admission·Safety 승인·Motion 계획은 해당 Owner가 수행한다. 정렬 슬롯은 Vision 관측과 다른 typed 작업 원천이다.

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
    participant P as 운영 Motion·Physical Tool Runtime
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
수령 위치에 롤 축이 수직으로 안착한 것이 확인된 뒤에만 가능하다. Tool 명령 수락·BT 반환·Relation 생성만으로 차감하지 않는다.
동일 예약의 완료 재확인은 수량을 중복 차감하지 않는다. 공급 소진 시 새 작업을 시작하지 않는다.

실패는 물체를 원래 위치로 되돌리지 않고 실행을 종료한다. 공급 점유는 미완료 기록이며 물체의 실제 위치를
보장하지 않는다. 실패 원인과 종료 전 물리 상태·수량을 보존하며 자동 재시도하지 않는다.
Playing 동안 강제 Attach·물체 pose/velocity 재설정·중력 또는 Collision 해제로 성공을 만들지 않는다.
PhysX의 접촉 오프셋 안에 있는 근접 보고는 모든 접촉점의 간격이 양수이고 충격량이 0으로 확인된 경우에만
실접촉에서 제외한다. 간격 0 이하, 충격량 발생 또는 관측 누락·비유한 값은 보수적으로 접촉으로 처리한다.

Timeline Stop은 종료 상태 기록 뒤 생성된 롤·카트리지를 제거하고 초기의 빈 컨베이어·공급 대기 상태와 새 예약 epoch로 복원한다.
이 복원은 성공 이력을 새 완료 수량으로 이월하지 않는다. Pause는 초기화 조건이 아니다.

## 실행 범위

Cell 선택·Scene Builder·Tool bootstrap은 공통 Runtime Workflow의 Plugin 경계를 사용한다.
M0609 + C10/Long은 정식 Physical Real 모드이며 운영 ROS Backend·Task/Mission으로 실행한다.
Orchestration의 configured source는 [Configured Supply ICD](../../interfaces/icd-configured-supply.md)의 예약을 사용한다.
Vision source의 Acquisition을 생성하거나 검출 성공을 합성하지 않는다. CNC Cell과 2F-85의 동작은 유지한다.

Robot HOME 초기화는 정지된 Scene 준비다. HOME에서 Cell 진입점까지는 Motion이 실제 Scene을 사용해
전역 계획한다. Cartesian 티칭 구간은 구간 FK·관절 제한·시간 매개화·충돌 검사를 통과해야 하며,
관절/Clock 관측으로 실행 결과를 확인한다. Physics Scene의 Attach·중력·충돌 설정을 성공 판정에 맞춰 바꾸지 않는다.

## Verification Requirements

| ID | 조건·자극 | 관찰 가능한 기대 결과·금지 결과 |
|---|---|---|
| VR-TAPE-SUPPLY-01 | 기본 17 mm 롤 15개 보관 및 순차 처리 | 각 슬롯에서 수평 삽입·물리 파지·지지면 이탈·이송·해제·후퇴, 수량 단일 증가 및 전체 소진 |
| VR-TAPE-SUPPLY-02 | 슬롯별 실행 및 실패 | Robot/설비·비핑거/롤·미선택 롤 접촉은 실패, 실패한 슬롯을 완료로 반영하지 않고 이전 수량 보존 |
| VR-TAPE-SUPPLY-03 | 처리 또는 실패 뒤 Timeline Stop | 생성 롤·카트리지 제거, 대기 공급·완료 0·예약 초기화, Pause에는 보존 |
| VR-TAPE-SUPPLY-04 | 기본 C10 및 기존 Cell/Tool 회귀 | 기본 C10의 같은 운영 ROS 물리 공정과 CNC·2F-85 관련 기존 동작 보존, 시험 Adapter 없이 실제 Task/Mission 실행 |
| VR-TAPE-SUPPLY-05 | 정식 Launcher 및 configured-source Mission | 실험 전용 제어 없이 Physical Real 선택, 실제 슬롯 예약·Task·수령 확인, Safety/공급 신선도 손실 시 중단, Vision 성공 합성 금지 |

## 장면 및 수직 Place

Cell은 SDK default Ground Plane을 사용하지 않는다. 초록색 에폭시 바닥과 로봇 지지대의 상면·충돌 형상은 Cell 설정에서 소유한다. Robot base는 지지대 상면이며 임의의 runtime 위치 보정으로 경로를 성공 처리하지 않는다.

카트리지는 15개 롤을 한 줄로 정렬하며 슬롯 수·피치를 설정한다. 공통 Hub의 Supply 입력 단위는 가득 찬 카트리지 수다. Cell이 요청을 수락하면 대기 카트리지를 한 개씩 실제 인피드 벨트 위에 생성한다. 유한 힘의 선형 구동 체인이 동적 카트리지를 이동시키고 롤은 V자 받침·포켓과의 접촉·마찰로 이동한다. 롤 자체의 pose/velocity를 써서 정렬하지 않는다.

비전 없이 작업하므로 컨베이어가 각 슬롯을 같은 고정 Pick 기준점에 순차 정렬한다. 구동 목표가 고정된 뒤 실제 카트리지·롤의 위치·축·속도가 정착 조건을 만족해야 공급을 ready로 발행한다. 해당 epoch의 유효 공급 상태를 ROS에 발행한 뒤 Mission 요청을 허용한다. Pick 및 Extract는 수평이며 운반 통로에서 아래 방향으로 전환한 후 고정 수령점으로 수직 하강·Release·수직 Retreat한다. 수령점의 두 벨트는 내경을 열어 두고 외주를 지지한다. Tool 길이에 따른 수령 높이·회전 지점은 Cell 설정으로 정하며 실제 선택 기구학을 사용한다.

실제 Mission 성공과 물리 안착 수량이 일치한 뒤에만 아웃피드 벨트를 구동한다. 해제된 롤이 다음 Place 통로를 벗어난 것을 측정한 뒤 다음 슬롯을 인덱싱한다. 마지막 롤 처리 후 빈 카트리지를 인피드로 반환·회수하고 다음 가득 찬 카트리지를 공급한다. 생성·회수는 Cell이 소유하며 미완료 롤이나 실패 상태를 삭제해 성공 처리하지 않는다. 첫 실패에는 자동 재시도하지 않는다.

설정 이동 시간은 최소 구간 시간이다. 선택된 Robot의 관절 속도·가속도 한계를 지키도록 구간 시간을 늘리며, 시간 조정으로 기하 경로를 바꾸지 않는다. 긴 Tool의 자세 전환은 실제 장착부 위치를 유지하는 원호를 사용해 불필요한 큰 관절 회전을 피한다.
