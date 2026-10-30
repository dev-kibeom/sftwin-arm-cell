# [SF-Twin] ARM Cell Motion 교란 실행

- **Function ID:** `ARM-MOTION-DISTURB`
- **Owner:** `ARM Cell Motion`

## 목적과 동작 범위

MotionPort로 전달된 하나의 Acquisition 교란 후보 집합에서 실행 가능한 짧은 밀기 동작을 계획하고 실행한다.

- **시작 경계:** Task Execution이 원본 Acquisition identity와 전체 교란 후보를 포함한 `executeDisturb` MotionPort operation을 제출한 시점.
- **종료 경계:** 측정된 접근·밀기·후퇴 완료, 계획 후보·방향 소진, 실제 실행 실패, 요청자 Cancel 또는 Safety direct Stop의 원인과 operation 결과가 확정된 시점.
- **비책임:** 미확정 형상의 판정, PICK 후보·Mission 수량·재관측·재시도 정책, 물체 이동량이나 다음 PICK 가능성의 확인.

## 참여 책임과 계약

| 참여 책임 | 이 기능에서의 역할 | 정식 계약·Owner |
|---|---|---|
| Task Execution | Action identity와 Vision 후보 원본을 MotionPort에 전달하고 operation 결과를 상위 Action 결과로 매핑 | Task Execution CDS 및 Orchestration–Task Execution ICD |
| Task Execution BT | `HEIGHT_FIRST` 후보 순서, 방향별 계획 실패 탐색 및 작업 SubTree 조합 | Task Execution CDS 및 이 FDS |
| Motion runtime | 대표점 기반 목표 계산 지원, 요청별 Scene 적용, 구간 계획·실행 lifecycle, Motion 결과 확정 | Motion CDS 및 이 FDS |
| Planning Scene / planner | 접근·밀기·후퇴의 계획 가능성과 로봇 충돌 제약 평가 | Motion CDS 및 기존 Motion planning 계약 |
| Backend / measured-state observer | 궤적 실행과 측정 움직임·software stop 완료 근거 제공 | Motion Backend ICD 및 Safety–Motion ICD |
| Safety | 기존 권한, direct Stop, 원인·정지 모드 결정 | Safety FDS 및 Safety–Motion ICD |

## 핵심 시나리오

```mermaid
sequenceDiagram
    autonumber
    participant T as Task Execution
    participant M as Motion runtime
    participant P as Planning Scene / planner
    participant B as Backend / measured state
    participant S as Safety

    T->>M: MotionPort.executeDisturb(identity, 전체 후보)
    Note over T,S: 요청자 Cancel 또는 Safety direct Stop은 어느 비terminal 단계에서도 발생 가능
    T->>T: 공통 HEIGHT_FIRST 후보 순서 적용
    loop 실행 가능한 전체 계획 확보 전, 검토할 후보·방향이 남아 있는 동안
        T->>T: 다음 후보와 허용 방향 선택
        M->>P: 접근·밀기·후퇴 전체 계획 검증
        alt 한 구간 이상 계획 불가
            P-->>M: 계획 실패 근거
            Note over M: 해당 방향은 제외; 실행 전 다음 방향·후보 검토
        else 전체 동작 계획 가능
            P-->>M: 유효한 계획 묶음
            Note over M: 계획을 선택하고 후보·방향 탐색 종료
        end
    end
    alt 실행 가능한 계획 없이 모든 후보·방향 소진
        M-->>T: FAILED / PLAN_EXHAUSTED
    else 실행 가능한 전체 계획 확보
        M->>B: 접근·의도된 밀기·후퇴 순서 실행
        Note over T,S: 실행 중 Cancel과 Safety 선점은 서로 독립적으로 발생할 수 있음
        par 요청자 취소 경로
            opt 요청자 Cancel 발생
                T->>M: cancelMotion(동일 활성 identity)
            end
        and Safety 직접 선점 경로
            opt Safety 선점 발생
                S->>M: 기존 direct StopMotion
            end
        end
        B-->>M: 단계별 측정 완료·실행 실패 또는 software stop 완료 근거
        alt Motion이 operation을 끝내기 전에 Safety 선점이 유효함
            M->>B: 기존 Safety 정지 전파·완료 관측
            M-->>T: SAFETY_PREEMPTED, 원인·software stop 근거
        else Motion이 Cancel에 따라 실행을 중단함
            M->>B: 기존 Backend 정지·완료 관측
            M-->>T: CANCELED 및 software stop 근거
        else 실제 실행 실패가 먼저 확정됨
            M-->>T: FAILED / EXECUTION_FAILURE 및 근거
            Note over M: operation terminal; 다른 후보·방향으로 전환하지 않음
        else 실행 완료가 먼저 확정됨
            M-->>T: COMPLETED
            Note over M,T: 이후 도착한 Cancel은 확정된 operation 결과를 바꾸지 않음
        end
    end
```

Cancel과 Safety 선점은 계획 단계에서도 발생할 수 있으며, 어떤 비terminal 단계에서도 현재 operation에 영향을 줄 수 있다. 취소 요청의 수락은 중단 완료가 아니다. Motion은 실제 operation 결과를 결정하며, 이미 확정한 완료나 실행 실패를 늦은 Cancel로 재분류하지 않는다. Cancel에 따라 실제 중단됐을 때만 `CANCELED`를 결과화한다. 실행 중 Cancel과 Safety가 함께 발생하거나 경합하면 Safety의 원인과 direct Stop의 software stop 근거를 보존해 `SAFETY_PREEMPTED`로 확정한다. Terminal 확정 뒤의 Cancel은 기존 정지 근거를 만들거나 결과를 바꾸지 않는다.

Task Execution BT는 Motion의 공통 `CandidatePriorityPolicy::HEIGHT_FIRST` 정렬을 사용해 모든 후보를 `observed_height_m` 내림차순으로 검토하며 높이가 같으면 입력 Enumeration 순서를 유지한다. 후보 `center`는 Height Grid의 관측 대표점이지 물체 중심이나 확정 접촉점이 아니다. SubTree는 대표점 위치와 네 고정 base 평면 방향으로 짧은 경로 목표를 만든다. 후보 분류, 물체 면·법선·크기·정확한 접촉점은 경로 생성에 사용하지 않는다. Vision 대표 Pose의 Orientation은 사용하지 않으며 각 목표는 중립 Orientation으로 설정한다. 접근 여유·밀기 거리·속도 비율은 Motion 운영 Profile에서 제공한다. 후보 면적 하한, 공칭 객체 치수 Gate 또는 대표점이 특정 CollisionObject 안에 있어야 한다는 Gate를 두지 않는다. 알고리즘은 [Motion disturbance path search](../../algorithms/motion-disturbance-path-search.md)를 따른다.

한 후보·방향의 접근·의도된 밀기·후퇴 전체가 계획되지 않으면 실제 동작을 시작하지 않고 다음 방향이나 후보를 검토한다. 실행 가능한 전체 묶음을 확보하면 후보 탐색을 끝내고 해당 묶음을 한 번 실행한다. 모든 적격 후보·방향을 검토해도 전체 계획을 확보하지 못한 경우에만 `PLAN_EXHAUSTED`로 종결한다. 실행을 시작한 뒤 어느 단계에서든 Backend 실행 실패나 측정 완료 조건 실패가 발생하면 해당 operation을 `EXECUTION_FAILURE`로 종결한다. 이미 시작한 실행 실패를 감추기 위해 다른 후보·방향으로 전환하지 않는다. 계획만으로 `COMPLETED`를 반환하지 않는다.

Planning Scene은 #679의 기존 요청별 정책을 사용한다. 교란은 독립 Scene을 만들거나 공유 Scene을 갱신하지 않으며, Vision 자재 Occupancy를 제외하는 PLACE 계획 요청을 사용한다. 이에 따라 교란 영역의 자재 점유와 충돌하지 않는 경로를 평가할 수 있지만, 식별되지 않은 자재에 대한 개별 접촉 예외를 만들지는 않는다. 기존 로봇 자체, Tray, Rail 및 고정 설비 충돌 검사는 유지한다. 고정 설비를 포함한 현재 Scene 표현이 필요한 경로를 허용하지 않으면 해당 계획은 실패한다.

| 구간 | 허용되는 의도적 접촉 | 충돌 제약 |
|---|---|---|
| 접근·밀기·후퇴 | 교란 영역 자재와의 의도적 접촉 | Vision 자재 Occupancy를 제외한 기존 요청별 Scene으로 계획한다. 로봇 자체·Tray·Rail·고정 설비 충돌 검사는 유지한다. |

모든 구간에서 Robot self-collision, Tray, Rail 및 고정 설비 충돌 제한은 유지한다. PnP 충돌 정책은 변경하지 않는다. 교란은 개별 자재와 접촉 경로의 충돌을 구분하지 못하므로 Vision 자재 Occupancy 제외 정책으로 계획한다. 이 동작은 접촉 위치의 정밀도나 자재 이동을 보장하지 않는다.

## 시간·취소·Safety 선점

| 사건 | Motion 처리 및 결과 | 정지 근거 |
|---|---|---|
| Motion operation 시작 전 명시적 Cancel | 실행을 시작하지 않고 `CANCELED`로 종료할 수 있음 | Motion 실행이 시작되지 않았으면 Backend 정지 근거 없음; `backend_inactivity_confirmed`를 추론하지 않음 |
| Task Execution Cancel, Motion operation 진행 중 | 동일 활성 identity의 실제 중단을 수행한 경우 `CANCELED`; 실행 결과가 이미 확정된 경우 그 결과 유지 | 요청 수락과 구분되는 기존 정지 완료 조건 및 Motion 근거 |
| Cancel 수락 후 실제 중단 지연 | operation은 미확정으로 유지; 실제 중단·terminal 근거까지 기다림 | 수락만으로 Backend 비활성 아님 |
| Motion이 `COMPLETED` 또는 `EXECUTION_FAILURE` 확정 후 Cancel 도착 | 이미 확정된 Motion 결과 유지 | 후속 Cancel은 완료/실패를 소급 재분류하거나 새 stop evidence를 만들지 않음 |
| Safety direct Stop 선점 | Safety가 시작한 정지를 수행하고 `SAFETY_PREEMPTED`로 반환 | StopMotion 수락과 별개인 기존 Motion software stop 완료·비활성 근거 |
| Cancel과 Safety 선점 경합 | terminal 확정 시 확인된 Safety 선점 원인을 보존하고 Safety 결과를 우선 | Safety 직접 요청 및 Motion software stop 완료 근거를 연결 |

경합은 아래 결정 규칙으로 확정한다. `확정`은 Motion이 operation terminal 결과를 정한 시점을 뜻하며 Cancel 요청의 도착 시각이나 응답 순서를 대신 사용하지 않는다.

| 경합 시나리오 | Motion operation 결과 확정 규칙 | Action terminal·증거 |
|---|---|---|
| 완료 직전 Cancel | Motion이 정상 측정 완료를 먼저 확정하면 `COMPLETED`; 완료 전에 Cancel에 의해 실행이 실제 중단됐으면 `CANCELED` | 각각 성공 또는 취소 terminal; Cancel로 완료를 덮지 않음 |
| 완료 확정 뒤 늦은 Cancel | 기존 `COMPLETED` 유지; 비활성 작업에 대한 Cancel은 실제 실행 결과를 변경하지 않음 | `COMPLETED` terminal 불변; 늦은 Cancel은 새 stop evidence 아님 |
| 실제 실행 실패와 Cancel 경합 | Motion이 실행 실패를 확인·확정하면 `EXECUTION_FAILURE`; 실패 확정 전에 Cancel로 실제 중단된 경우에만 `CANCELED` | 실패 분류·원인 보존; Cancel 요청만으로 실패를 취소로 변환하지 않음 |
| Cancel 수락 뒤 Backend 중단 지연 | operation은 미확정으로 유지하고 Motion이 중단 완료 또는 실제 결과를 확정할 때까지 대기 | Cancel 수락은 ROS Action terminal도 `backend_inactivity_confirmed`도 아님 |
| operation 시작 전 Cancel | 실행을 시작하지 않고 요청자 취소로 종료 가능 | `CANCELED` 가능; Backend 중단 근거 없을 수 있으며 비활성을 만들지 않음 |
| Cancel과 Safety direct Stop 동시 발생 | Motion terminal 전에 Safety가 선점하면 `SAFETY_PREEMPTED`; Safety 원인·stop 근거를 보존 | 요청자 Cancel로 강등 금지; 실행 실패도 함께 확인되면 진단에 실제 원인 보존 |
| Safety stop이 Motion terminal 뒤 도착 | 이미 확정된 operation 결과 유지; Safety는 기존 Safety lifecycle에서 처리 | terminal 소급 변경 금지; 새 Safety stop 근거를 과거 operation에 붙이지 않음 |

Cancel 요청 수락이나 StopMotion 요청 수락만으로 Backend 비활성 또는 terminal 완료를 의미하지 않는다. 실행 전 Cancel은 실제 실행이 시작되지 않았다는 lifecycle 사실로 취소할 수 있지만, Backend 정지 사실이 아니므로 `backend_inactivity_confirmed`를 만들지 않는다. 기존 software stop 완료는 이전 명령 생산자·Action 종료와 Stop/Hold 전달 완료 및 `backend_inactivity_confirmed` 관측을 뜻하며, 관절이 완전히 정지했거나 Hardware Safety가 확인됐다는 의미가 아니다. Task Execution은 Cancel을 MotionPort로 전달할 수 있지만 Safety 정지를 요청하거나 정지 모드·원인을 선택하지 않는다. 정지 근거 생성은 Motion/Backend가, Safety 원인·모드는 Safety가 소유하며, Task Execution은 이 원인을 외부 Action 결과에서 구별해 보존한다. 하나의 Action Goal의 terminal outcome은 Motion의 확정 결과를 반영해 단 한 번 확정하며 소급 변경하지 않는다.

## 결과·오류·관측

| 조건 | 감지 책임 | MotionPort 결과 | 상위 전파·비보장 |
|---|---|---|---|
| 모든 계획 구간 및 측정 실행 완료 | Motion / Backend | `COMPLETED` | 명령된 동작의 완료만 뜻하며 물체 이동·재배치·PICK 가능성을 보장하지 않음 |
| 모든 후보·방향에서 전체 동작 계획 불가 | Motion planner | `FAILED`, `PLAN_EXHAUSTED` | 실제 동작을 시작하지 않은 계획 소진 근거 |
| 계획 이후 실제 궤적·측정 완료 실패 | Backend / Motion | `FAILED`, `EXECUTION_FAILURE`와 원래 근거 | 후보 전환으로 분류하거나 은폐하지 않음 |
| 명시적 요청자 Cancel | Task Execution 요청, Motion 실행 lifecycle | `CANCELED`와 기존 software stop 완료 근거 | 성공으로 해석하지 않음; 물리적 완전 정지를 뜻하지 않음 |
| Safety direct Stop 선점 | Safety가 원인 결정, Motion/Backend가 정지 수행 | `SAFETY_PREEMPTED`와 Safety 원인·기존 software stop 완료 근거 | 일반 Cancel 또는 계획 실패로 합치지 않음; Hardware Safety 완료를 뜻하지 않음 |

Motion terminal 결과는 해당 operation의 정규 결과다. `MotionStatus`는 현재 동작·정지 상태를 별도로 관측하며 terminal 결과를 대신하지 않는다. 이 기능은 자재 수량·Holding·배치 사실을 갱신하지 않고 후속 Acquisition을 요청하지 않는다.

## Verification Requirements

### VR-MOT-DISTURB-01 — 높이순 후보 및 계획 탐색

Motion은 모든 `disturbance_candidates`를 `observed_height_m` 내림차순, 동률에서 입력 Enumeration 순서로 검토해야 하며(SHALL), category 기반 순위를 적용해서는 안 된다(SHALL NOT). 네 고정 평면 방향 각각에 대해 접근·접촉 진입·밀기·후퇴 전체 경로를 계획하고, 하나라도 계획할 수 없으면 다음 방향 또는 후보를 검토해야 한다(SHALL). 후보 면적·형상 또는 CollisionObject containment를 실행 Gate로 추가해서는 안 된다(SHALL NOT). 모든 계획을 소진하면 `PLAN_EXHAUSTED`를 반환해야 한다(SHALL).

### VR-MOT-DISTURB-02 — 계획과 실제 실행 실패 구분

계획된 동작 묶음을 실행한 뒤 실제 실행 실패가 발생하면 다른 후보·방향으로 재분류하지 않고 원래 실행 실패 근거와 함께 `EXECUTION_FAILURE`를 반환해야 한다(SHALL). 모든 접근·밀기·후퇴 구간의 기존 측정 완료 조건을 충족한 경우에만 `COMPLETED`를 반환해야 한다(SHALL). 물체 이동·Holding·PICK 가능성 검증을 완료 조건에 추가해서는 안 된다(SHALL NOT).

### VR-MOT-DISTURB-03 — Cancel·Safety 정지 근거

요청자 Cancel과 Safety direct Stop은 서로 구별되어야 하며(SHALL), terminal 결과는 각각 `CANCELED`와 `SAFETY_PREEMPTED` 및 해당 원인을 보존해야 한다(SHALL). 어느 정지 요청의 수락만으로 `backend_inactivity_confirmed`를 확정해서는 안 된다(SHALL NOT). 자재 간 의도된 교란 접촉을 허용하더라도 Robot self-collision·Tray·고정 설비 충돌 제한은 유지되어야 한다(SHALL).

### VR-MOT-DISTURB-04 — 요청별 Scene 및 고정 설비 충돌

교란은 기존 요청별 Scene 구성 정책을 사용하고 Vision 자재 Occupancy를 제외해야 한다(SHALL). 별도 Scene이나 교란 전용 ACM 예외를 만들어서는 안 된다(SHALL NOT). Robot self-collision, Tray, Rail 및 고정 설비 충돌 검사는 유지해야 한다(SHALL).

### VR-MOT-DISTURB-05 — 결과·취소 경합의 단일 확정

Cancel이 Motion operation 시작 전에 도착하면 실행을 시작하지 않고 취소할 수 있어야 하며(SHALL), 이 경우 Backend 비활성 근거를 꾸며내서는 안 된다(SHALL NOT). Cancel 수락 후 실제 중단 지연 중에는 operation terminal을 미리 확정하지 않아야 한다(SHALL). Motion이 `COMPLETED` 또는 `EXECUTION_FAILURE`를 확정한 뒤 도착한 Cancel은 해당 결과를 재분류해서는 안 된다(SHALL NOT). 실제 실행 중단을 Motion이 확정한 경우에만 `CANCELED`를 반환해야 한다(SHALL). Safety와 Cancel이 경합하면 Safety 원인·정지 근거가 보존되어야 하며 한 Goal당 terminal은 한 번만 확정되어야 한다(SHALL).

### VR-MOT-DISTURB-06 — 전체 계획 후 실행

선택한 접근·접촉 진입·밀기·후퇴 경로 전체를 계획·time-parameterize한 뒤에만 execution worker를 시작해야 한다(SHALL). 실행이 시작된 뒤에는 실패나 중단이 발생해도 다른 후보 또는 방향으로 전환해서는 안 된다(SHALL NOT). 기존 측정 joint-state completion oracle을 만족할 때만 `COMPLETED`를 반환해야 하며(SHALL), 자재 이동 또는 후속 PICK 가능성을 완료 의미로 주장해서는 안 된다(SHALL NOT).

## 참조

- [Motion CDS](cds.md)
- [Task Execution–Motion ICD](../../interfaces/icd-task-execution-motion.md)
- [Motion Execution FDS](fds-motion-execution.md)
- [Stop and Recovery FDS](fds-stop-recovery.md)
- [Safety–Motion ICD](../../interfaces/icd-safety-motion.md)
- [Motion Backend ICD](../../interfaces/icd-motion-backend.md)
