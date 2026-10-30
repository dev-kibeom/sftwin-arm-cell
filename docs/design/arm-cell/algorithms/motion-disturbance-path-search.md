# ARM Cell Motion 교란 경로 탐색 Algorithm

- **Algorithm ID:** `motion-disturbance-path-search`
- **Owner:** Task Execution BT (search composition); ARM Cell Motion (physical planning and execution)
- **실현 대상 계약:** [Motion 교란 실행 FDS](../components/motion/fds-disturb-execution.md), [Task Execution–Motion ICD](../interfaces/icd-task-execution-motion.md)

## 목적과 경계

Vision의 미확정 disturbance 후보 주변에서 자재 배치를 조금 바꿀 짧은 경로를 찾는다. 후보 category나 확정 물체 형상을 재해석하지 않으며, 결과는 계획된 로봇 동작의 완료만 뜻한다. 새 관측, 실제 이동 여부 및 다음 PICK 가능성은 Orchestration/Vision이 확인한다.

## 입력과 순서

입력은 하나의 Acquisition identity, 전체 원본 `DisturbanceCandidate` 목록, 기존 MoveIt Planning Scene 및 Motion 운영 Profile이다. 각 후보에서 사용하는 Vision 값은 `center`의 위치와 `observed_height_m`뿐이다. 기존 공통 `CandidatePriorityPolicy::HEIGHT_FIRST`로 후보를 정렬하며 높이 동률은 입력 순서를 보존한다. category는 순위와 경로 생성에 사용하지 않는다.

## 경로 생성 및 탐색

Task Execution BT가 후보 대표점 `p`와 고정 base 평면 방향 `d`에 대해 다음 TCP 위치를 만든다. Vision 대표 Pose의 orientation은 사용하지 않고 목표에는 중립 orientation을 지정한다.

```text
approach = p - approach_clearance * d
push_end = p + push_distance * d
retract  = p + (push_distance + approach_clearance) * d
```

`d`의 결정적 순서는 `+X`, `-X`, `+Y`, `-Y`다. 기본 profile은 `approach_clearance = 0.01 m`, `push_distance = 0.02 m`, `speed_fraction = 0.2`이며, 속도·가속도 scaling은 기존 planning scale 및 현재 Safety envelope에 이 비율을 적용한다. 이 점 경로는 정밀 접촉면 계산이나 실제 밀기 성공을 주장하지 않는다.

각 후보·방향에 대해 모든 구간을 현재 measured start에서 차례로 계획하고 이전 구간의 planned end state를 다음 구간의 start state로 사용한다. 기존 요청별 PLACE Scene 구성을 사용해 Vision 자재 Occupancy를 제외하고, 로봇 자체·Tray·Rail·고정 설비 충돌 검사는 유지한다. 별도 Scene, 식별 불가능한 자재 대상 접촉 예외, 후보 대표점 containment Gate는 추가하지 않는다.

해당 후보·방향의 모든 segment가 계획되면 탐색을 끝낸다. 일부라도 실패하면 실행하지 않고 다음 방향, 이어 다음 후보로 간다. 모든 선택지를 소진하면 실행 전 계획 실패로 종료한다. 전체 계획이 확보된 뒤 기존 `executePlannedMotion()` 경계로 순차 실행한다. 실행 실패·Cancel·Safety 선점 이후 후보를 바꾸지 않는다.

## 검증 요구사항

| ID | 관찰 가능한 기대 결과 |
|---|---|
| `VR-ALGO-DISTURB-01` | 높이 내림차순이며 같은 높이는 Enumeration 순서가 보존된다. category는 순서를 바꾸지 않는다. |
| `VR-ALGO-DISTURB-02` | 네 방향 각각 대표점 기준 접근·접촉 진입·밀기·후퇴 위치를 생성하며 임의의 물체 형상·면·법선을 계산하지 않는다. |
| `VR-ALGO-DISTURB-03` | 한 segment라도 계획에 실패하면 그 계획을 실행하지 않고 다음 방향·후보를 탐색한다. |
| `VR-ALGO-DISTURB-04` | 전체 경로 계획·검증 전 execution worker를 시작하지 않으며 실행 이후 다른 후보·방향으로 전환하지 않는다. |
| `VR-ALGO-DISTURB-05` | Vision 자재 Occupancy를 제외한 기존 요청별 Scene 정책을 사용하고 고정 설비 및 robot 충돌 제한을 보존한다. |
