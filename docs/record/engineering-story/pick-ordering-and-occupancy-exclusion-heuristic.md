# PICK 작업 순서로 삽입·후퇴 구간의 자재 간섭 가능성을 낮춘 휴리스틱

*Engineering Story — 2026-10-30 개발자 회고 및 현재 Motion/Task Execution 설계, PR 이력을 연결한 역사적 판단 기록. 현재 충돌 검사 계약이나 Safety 보증을 대체하지 않는다.*

## 문제와 의도

밀집된 Tray의 자재를 PICK할 때 주변 자재를 Planning Occupancy로 전 구간에 적용하면, 실제 작업이 가능한 삽입·후퇴도 충돌 근사나 궤적 제약 때문에 실패할 수 있다. 반대로 단순히 모든 검사를 없애면 주변 자재와의 간섭을 고려하지 못한다.

설계자가 2026-10-30 회고에서 설명한 선택은 **PICK 삽입·후퇴에서 다른 자재에 대한 동적 충돌맵 검사를 적용하지 않고 작업 성공률을 확보하되, 자재를 처리하는 순서로 후속 간섭 가능성을 줄이는 휴리스틱**이다. 별도 실시간 충돌회피 알고리즘을 만드는 대신, Tray의 로봇 쪽에서 바깥쪽으로 순차적으로 자재를 처리하도록 우선순위를 정했다.

이는 간섭이 절대로 발생하지 않는다는 증명이 아니다. **검사 완화와 순서 휴리스틱을 함께 사용해 정상 작업 가능성을 확보**하려는 실용적 선택이다.

## 구체적인 정책과 소유권

현행 [PICK Candidate Search Algorithm](../../design/algorithms/motion-pick-candidate-search.md)은 한 Acquisition의 후보를 `base_link` Y 내림차순으로 정렬한다. 현재 설치 좌표계에서 Tray의 +Y는 로봇 쪽이므로 **로봇 쪽에 가까운 자재를 먼저 처리하고 점차 Tray 바깥쪽으로 진행**한다. Y가 같으면 관측 상판 Z 내림차순, 다시 같으면 Candidate ID 오름차순이다. 이 순서는 파지 성공이나 궤적 충돌 안전성을 보장하는 판정이 아니라 작업 우선순위다.

현행 [Motion Execution FDS](../../design/components/motion/fds-motion-execution.md)는 후보에서 만들어진 다른 자재의 임시 Occupancy를 PICK Approach와 Grasp 검증의 적용 목적에 맞게 사용하지만, **Cartesian Descent/Insertion과 Grasp 이후 Lift/Retract 및 PLACE에는 적용하지 않는다.** Robot Self Collision, Tray 및 정적 Fixture의 적절한 충돌 검사는 유지된다. 즉, '삽입·후퇴 충돌 무시'는 **모든 충돌과 Safety를 끈다**는 뜻이 아니라 *다른 검출 자재에서 파생된 동적 Occupancy를 해당 구간에서 제외한다*는 뜻이다.

충돌맵의 목적도 현장 Safety 인증이 아니라 다른 검출 자재의 상태를 불필요하게 손상하지 않도록 하는 데 있었다. 관련 관측 Geometry/Occupancy 책임 변화는 [Vision→Motion Occupancy 기록](vision-to-motion-occupancy-ownership.md)에 따로 남아 있으며, 본 기록은 **구간별 사용 정책과 작업 순서의 조합**에 집중한다.

## 판단의 발전과 근거

- PR #583: 높이 우선 및 Tray 안쪽 우선의 초기 후보 순서 변경. 당시부터 이 정렬은 IK·충돌 검증을 대체하지 않는 휴리스틱으로 설명되었다.
- PR #652: 후보 순서를 더 단순하게 `base_link` Y 우선 → 관측 Z → ID 기준으로 변경. 좌표계상 Tray +Y가 로봇 방향이라는 근거를 제시했다.
- PR #679: 재사용 가능한 Planning/Execution API 분리 시에도 요청별 Scene의 목적에 따라 PICK 구간의 Occupancy 적용/제외 정책을 보존했다.
- PR #715: Grasp 끝점에 대해 독립적인 Occupancy 검증 전략을 제공했다. 이는 삽입·후퇴의 Occupancy 제외와 다른 판단 경계다.

## 효과·제약 및 교훈

이 접근은 Motion이 모든 이동 단계에서 주변 자재 충돌을 엄격히 해석해 작업을 중단하는 대신, 필요한 곳에만 동적 점유 정보를 적용하고 처리 순서로 간섭 가능성을 낮춘다. 실제 파지 성공률 개선은 설계자의 경험적 취지에 부합하지만, 동일 조건의 전후 비교나 충돌 사고 발생률 같은 정량 자료를 이번 기록에서 확보한 것은 아니다.

**적용 범위와 잔여 간섭 가능성:** 인접·적층 배치, 비정상적인 Grasp 자세, 실제 후퇴 경로의 기하학, 물체 접촉·이동 및 센서 오차에 따라 간섭은 여전히 발생할 수 있다. 따라서 순서가 물리적 충돌 검사 또는 실기 장비의 안전 제어를 대체한다고 주장하지 않는다.

재사용할 판단은 **알고리즘으로 모든 기하학적 상호작용을 해결하려 하기보다, 작업 도메인과 처리 순서를 이용해 문제를 단순화할 수 있다**는 것이다. 하지만 단순화가 의존하는 환경 가정과 검증하지 않는 위험은 명시해야 한다.

## 구현 근거와 미측정 효과

현재 문서와 PR은 우선순위 및 구간별 Occupancy 적용의 구현·설계 근거를 제공한다. 이번 기록 작성 시 신규 시뮬레이션 Live Test, 물리 충돌 측정 또는 비교 성능 실험은 수행하지 않았다. 따라서 성공률 향상 정도, 모든 적재 패턴의 무간섭성, 실제 로봇 안전성은 미확인이다.

## 참조

- [PICK 후보 탐색 Algorithm](../../design/algorithms/motion-pick-candidate-search.md)
- [Motion Execution FDS](../../design/components/motion/fds-motion-execution.md)
- [Vision→Motion Occupancy 책임 기록](vision-to-motion-occupancy-ownership.md)
- PR #583, PR #652, PR #679, PR #715
