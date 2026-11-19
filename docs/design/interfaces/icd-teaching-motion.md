# ARM Cell Teaching Motion 인터페이스 계약

- **Contract ID:** ARM-CELL-TEACHING-MOTION
- **Semantic Owner:** ARM Cell Shared Interface
- **Consumers:** Cell 티칭 Owner, UI Hub, Motion

## 목적과 경계

설비 기준 티칭을 실행 가능한 TCP 이동으로 전달하고 적용 응답과 실제 이동 결과를 구별한다.
포인트·배치·저장은 Cell, 운영 편집은 [UI Hub FDS](../components/ui-hub/fds.md),
계획·실행·Safety 집행·완료는 [Motion FDS](../components/motion/fds-motion-execution.md)가 소유한다.

## 참여 인터페이스

| 인터페이스 | Producer → Consumer | 공유 의미 |
|---|---|---|
| `/motion/configure_teaching` | Cell → Motion | 전체 Named Motion Registry의 유휴 교체 요청·응답 |
| `/motion/execute_teaching` | Cell 수동 조작 → Motion | 단일 수동 이동 요청·Action 취소·종료 결과 |

Registry 교체 성공은 정의가 적용됐다는 뜻이며 IK·경로·물리 실행 성공이 아니다.
유휴가 아니거나 Backend 정지가 확인되지 않으면 교체하지 않는다. 교체와 Motion 예약은
원자적으로 직렬화되어 이전·새 정의가 하나의 movement에 혼합되지 않는다.

## 데이터와 결과 의미

| 데이터 | 의미·단위·유효성 |
|---|---|
| `cartesian_waypoints` | 순서가 있는 희소 TCP 목표. 위치는 base_link 기준 m, 자세는 정규 quaternion xyzw |
| `mode` | `MoveJ`는 TCP 목표의 IK 후 관절 이동, `MoveL`은 TCP 직선 구간. 기본 환경 충돌 검사·자동 우회 없음. 명시적 Checked/Planned 이동과 구별하며 실패 fallback 금지 |
| 속도·가속도 비율 | Robot 한계 대비 `(0,1]`. 수동 Action에서는 각각 최대 0.1로 제한하며 Safety envelope를 추가 소비 |
| 수동 Action identity | ROS Goal UUID가 이동·취소의 상관관계. UI의 로컬 요청 수락과 Motion 수락은 구별 |
| 수동 결과 | 기존 MotionTaskResultCode와 설명. Action 종료 상태와 함께 해석하며 그리퍼·자재 처리 완료를 뜻하지 않음 |

빈 정의, 비유한 값, 잘못된 quaternion, mode, 비율은 무효다. 수동 요청은 TCP 이동만 수행하며
그리퍼를 열거나 닫지 않는다. 기존 Safety·MotionStatus·StopMotion 의미를 재정의하지 않는다.

## 검증 조건과 의존성

| ID | 조건 | 소비자가 보존할 의미 |
|---|---|---|
| VR-TEACH-ICD-01 | 정의 교체와 실행 요청 경합 | 한 movement가 하나의 Registry 정의만 소비하며 비유휴 교체를 거부 |
| VR-TEACH-ICD-02 | 수동 이동 및 취소 | 같은 Motion 예약·Safety·취소 경로와 기존 결과 코드를 사용 |
| VR-TEACH-ICD-03 | 프레임·모드·비율이 잘못된 입력 | 실행을 시작하지 않고 오류로 반환 |

설비 프레임을 base_link 목표로 변환하는 책임은 Cell 티칭 Owner에 남는다.
[Configured Supply ICD](icd-configured-supply.md)의 예약과 자재 수량 의미는 유지한다.
