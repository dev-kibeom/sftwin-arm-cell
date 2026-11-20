# PLC 단일 작업 요청·응답 계약

- **Contract ID:** ARM-CELL-PLC-TASK
- **Owner:** PLC Adapter (요청 상관 및 결과 전달)
- **참여자:** PLC, PLC Adapter, Configured-source Orchestration

PLC는 작업 요청을 유지하고 Adapter는 같은 `command_id`의 마지막 관측을 반환한다. Adapter는 [Configured Supply Mission](icd-configured-supply.md)에 설정된 자재, 수량 1, zero Delivery UUID를 제출한다. Goal 수락·실행·terminal은 Orchestration, 슬롯 예약·퇴역은 Configured Supply, 장치 해제 사실은 Task Execution에서 온다. 설비 Gate·Cell Commit·실제 슬롯 점유·Robot Ready의 판단은 이 인터페이스 밖에 있다.

## 요청과 결과

| 필드 | PLC가 해석하는 의미 |
|---|---|
| 요청 ID | 단일 PLC 요청 스트림에서 1부터 연속 증가하는 부호 없는 32비트 값. 0은 마지막 상태 조회 및 신규 스트림의 초기 유휴 상태다. PLC는 미확정 ID를 재시작 후에도 보존한다. |
| 응답 ID | 조회한 ID의 기록. 새로운 요청을 제출할 수 없으면 현재 마지막 기록을 반환한다. PLC는 자신의 ID와 일치하는 응답만 적용한다. |
| 상태 | 유휴(0), 제출 기록(1), ROS 수락(2), ROS 실행 피드백 관측(3), 성공(4), 취소(5), 중단(6), 명시적 거부(7), 실행 여부 또는 결과 미확정(8). 폴링 사이에 중간 상태를 건너뛸 수 있다. 제출은 수락을 보장하지 않는다. |
| terminal 사유 | ROS terminal에서 받은 `MissionExitReason.value`. 거부·미확정 등 Mission 결과가 없으면 0이다. |
| 배치 사실 | 0은 정보 없음, 1은 관측된 Task 결과의 `material_placed=false`, 2는 `true`다. Mission 실패와 배치 사실이 동시에 성립할 수 있다. |
| heartbeat | 응답 블록마다 순환 증가하는 16비트 값. PLC는 변화와 수신 경과시간으로 링크 신선도를 판단한다. |

PLC는 같은 ID를 유지해 수락·진행·terminal을 조회한다. 결과를 소비한 후 다음 자재에 새 ID를 부여한다. 요청·응답 블록은 각각 일관된 한 프로세스 이미지로 교환한다. 이전 ID의 재조회는 해당 기록만 반환한다. 한 스트림에서 ID를 순환·재사용하지 않는다.

## 단절과 재시작

Adapter는 제출 전에 ID와 ROS Goal UUID를 영속화하고 결과를 저장한 뒤 PLC에 전달한다. PLC 링크 단절은 실행 취소 사유가 아니다. 재접속 시 유지된 요청으로 현재 기록을 다시 전달한다. Adapter 재시작 시 진행 중이던 ID의 terminal을 같은 UUID로 ROS Action 서버에 조회한다. 서버가 결과를 보존하지 않았거나 제출 여부를 확인할 수 없으면 미확정 상태를 유지하며 후속 작업 제출을 보류한다.

PLC는 응답 블록이 오래되었거나 ID가 다르면 공정 판단을 보류한다. PLC의 보존 ID와 Adapter 기록은 동일 설비 스트림에 속해야 한다. 기록 유실·ID 소진·설비 교체 시에는 잔류 작업·자재와 PLC의 인수 기록을 대조한 후 양측에서 새 스트림을 초기화한다. Cell Commit 기록은 PLC에 보존한다.

## 검증 조건

| ID | 소비자가 관찰할 수 있는 결과 |
|---|---|
| VR-PLC-01 | 지연 또는 동일 ID 반복 시 한 ROS Goal만 제출되고 응답 ID는 제출한 작업과 일치한다. |
| VR-PLC-02 | 단절 중 완료된 작업은 재접속 후 원래 terminal 사유와 배치 사실을 반환한다. |
| VR-PLC-03 | 재시작 시 보존된 terminal 또는 같은 Goal의 조회 결과만 복원한다. 불확실한 작업은 재실행하지 않는다. |

Modbus TCP 블록과 장치별 구성 예시는 [Adapter 실행 안내](../../../ros2_ws/src/plc_adapter/README.md)에 있다.
