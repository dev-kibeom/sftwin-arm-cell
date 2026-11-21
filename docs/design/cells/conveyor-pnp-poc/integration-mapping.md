# Conveyor PnP Cell — Integration Mapping Design

- **대상:** `Conveyor Equipment ↔ SoftPLC ↔ PLC Adapter ↔ Robot Mission/Task ↔ History`
- **Owner:** 각 Adapter의 변환 책임. Source의 결과 Authority는 이전하지 않는다.

## 의미 매핑

| Source | Target | 변환 및 식별 | Owner·비보장 |
|---|---|---|---|
| 실제 공급/센서 관측 | PLC 설비 IR/HR | 생성 번호, Pick 감지, 정지, 스토퍼/트레이 점유를 레지스터 표현으로 변환 | Equipment가 물리 사실을 판단. Bit 수신은 Commit 아님 |
| Isaac 런처의 PLC 통신 상태와 신선한 Safety 정상 운전 권한 | Conveyor 시뮬레이션 설비 허가 | 현재 레지스터 교환 성공과 유효한 정상 운전 권한을 함께 관측하면 허가, 입력 만료·통신 실패 시 보류 | 런처가 시뮬레이션 발행자를 관리한다. 실제 설비의 허가는 해당 설비 제어계가 소유한다 |
| PLC `requested` | PLC Adapter 영수증 | `command_id`로 ROS Mission Goal UUID와 결과를 지속 연결 | Adapter가 중복 요청을 재실행하지 않음. 요청 ID는 자재 ID와 1:1이 아닐 수 있음 |
| Supply RESERVE | Configured Mission/Task | epoch, slot, type, instance, reservation ID를 변경 없이 전달 | Supply가 예약·점유의 Source. Task는 자기 실행만 판단 |
| Tool Surface Gripper | Task/Mission terminal | 같은 실행에서 관측한 HELD, 목적지 RELEASED와 작업 결과를 구분 | GRASP 명령 성공 ≠ 파지 성공 |
| Task terminal + 실제 트레이 점유 | PLC P4 Commit | 서로 독립인 `material_placed`와 점유 관측을 PLC가 결합 | Adapter/DB에서 성공으로 재분류하지 않음 |
| 검사 설비의 독립 판정 | History Inspection | 개체 ID·검사 ID·설비 ID·판정·근거 참조를 보존 | History 기록은 검사 결과를 생성하거나 PLC Commit하지 않음 |

## 오류 및 인과 관계

| 사건 | 전파 의미 | 보존할 원인 | 결과 결정 Owner |
|---|---|---|---|
| PLC 응답 단절·불확실 ROS terminal | UNKNOWN/UNCERTAIN 유지, 동일 명령 재실행 금지 | command ID, 원본 영수증·상태 | PLC Adapter·Mission |
| Robot 파지/이송 실패 | Task 실패와 `material_placed` 독립 보존 | 실행 ID, 실패 단계, 장치 상태 | Robot Task |
| Release 이후 Retreat 실패 | 원래 실패 코드 유지, 배치 사실 별개 전달 | Release 및 PLC 트레이 점유 증거 | Task / PLC |
| History DB 실패·검사 중복 충돌 | 기록 요청 거부 및 진단, 원본 센서·로봇 결과는 변경하지 않음 | 원본 Source와 기록 실패 | History의 로컬 데이터 책임 |

## 식별과 관측

`stream_id/plc_material_number`는 실제 개체, `slot_id`는 위치, `supply_epoch + reservation_id`는 예약, `command_id`는 PLC 실행 요청, `Goal UUID`는 ROS 실행, `station_id + inspection_id`는 해당 개체의 검사 근거를 구별한다. History는 이들의 기존 관계를 조회 가능하게 만들되 새로운 제어 ID를 발급하지 않는다.

새 자재의 첫 로봇 요청 전에는 PLC 결과 레지스터가 직전 요청 값을 유지할 수 있다. History는 새 자재의 결과·사유·배치를 미확정으로 두며 기존 자재의 결과와 Commit을 보존한다. History 쓰기 장애 중에도 Equipment는 PLC에 관측한 센서와 생성 번호를 전달하고, Supply는 영속화한 배정만 PLC에 확인한다.

자재별 실행 이력은 스트림에 연결된 Adapter 영수증에서 실제 개체 ID로 조회한다. Mission이 관측한 예약·Task UUID·terminal을 같은 Mission UUID의 영수증에 연결한다. 예약 전 거절 등 개체와의 연결 근거가 없는 요청은 자재에 귀속하지 않는다.

일반 INFO/WARN/ERROR는 원인이 의미 있게 변하는 경계에서 한 번 기록하고, 상세한 ID·원본 필드·시간/품질 근거는 구조화 이력으로 둔다. 고빈도 장치 샘플을 그대로 로그/DB에 복제하지 않고 PLC 단계 변화 등 의미 있는 사건만 기록한다.

실제 ROS 필드·Modbus 주소의 실행 선언은 Interface 코드와 PLC/Adapter Configuration을 각각 정식 원본으로 사용한다. 본 문서는 변환된 결과의 의미만 정의한다.
