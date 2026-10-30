# ARM Cell — Final Live Acceptance

- **문서 유형:** Product Final Live Acceptance
- **대상:** ARM Cell / Isaac Sim 기반 통합 데모
- **Owner:** ARM Cell Product

## 1. 목적과 인수 범위

이 문서는 **현재 제품 데모의 16개 독립 라이브 인수 시나리오에 대한 정식 Product 계약**이다. 기존 데모 정의의 인수 시나리오 행·PASS·비보장을 누락 없이 이관한 문서이며, 현재 제품 전체의 정의는 [ARM Cell 데모 정의](definition.md)를 따른다. 시장·기술 분석 또는 실행 보고서가 아니며 실제 검증 완료를 주장하지 않는다.

원문 기준의 세부 전제·자극·전이·관측·PASS·비보장 의미를 보존한다. 표의 각 행을 개별 Use Case로 기계적으로 분리하지 않는다. 해당 실행 설계 및 세션 절차의 Owner는 기존 CDS/FDS/ICD/Guide에 둔다.

## 2. 완료 판정 원칙

필수 인수 대상은 아래 16개 독립 시나리오 전체다. 선택적 시나리오는 별도로 지정하지 않는다. 제품 전체의 Demo Complete 의미는 [Definition](definition.md)이 소유하며, 실제 판정·실행 결과는 별도의 검증 근거와 Acceptance Report가 소유한다.

## 3. 인수 시나리오

Demo Complete는 적용되는 live-current Profile을 기준으로 하나 이상의 Isaac Sim 라이브 세션에서 아래 모든 독립 시나리오가 PASS해야 인정된다. 하나의 긴 실행 스크립트는 이 독립 시나리오 집합을 대체하지 못한다. 각 행은 전제조건, 자극, 기대 전이, 관측 Oracle, PASS 조건, 비보장을 구분한다.

| 시나리오 | 전제조건 | 자극 | 기대 흐름·전이 | 관측 가능한 근거 | PASS 조건 | 비보장 |
|---|---|---|---|---|---|---|
| 정상 수량 Cycle | Production RGB-D Vision, 설정된 RawPart Profile, 수락한 수량 N | Trigger(quantity=N, material_type=RawPart) 발행. Isaac 데모 Adapter는 자세 보정 없이 무작위 Physics Drop 수행 | Gateway 수락·전달 → VDA 간략 단계 및 Simulation Adapter가 N개 자재 생성·낙하 → 공급자 완료 사실 → Delivery와 연관된 MATERIAL_READY 한 번 → 기존 Safety/Readiness Admission → Vision이 실제 RGB-D의 가시성·형상·안정성·Target Pose 독립 평가 → 수량 충족 또는 기존 부족/오류 정책까지 PICK/HELD/PLACE/RELEASED → GO_HOME 한 번 → Cycle 결과 | Trigger ID/유형/수량/Seed, VDA 단계, 공급자 완료 결과 및 선택적 Simulation 진단, 독립 Vision 관측/결과, RGB-D·Overlay, 선택 Target, Cycle별 processed_count, 누적 PLACE Commit, 작업 순서, GO_HOME 결과 | 공급자 정상 완료로 MATERIAL_READY가 발생하나 Vision 검출·파지 가능성은 확립되지 않음. Orchestration은 Vision이 적격 판정한 Target만 처리하고, 실행 인스턴스에서 Delivery별 한 번만 소비하며 기존 수량 회계를 적용 | 실제 AMR 주행이나 물리 배치 입증 아님. 결정적인 파지 성공 보장 없음 |
| 누적 반복 공급 | Timeline Running. Trigger A로 수량 3을 공급했고 Mission은 PLACE/RELEASED 하나 후 종료됨 | Trigger B, 수량 2 발행 | B는 A의 잔여물을 삭제하지 않고 자재를 추가함. 다음 순차 Cycle 작업량은 알려진 미처리 자재 4개 포함 | Delivery별 수락/공급 기록, Cycle/Goal ID 및 수량, 각 PLACE/RELEASED, 누적 처리량, Vision 후보 | 알려진 미처리 공급 4단위를 한 번씩 처리하며 이전 성공 PLACE를 중복 계산하지 않음 | 회계상 총량만으로 실제 물체의 물리적 존재를 증명하지 않음 |
| 활성 Cycle 중 Delivery | 한 Delivery의 ExecuteCycle이 실행 중 | 새 Trigger B 제출 | Gateway가 B를 수락하고 Adapter가 즉시 누적 Physics Drop 시작. 기존 Mission과 작업량 Snapshot은 변경하지 않고 B는 다음 순차 ExecuteCycle에서 처리 | 활성 Goal ID/수량의 불변성, 새 B Delivery ID와 Supply/Readiness, 다음 Goal 순서 및 합산 작업량 | 활성 Cycle은 취소·대체되지 않고 B는 기존 Cycle이 Terminal이 된 뒤에만 처리 | ExecuteCycle 동시 실행은 요구하지 않음 |
| Terminal 실패 후 잔여 공급 | Cycle이 미처리 공급을 남긴 채 실패로 종료 | 새 고유 Publish Trigger 제출 | 기존 잔여 공급과 성공 PLACE 회계 유지. 새 공급 성공분을 추가하며 다음 Cycle은 미처리 총량을 사용 | 이전·새 Delivery ID, Terminal 결과, 누적 PLACE, 다음 Cycle 수량과 Vision 관측 | 기존 잔여 Workload를 한 번만 고려하고 신규 공급으로 이전 Cycle을 재실행하거나 이전 자재를 삭제하지 않음 | 자동 Retry 또는 전체 Cycle Replay 없음 |
| Communication Health 경고 | Motion 활성/비활성 상태에서 필수 Safety-HW/Motion 정보가 유효하며 허용된 Freshness 범위 안에 있음 | 유계 수신 지연 또는 Timestamp/Freshness 경고 주입 | Communication Health가 관측되지만 Motion Capability·현재 Envelope·활성 Trajectory는 그대로 유지. 지연 경고만으로 StopMotion 없음 | 입력 Age/진단, 변경되지 않은 Safety Capability/Envelope, StopMotion 부재 | 경고만으로 자동 속도/가속도 감소, Safety Cause/Latch, Stop을 발생시키지 않음 | 설정된 데모 입력과 관측 범위를 넘어 일반화하지 않음 |
| 재시도 가능한 Target Episode 실패 | 하나의 Target Episode 활성, Motion/Safety/Gripper 관측 사용 가능, RawPart Fallback 정책 금지 | Trajectory 제출 후 재시도 가능한 일시적 Vision 실패·관측 불가 Orientation 또는 Terminal Retry-safe PICK 실패를 주입한 뒤 정상 조건 복원 | 적격 실패는 각 슬롯을 소비하고 새 Acquisition 사용. Orientation 관측 불가 시 Motion Goal 없음. Motion Grasp Pose Candidate/Planning Attempt 소진은 Retry 슬롯 없이 다음 로컬 Material Candidate로 진행. 로컬 목록 소진 후 적격한 외부 Retry는 Retry Admission → GO_HOME → 새 DetectTarget → 새 PICK 수행 | ExecuteCycle ID, retry_count, Acquisition·Candidate ID, Motion Action ID/결과, 관측 전용 Retry의 Motion 부재, HOME 결과, 신선한 Gripper/Safety 근거 | Candidate 내부 전환은 GO_HOME 없이 새 Motion Action 사용. 외부 Retry는 새 Acquisition을 사용하고 Candidate를 Replay하지 않음. 총 시도 슬롯 최대 4개, PICK 성공은 여전히 신선한 HELD 필요 | Safety Preemption, HELD/UNKNOWN, 활성 Motion, 안전하지 않은 HOME, 지속적 Backend/설정/계약 실패, 취소·복구 시 Retry 없음 |
| Retry 소진과 명시적인 새 Delivery | Mission 결과가 재시도 불가이거나 Target Episode Budget 소진 | 현재 ExecuteCycle 종료 후 새 Publish Trigger로 공급 추가 및 다음 정상 Cycle 개시 | 현재 ExecuteCycle은 원래 실패와 진단을 보존하며 종료. 신규 Delivery ID와 Supply Event, 다음 Cycle은 알려진 잔여 수량 포함 | Terminal 결과/원인, 이전·새 Delivery ID, Mission ID, Trigger/Gateway 수락 및 다음 Workload | 자동 Handoff Trigger Replay, 동일 Delivery 재수락, 전체 Mission 재시작은 없음. 잔여 Workload 보존 | 개발용 Retry Detection은 활성화해도 Vision만 수행하며 Mission을 재개하지 않음 |
| Retry Detection | 활성 Mission 없음, 개발 제어 활성화, Vision 사용 가능 | Retry Detection 한 번 실행 | 하나의 Vision Acquisition에서 Caller Timeout 30초 적용. Mission·Motion·자재 공급·PLACE 상태 변화 없음 | Acquisition ID, Terminal Vision 결과와 Snapshot Identity, 변경되지 않은 Mission/Motion 상태 | Vision 전용 결과가 PnP 실행 및 처리 수량 회계와 격리됨 | 공급이나 Motion 성공 주장 없음 |
| ROS Bringup 새 세션 | Kit 유지 중이며 이전 실험이 완료됐거나 의도적으로 리셋됨 | 동일 Kit 안에서 세션 가이드에 따라 새 Bringup 실험을 준비하고 새 Delivery 발행 | Stage 자재 및 Simulation 소유 상태 초기화. 이전 Delivery 회계·Mission·Retry Budget·Pending Readiness·Action·Vision 결과는 상속하지 않음 | Kit Identity, 서로 다른 ROS Process/Request ID, 초기화된 진단, Play 이후의 새 관측 | Kit 재시작 없이 새로운 실험을 수락·실행하고 이전 세션의 상태를 추정·이전하지 않음 | Bringup 교체를 통한 진행 중 운영 상태 보존은 범위 밖 |
| Isaac Python/Extension Reload | Kit와 Stage가 유지되고 활성 상태 | 세션 가이드의 Reload 절차로 Runtime Code 업데이트 적용 | 갱신된 코드가 한 번만 실행. 명시적 재구성 전까지 Stage/자재 상태 유지 | Module Revision, Callback/Node/Runtime ID, 변경되지 않은 Kit Identity | Kit/ROS Bringup 재시작이나 중복 Runtime Resource 없이 새 코드 적용 | Kit 전용 API 한계는 구체적으로 문서화된 예외 필요 |
| Isaac Timeline Stop Reset | 라이브 실험 종료 또는 초기화 시점 | Timeline Stop으로 실험을 리셋하고 세션 가이드에 따라 새 ROS 세션 구성 | Isaac 소유 로봇·Attachment·자재·Fixture·Fault 초기화. 새 ROS Process에는 기존 Workload/Readiness 및 Mock Fault 없음. Timeline Stop은 Safety Reset 호출하지 않음 | Timeline Event, Owner별 Reset 결과, 생성 객체/Fixture 제거, 유지된 Calibration/Log/Snapshot 근거, 새 Endpoint/Readiness 관측 | 새 실험은 수동 절차를 따르고 기존 잔여 Workload가 실행 수량에 반영되지 않음. Timeline Stop으로 Safety StopMotion/E-Stop/Safety Reset/Hardware Release 없음 | Pause→Play는 동일 실험. 같은 Process Stop→Play 격리 불요. Bringup 교체는 Safety Latch/Cause 기억을 이관하지 않음 |
| 통제 정지 | Task 활성 | 정지가 필요한 대표 비긴급 이상 주입 | Safety가 Severity를 선택하여 직접 정지 요청. Motion은 비활성을 확인 | Safety Cause/Severity, StopMotion, Motion 비활성 및 Task 순서 | 정지 확인 전에 후속 Task 진행 없음. 원인 근거 보존 | 안전 등급 인증 주장 없음 |
| E-Stop과 운영자 복구 | Motion 활성 | E-Stop 활성화, 원인 해제, 명시적 확인·Reset | Task/Iteration 종료. Reset 필요 상태는 수락 전까지 유지. Readiness 재평가, 중단 Task 재개 없음 | E-Stop/Safety 상태, 운영자 Request/Result, Motion 정지, 이후 Task ID | 운영자 조치와 현재 Safety Readiness 이후에만 복구 시작 | 소프트웨어 데모는 안전 등급 E-Stop Hardware를 검증하지 않음 |
| Holding 불확실성 | PICK 경로 활성 | 신선한 UNKNOWN Holding 결과 반환 또는 관측 Freshness 만료 | PLACE와 자동 계속 진행 차단 | MotionStatus Holding 값·Timestamp와 PLACE Request 부재 | PLACE 또는 Batch 진행 없음 | 실제 힘 센싱 정확도는 입증하지 않음 |
| 무효·오래됨·사용 불가 Target | Production Vision 실행 중, 배치 목표 수량 미달 | 일시적 Invalid/Stale/Unavailable 입력 주입 후 복원, 별도 지속 오류 주입 | 적격 일시 오류는 유계 시도를 소비하며 새로운 Acquisition 가능. 지속 오류는 Terminal. 안전하지 않은 PICK이나 자재 소진으로의 잘못된 매핑 없음 | Camera/Overlay, 관측 Timestamp/Validity, retry_count, Acquisition ID, Terminal Mission 결과 | 잔여 수량이 있는 유효한 OBJECT_NOT_FOUND는 수량 부족을 의미. 일시 오류 Retry는 예산 내 새 관측을 사용하고 Budget 소진 시 원래 실패 원인 유지 | Fixed Vision은 Production Perception 증거가 아님 |
| 복구 후 배치 재개 | 이미 처리한 수량이 있는 배치가 중단됐으며 Holding은 RELEASED | 운영자가 원인 해제 후 Acknowledge/Reset 요청 | 기존 원인 관측 유지. 새 GO_HOME 이후 신선한 Perception을 통해 보존된 배치에서 새 Iteration 시작 | 실패 원인, Holding, 새로운 Task·Observation ID, 배치 처리 수량 | Task/Pose Replay 없음. 배치 문맥과 실패 근거 보존 | HELD/UNKNOWN에서는 자동 계속 진행 없음 |

세션 격리와 Reload에 대한 보장은 필수 인수 계약으로 유지한다. 실제 수동 절차는 [Isaac Sim 실험 가이드](../../guides/arm-cell-isaac-sim.md)가, Reset/Epoch의 동작은 [Simulation Session FDS](../../design/arm-cell/components/simulation/fds-sim-session.md)가 소유한다. 절차의 문서 이동은 어떤 시나리오·관측·PASS·비보장 조건도 완화하지 않는다.

필수 시나리오는 각각 독립적으로 판정한다. 반복 공급, Retry Detection과 개발 세션 검사는 동일한 Isaac Kit 세션에서 수행할 수 있으며 Stop→Play는 명시적인 시뮬레이션 초기화에만 사용한다. 하나 이상의 Live Session에서 얻은 증거가 전체 인수 근거가 된다. Demo Complete는 해당 live-current 기준선에서 모든 필수 시나리오를 PASS한 뒤에만 주장할 수 있다.

## 4. 판정 및 증거 유효성

라이브 증거는 적용된 기준선 ID, 시나리오 및 Provenance, 한계와 무효화 조건을 포함해 저장소 Verification Policy에 부합하는 지속 가능한 형태로 확보한다. 인수 근거는 주장별로 분리한다. 합성 또는 과거 증거는 비라이브 주장을 뒷받침할 수 있지만 현재 라이브 인수 근거로 묵시적으로 승격하지 않는다.

## 5. 인수 범위와 비보장

아래 각 시나리오의 비보장 항목은 해당 PASS의 주장 경계를 제한한다. 전체 데모 인수는 실제 공장·안전 등급 장비·AMR 주행의 적합성을 검증하지 않으며 상세 비목표는 [Definition](definition.md)이 소유한다.

## 참조

- [제품 정의](definition.md)
- 자재 처리 Use Case
- 운영자 관측 Use Case
- 이상 대응 Use Case
- 시뮬레이션 실험 Use Case
