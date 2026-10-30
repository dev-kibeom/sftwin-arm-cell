# [SF-Twin] ARM Cell 라이브 테스트 분석

- **범위:** 하나의 ARM Cell Isaac 라이브 테스트 세션에 대한 실행 후 분석
- **관심사:** 증거 해석과 Ground Truth 비교 가능성
- **소유자:** ARM Cell Observability(진단 증거). 분석 작성자는 이 계약을 따른다.
- **관련 FDS:** [Observability Records](../interfaces/icd-observability-records.md), [Simulation Session](simulation/fds-sim-session.md), [Motion Execution](motion/fds-motion-execution.md), [Task Execution PnP](task-execution/fds-pnp-execution.md), [Orchestration Mission Cycle](orchestration/fds-mission-cycle.md)

## 1. 목적과 경계

이 FDS는 종료된 Isaac 라이브 테스트 세션 하나에 대해 사람이 작성하는 분석의 의미와 증거 제약을 정의한다. 기존 기록을 **관측 → 결정 → 결과**로 해석하며, Ground Truth와의 비교는 식별자, 시간, 좌표계, 관측 시점이 서로 호환될 때만 수행한다.

저장소 공통 라이브 테스트 분석 절차와 보고서 양식을 ARM Cell에 맞게 구체화한다. 이 FDS는 ARM Cell 세션, 증거, 비교의 의미를 소유하며, 일반 작업 흐름과 독자 대상 형식은 해당 공통 문서가 소유한다.

분석은 설명 자료다. 런타임 결과, 인수 보고서, Safety 판정, Mission 기록 또는 SSOT가 아니다. Vision, Motion, Gripper, Material Supply, Orchestration, Safety, Simulation은 기존 결정·결과·진단 기록 책임을 유지한다. 보고서는 런타임 결과를 변경하거나 컴포넌트의 결정을 재평가한 것처럼 암시해서는 안 된다(MUST NOT). 공식 인수 결과는 `docs/reports/acceptance/`에 유지되며, 분석은 인수 검증이나 PR 검증을 대체하지 않는다.

[Observability Records ICD](../interfaces/icd-observability-records.md)는 기록 분류, 수집, 세션 상관관계, 시간 기준, 손실 의미를 소유한다. [Simulation Session FDS](simulation/fds-sim-session.md)는 Play, Pause/Resume, Stop, Reset, 세션 생성 및 진단 마무리를 소유한다. 이 FDS는 실행 후 분석의 해석 및 비교 제약을 소유하며, 분석 작성자나 보고서 생성기에 진단 수집 책임을 부여하지 않는다.

## 2. 세션 식별과 증거 포함 기준

하나의 Play부터 Stop까지 구간이 라이브 테스트 세션을 식별한다. Pause/Resume은 같은 세션에 속하며 Stop 이후 다시 Play하면 새 세션이 생성된다. 분석은 세션의 `private local diagnostic artifact (not included in the public package)`에서 시작해 세션 식별자, 상태, 시작·종료 시각, 참여자, 산출물 링크를 확인한다.

내부 UUID와 작업 ID는 기록 연결에 사용할 수 있으나 독자 대상 서술에는 근거 있는 사람 친화적 설명과 순서(예: '첫 번째 자재', '두 번째 파지 시도')를 사용한다. 디렉터리의 인접성, 타임스탬프만의 유사성, 비슷한 식별자로 순서나 동일성을 추론해서는 안 된다.

세션 서술자는 색인이며 모든 생산자가 정상적으로 Flush를 끝냈다는 증거가 아니다. 서술자가 명시적으로 연결했거나 생산자가 세션·작업 참조를 제공한 산출물만 세션 구성원이다. 부재, 읽기 실패, 드롭, 잘림, 미연결, 미수집 증거는 구분해 보고한다. 기존 기록, 스냅샷, Replay 입력, Trace, Pose 기록은 생산자 경로에 그대로 보존해야 하며(MUST), 원본 위치를 인용하거나 연결한다.

분석 작성자는 Payload 의미, 식별자, 경로, 샘플링 한계 및 수집 손실에 대해 각 생산자의 계약을 따른다. 소유자의 결과 기록이 없을 때 인접 로그, 객체 개수 또는 타 컴포넌트 상태에서 정식 결과를 재구성해서는 안 된다(MUST NOT).

## 3. 분석 의미

Mission, Delivery 또는 관련 시도에 대해 다음 증거 연결을 유지한다.

| 요소 | 해석 |
|---|---|
| 관측 | 실제 수집된 값, 생산자, 식별자·참조, 좌표계와 시간 기준 및 수집 범위 |
| 결정 | 소유 컴포넌트가 기록한 규칙·설정과 실제 분류. 현재 설정으로 과거 결정을 다시 계산하지 않는다. |
| 결과 | 소유자의 정식 결과, 재시도·복구, 기록된 최종 관측 상태. 후속 복구에 성공해도 앞선 실패 시도를 보존한다. |
| 귀속 | 관측 사실, 소유자가 보고한 사유, 독립적으로 뒷받침되는 원인, 추론, 가설을 구분한다. 미해결 설명을 구별하기 위해 필요한 증거를 명시한다. |

실패 단계와 인과 설명은 서로 다른 주장이다. 시간적 인접성, 상관관계, Behavior Tree의 `FAILURE`, Timeout만으로 근본 원인을 확정할 수 없다. `CONFIRMED`, `NARROWED`, `HYPOTHESIS`, `INSUFFICIENT EVIDENCE`는 인용된 증거가 뒷받침하는 경우에만 일반 언어의 원인 귀속 분류로 사용한다. 소유자의 결과가 없으면 성공이 아니라 `UNAVAILABLE`로 취급한다.

확인된 세션별로 증거를 묶되 Delivery, Mission, ExecuteCycle, Acquisition, Goal, 재시도 범위를 구분해 보존한다. 세션 합계는 소유자 기록에 근거한 개수를 요약할 수 있지만 서로 다른 종료 결과를 합쳐서는 안 된다(MUST NOT). 안정적인 식별자나 순서를 알 수 없다면 순번을 부여하지 말고 그 제약을 설명한다.

## 4. Ground Truth 비교

Ground Truth는 실행 후 검증 입력일 뿐 런타임 컴포넌트는 이를 사용하지 않는다. 정량 비교는 적용 가능한 다음 조건이 모두 확인될 때만 허용한다.

1. **세션 및 객체·작업 식별:** 두 값 모두 확인된 동일 세션과 동일 객체 또는 작업에 속해야 하며 생산자 ID나 명시적 링크로 입증한다. 휴리스틱 연결은 추론일 뿐 식별 사실이 아니다.
2. **관측 시간:** 동일한 원본 프레임·시뮬레이션 관측 또는 유계 구간을 가진 기록된 매핑·보간 규칙을 기준으로 해야 한다. Epoch와 시간 기준을 기록한다. 가까운 샘플만으로는 부족하며 서로 다른 프로세스의 로컬 Steady Clock은 비교할 수 없다.
3. **좌표계:** Pose가 동일한 알려진 좌표계를 사용하거나 해당 시각에 유효한 기록된 변환을 사용해야 한다. 변환이 없거나 오래됐거나 Epoch가 일치하지 않으면 지표를 구할 수 없다.
4. **관측 지점과 상태:** Vision의 입력 프레임 추정치와 대응 객체 관측, Motion의 종료 시점 측정 상태, Gripper의 Attach 전후 상태처럼 생명주기상 동등한 지점을 비교한다. 나중에 읽은 가변 Stage 상태는 결정에 실제 사용된 값을 대체할 수 없다.
5. **형상 해석:** 치수에는 단위와 축 순서를 명시하며 자세 대칭성은 소유 Vision·형상 계약을 따른다. Stage 작성 변환, PhysX Pose, 렌더링·이미지 공간 추정치를 서로 대체할 수 없다.

조건이 충족되지 않으면 지표를 `UNAVAILABLE`로 표시하고 실패한 조건과 누락된 증거를 명시한다. 가능한 연결이 여러 개 남으면 `UNRESOLVED`로 표시하며 정밀한 오차값을 게시하지 않는다. 비교 가능한 값에는 양쪽 원본 참조, 좌표계, 시간·Epoch, 변환 경로, 계산 방법, 단위, 연결 과정의 추론을 포함한다.

이 규칙은 위치·자세·치수·관절 및 TCP 종료 오차·Capture/Release 일관성·수량 대조·성능 비교에 적용한다. 소유 계약이 분류 기준을 정의하지 않았다면 새 통과·실패 임계값을 만들거나 작은 오차를 성공으로 취급하지 않는다. 사용 가능한 세션이 비교 조건을 충족하지 않으면 대리값을 계산하지 말고 제약을 설명한다.

## 5. 수량·지표·성능

요청·공급·처리·성공·실패·잔여 수량은 해당 책임 소유자의 기록에서만 사용한다. 공급 제공자 결과, Mission 결과, 관측된 객체 개수는 구분한다. 객체 샘플은 공급 제공자의 종료 결과를 대신할 수 없다. 소유자 결과가 없거나 세션에 연결되지 않았다면 해당 값은 사용 불가로 보고한다.

지표와 차트는 실제 비교 가능한 원본 값으로만 도출한다. 입력, 단위, 분모, 포함·제외 기준, 계산을 재현할 수 있어야 한다. Confusion Matrix에는 비교 가능한 Ground Truth, 명시적으로 정의된 분류 단위, TP/FP/FN/TN 의미가 필요하다. 레이블 없는 관측은 음성 또는 성공 사례가 아니다. 지표의 수집 범위와 제외 내역을 보고하고 근거 없는 숫자와 차트는 생략한다.

성능 요약에는 생산자, 구간의 시작·끝, Clock 기준, 샘플 수, 계산, 수집 손실을 명시한다. 타임스탬프 차이는 동일 Clock·프로세스 범위 또는 검증된 공통 시뮬레이션 시간 구간에서만 계산한다. 관측된 `/clock` 진행을 해석에 필요한 종점 증거 없이 물리 RTF나 네트워크 지연이라고 볼 수 없다. 유효한 세션 서술자의 UTC 시작·종료 시각은 별도 표시한 실제 경과시간 계산에는 쓸 수 있으나 시뮬레이션 시간 또는 프로세스 간 실행 지연을 입증하지는 않는다.

## 6. 출력과 추적성

분석 보고서는 임시 진단 세션 산출물과 분리된 `docs/reports/live-test/YYYYMMDD-HHMMSS[-suffix].md`에 보존한다. 날짜·시간과 충돌 방지 접미사는 선택한 세션을 식별한다. 차트 등 보고서 자산은 `docs/reports/live-test/assets/<session-id>/`를 사용한다. 기존 보고서는 보존한다. 세션 기반 보고서 파일명이 충돌하면 해당 세션 날짜·시간과 경로 접미사를 유지하면서 사용하지 않은 분석 접미사를 추가한다. 다른 내용의 보관 자산을 덮어쓰지 말고 별개 이미지나 차트에는 고유한 파일명을 사용한다. 보고서 생성 과정은 `.local_artifacts/`의 세션 서술자나 생산자 증거를 수정하지 않는다.

보고서는 실제로 확인한 도메인, 세션 식별자, 분석 시각, 원본 경로 및 코드 기준을 보존한다. 런타임 코드 버전은 실행 당시 증거에 기록된 경우에만 보고하고, 분석 시점에 확인한 리비전은 별도로 표시한다. 로컬 `.local_artifacts/` 증거 링크는 체크아웃 외부에서 유효하지 않을 수 있으므로 경로와 세션 식별자를 유지한다. 선택한 세션 연결 이미지를 보고서 자산에 보관할 수 있으나 진단 기록이나 전체 Replay Payload를 `docs/reports/`에 복제하지 않는다.

한국어 기사형 구조와 독자 대상 용어는 라이브 테스트 분석 보고서 양식에서 정의한다. 양식은 길이를 고정하거나 근거 없는 데이터를 특정 절에 채우도록 요구하지 않는다.

UUID, Goal ID, Acquisition ID, Execution ID는 내부 연결과 원본 참조에 사용하며 기본적으로 본문 서술에서는 생략한다. 전체 증거 목록을 매 표 행에 반복하지 않고 주장을 뒷받침하는 원본 증거를 그 근처에 인용한다. 원본 Payload는 변경하지 않는다. 보고서는 파생된 분석이지 기계가 소비하는 계약이 아니므로 제목과 본문은 실행 권한을 부여하지 않는다.

## 7. 참조

- [Observability Records ICD](../interfaces/icd-observability-records.md)
- [Simulation Session FDS](simulation/fds-sim-session.md)
- [Observability Record Inventory](observability-record-inventory.md)
- [Simulator State, Time, and TF ICD](../interfaces/icd-sim-state-time-tf.md)
- Live Test Analysis Report Template
- Verification Policy
