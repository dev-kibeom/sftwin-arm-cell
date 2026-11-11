# 진단 필드 명명과 호환성

공유 의미는 [관측·진단 기록 ICD](../interfaces/icd-observability-records.md)가 소유하고, 기록 생성은 각 생산자가 소유한다. 이 문서는 기존 근거에 추가한 이름을 정의하며 새로운 관측이나 제어 입력을 도입하지 않는다. 기존 스키마 버전과 이전 키는 계속 읽을 수 있다. 새 소비자는 아래 정식 이름을 사용하고, 과거 기록에는 명시된 이전 필드를 대체 입력으로 사용한다. 정식 필드와 호환 별칭을 별개의 근거로 중복 집계하지 않는다.

| 정식 필드 | 이전 필드와 해석 |
|---|---|
| Motion `task_result_code`, `task_result_name` | 숫자 `task_executor_result_code_value`, 문자열 `task_executor_result_code`에 대응한다. 계획 진행 중에는 두 정식 결과가 모두 null이다. |
| Motion `record_phase` | `record_status`의 `planning_in_progress` 또는 `terminal`에 대응한다. 실행 단계를 나타내며 저장 성공 여부를 뜻하지 않는다. |
| PICK 요약 `insertion_started` | `pick_grasp_insertion_started`에 대응한다. 이전 `insertion_execution_count`는 이벤트 횟수가 아닌 0/1 표시다. |
| PICK 요약 `material_candidate_search.multiple_material_candidates_searched` | `candidate_switch_occurred`에 대응한다. `precise_search_began`인 시도가 둘 이상인지를 뜻한다. 이전 기록에서 이 플래그가 없으면 true로 간주한다. 실제 물리 동작의 전환을 입증하지 않는다. |
| PICK 요약 `selected_grasp_pose_index` | `selected_candidate`에 대응하며 0부터 시작한다. -1은 선택 없음을 뜻한다. 범위는 GRASP_POSE다. |
| PICK 요약 `last_evaluated_grasp_pose_number` | `last_evaluated_candidate_number`에 대응하며 1부터 시작한다. `last_evaluated_material_candidate_id`와 함께 해석한다. null은 미가용을 뜻한다. |
| 작업 정확도 `selected_grasp_or_place_pose_index` + `selected_pose_scope` | PICK과 PLACE 모두 0부터 시작한다. 이전 `selected_planning_candidate_index`는 PICK에서 0부터, PLACE에서 1부터 시작하는 값을 유지한다. |
| 작업 정확도 `feedback_fk_tcp_pose` | `actual_tcp_pose`에 대응한다. 측정 관절 피드백과 모델의 순기구학으로 계산한 값이며 독립적인 물리 TCP 측정값이 아니다. 가용성은 기존 측정 상태를 따르고, 미가용 Pose는 빈 객체로 유지한다. |
| 궤적 `final_command_attempt_simulation_time_s` | `final_command_attempt_simulation_time`에 대응하며 단위는 시뮬레이션 초다. 명시적인 시간 정렬 없이 서로 다른 시뮬레이션 epoch의 값을 비교하지 않는다. |
| Orchestration `result_code`, `result_name` | `failure_code`, `failure_name`은 성공한 Motion Action 결과도 포함한다. 미가용 최종 결과는 null / UNAVAILABLE로 유지한다. `failure_domain`과 `event`를 함께 읽는다. 결과 도메인이 없거나 결과가 미가용이면 정식 필드는 null / UNAVAILABLE이며, 이전 필드는 기존 기본값을 유지한다. |
| Orchestration `motion_feedback_gap_elapsed_ns` | `feedback_gap_elapsed_ns`에 대응하며, 정식 필드는 피드백 공백·복구 이벤트에만 출력한다. 이전 필드는 release·통신 이벤트에도 나타나므로 해당 문맥에서 측정된 피드백 공백으로 해석하지 않는다. |
| Safety/Orchestration `capture_complete` | 요약의 `complete`에 대응한다. 기록된 유실이나 잘림 없이 Writer가 대기 기록 처리를 마쳤음을 뜻한다. Mission 성공이나 전체 센서 수집 범위를 보장하지 않는다. 요약이 없으면 완전성은 미확인이다. |
| Vision 입력 `depth_cache.available/reused/status` | `cached_pair_available/reused/status`에 대응한다. RGB와 독립적인 Depth 캐시이며 이전 `pairing` 객체 밖에 별도 범위를 둔다. 명시적인 RGB-D Calibration 쌍의 진단은 pair 이름을 유지한다. |

`ARM_CELL_MOTION_CLOCK_CAPTURE_ID`는 Motion Clock 수집 ID를 지정한다. 비어 있지 않은 정식 설정값을 우선하며, 없거나 비어 있으면 `ARM_CELL_OBSERVABILITY_CAPTURE_ID`를 대체 입력으로 사용한다. 이 ID는 Clock 수집에 속하며 컴포넌트 전체를 연결하는 전역 추적 ID가 아니다.

Vision 내부 평면 컬렉션 이름은 `cuboid_planes`다. 직렬화된 `cuboid_height_patches`는 평면 근거를 담는 호환 키로 유지하며, 크기가 큰 기하 배열은 복제하지 않는다. 자재 후보 ID, 파지 자세 인덱스, 시도 순서는 서로 다른 식별 정보다. 이전의 일반적인 `candidate_id`는 상위 객체의 범위 안에서 해석하며 이름만으로 전역 연결하지 않는다.

중복된 `final_result`/`mission_exit_reason`, Vision 최상위·중첩 결과 코드, PICK 선택 방향은 호환용으로 유지한다. 새 소비자는 Mission의 `mission_exit_reason`, Vision의 `terminal_result`, PICK의 `candidate_evaluation`을 정식 위치로 사용한다. 이번 전환에서는 이전 필드를 삭제하지 않는다.

새 실행은 예약된 실행 디렉터리 아래 `diagnostic/session.json`과 `diagnostic/<component>/`에 기록한다. `diagnostic_records` 경로는 명시적으로 연결된 과거 세션을 읽기 위해 계속 지원한다. 디렉터리가 가깝다는 사실만으로 같은 세션에 속한다고 판단하지 않는다.
