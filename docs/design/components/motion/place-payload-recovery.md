# Place 탐색과 부착 자재 home 복구

운영 PLACE는 전체 경과시간으로 탐색을 중단하지 않는다. 설정한 pose 수, pose별 IK seed 수, 후보별 global planning 요청 수를 소진하거나 실행 가능한 경로를 찾으면 종료한다. 개별 IK/MoveIt 호출 시간 한도는 유지한다. `place_total_planning_time_s`는 명시적 target qualification/waypoint 생성 경로의 호환 설정이며 production 탐색 deadline이 아니다.

운영 프로파일의 독립된 초기값은 pick과 같다: `place_max_pose_candidates=48`, `place_ik_seed_attempts_per_pose=2`, `place_ik_timeout_s=1.0`, `place_candidate_planning_time_s=15.0`, `place_planning_num_attempts=3`, `place_approach_global_max_requests=3`. Place pose 생성기는 현재 최대 13개를 생성하므로 48은 상한이다. Pick 설정을 바꿔도 place 설정은 자동 변경되지 않는다.

접근 또는 삽입 계획의 후보 소진 후 fresh HELD가 확인되면 `place_home_retry_count`만큼 home 경유 재시도한다. 기본 1이며 최초 탐색은 포함하지 않는다. 0이면 즉시 기존 반환 시퀀스로 간다. 각 재탐색에는 동일한 새 계산 예산을 부여한다. home 계획·실행 실패, 취소, 안전 중단, backend 오류는 재시도를 허용하지 않는다. 자재를 임의 위치에서 release하지 않는다.

`PAYLOAD_HOME` Motion segment는 현재 상태에서 home 관절 목표까지 MoveIt으로 계획하고 경로를 검증한 뒤 기존 trajectory executor로 실행한다. 기본 BT subtree `PayloadHome`의 `ReturnHomeWithPayload` action은 `/motion/return_home_with_payload` ROS action에서도 호출 가능하다. ROS 타입은 `arm_cell_interfaces/action/ExecuteTask`, task type은 `TASK_TYPE_PAYLOAD_HOME=6`이다. 다른 goal type은 이 endpoint에서 거절된다. 기존 `/motion/execute_task`와 실행 mutex·취소 처리를 공유한다.

`payload_home_bt_plugin`이 비어 있으면 built-in BT 노드를 사용한다. 절대 shared-library 경로를 지정하면 해당 라이브러리의 `BT_RegisterNodesFromPlugin`을 로드한다. 라이브러리는 `ReturnHomeWithPayload` 노드를 등록해야 한다. 제공되는 `libpayload_home_bt_plugin.so`는 기본 구현이다. 각 subtree blackboard의 `payload_home_operation`은 executor가 제공하는 `std::function<BT::NodeStatus()>`이다. 플러그인은 이를 호출해 동일한 계획·검증·취소 계약을 유지해야 한다.

캐시된 waypoint도 실제 자재를 잡고 있으면 최신 scene과 부착 자재로 경로를 재검증한다. 사전 waypoint 생성은 자재가 없는 offline 경로로 유지한다.

부착 충돌 모델은 accepted pick의 object-to-TCP 상대 변환과 관측한 직육면체 치수에서 생성한다. MoveIt monitor 전역을 변경하지 않고 각 place/home 요청의 scene diff와 시작 상태에 동일한 attached body를 넣는다. Endpoint 검사와 경로 계획·검증에 반영하며, finger/tip과 TCP만 접촉을 허용한다. Scene 또는 accepted geometry가 없으면 계획을 거절한다. RELEASED 확인 후 grasp relation과 치수를 해제한다.

재시도를 소진하면 기존 home→tray 직접 관절 이동→open→fresh RELEASED 확인 순서를 재사용한다. 사용자 결정에 따라 이 **자재 반환 이동은 충돌 검사 대상에서 제외**한다. 반환이 완료되면 RETURNED와 원래 place 실패 코드를 유지한다. 반환 실패는 그 실패를 보고하며 RETURNED로 간주하지 않는다.
