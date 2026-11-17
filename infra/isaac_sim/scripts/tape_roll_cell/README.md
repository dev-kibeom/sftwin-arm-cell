# Tape Handling Cell

Launcher에서 M0609, Hand-E C10 또는 C10 + 100 mm 핑거, Tape Handling Cell, Physical Real을 선택한다. Isaac 시작 → ROS 2 시작 후 공통 Simulation UI Hub의 **Supply**에 가득 찬 카트리지 개수를 입력한다. 한 카트리지는 17 mm 롤 15개를 30 mm 피치로 보관한다.

Cell 설정은 `ros2_ws/src/bringup/config/cells/tape_roll_cell.yaml`이다. 설비·슬롯·컨베이어·티칭은 Cell, 중공 형상·질량·마찰은 Material, 핑거 구동·접촉 판정은 Tool, 관절 제한·HOME은 Robot이 소유한다. 공통 UI는 Cell의 Supply capability를 사용하며 CNC의 기존 Gateway 공급 경로도 유지한다.

Supply는 동적 카트리지와 자유 동적 롤을 인피드 위에 생성한다. 유한 힘의 프리즘 구동이 체인 구동 카트리지를 이동시키며 롤은 V자 받침·포켓과의 접촉·마찰로 운반된다. 각 슬롯을 같은 고정 Pick 기준점에 정렬하고 실제 위치·축·속도를 확인한 뒤 PnP를 요청한다. 별도 Vision은 없다.

Configured-source Mission → Cell 예약 → `/task_execution/execute_configured_grasp` → 공통 GraspCycle BT → 운영 MotionCore·Physical Tool Runtime → Isaac ROS Backend를 사용한다. 수평 Approach·Insert·Grasp·Lift·Extract 후 운반 통로에서 수직으로 회전하여 고정 아웃피드 수령점에 Place·Release·Retreat한다. 기구학은 실제 조립 URDF로 산출하고 구간의 연속 관절 해를 보존한다. HOME 진입은 MoveIt 전역 계획, 이후 경로는 운영 Motion의 충돌·관절·속도·가속도 검사를 거친다.

실제 Release·수령 위치·축·정착 속도와 예약 token·epoch가 모두 맞아야 완료를 한 번 반영한다. Mission terminal 성공과 수량을 확인한 뒤 아웃피드 벨트가 롤을 배출한다. 실제 배출 위치 확인 후 다음 슬롯을 인덱싱한다. 마지막 롤 후 빈 카트리지를 회수하고 다음 카트리지를 공급한다. 실패 시 자동 재시도하지 않고 수량과 실제 물체 상태를 보존한다.

모든 롤의 중력·충돌·마찰을 유지한다. Playing 중 롤 강제 Attach·pose/velocity 재설정은 없다. MoveIt의 논리적 AttachedCollisionObject는 충돌 계획만 위한 것이며 PhysX Joint가 아니다. Stop은 생성 롤·카트리지와 대기 요청을 제거하고 초기 빈 컨베이어·완료 0·새 epoch로 복원한다. Pause는 초기화하지 않는다.

Cartesian 티칭을 만들 수 없는 설정에서도 장면은 열리지만 부분 경로를 실행 목록에 등록하지 않는다. HCR-14는 정적 모델 검증 범위이며 운영 배포로 등록하지 않는다. 실행 기록은 `.run_records/`와 ROS/Kit 로그에 남긴다.
