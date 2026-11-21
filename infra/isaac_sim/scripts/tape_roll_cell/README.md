# Tape Handling Cell

Launcher에서 M0609, Hand-E C10 또는 C10 + 100 mm 핑거, Tape Handling Cell을 선택하고 파지 모드가 `물리 파지`인지 확인한다. Isaac 시작 → ROS 2 시작 후 공통 Simulation UI Hub의 **Supply**에 가득 찬 카트리지 개수를 입력한다. 한 카트리지는 17 mm 롤 15개를 30 mm 피치로 보관한다.

Cell 설정은 `ros2_ws/src/bringup/config/cells/tape_roll_cell.yaml`이다. 설비·슬롯·컨베이어·티칭은 Cell, 중공 형상·질량·마찰은 Material, 핑거 구동·장치 보유 추정은 Tool, 관절 제한·HOME은 Robot이 소유한다. 공통 UI는 Cell의 Supply capability를 사용하며 CNC의 기존 Gateway 공급 경로도 유지한다.

벨트 표면 구동은 Isaac Sim 5.1의 `isaacsim.asset.gen.conveyor` 내장 `IsaacConveyor` 노드를 사용한다.
Cell 시작 시 확장을 활성화하고 `/World/TapeRollCell/ConveyorGraph`에 인피드·아웃피드 좌우의
노드 3개를 생성한다. Python은 물리 Step마다 속도 명령을 전달하고 해당 그래프만 즉시 평가한다.
설비 회전은 월드 방향으로 변환하며 벨트 형상의 크기가 이송 속도를 바꾸지 않는다.
0 명령은 표면 이송을 명시적으로 비활성화한다. Isaac 5.1에서 작은 속도 변경 뒤 이전 속도값이
남는 경우에도 저속→정지, Pause·Stop·종료 시 정지를 보장하며 속도 벡터는 내장 노드가 작성한다.
내장 노드의 텍스처 애니메이션은 사용하지 않는다.
인피드 카트리지의 유한 힘 선형 구동과 구동축 정착 판정, 아웃피드의 시간 기반 운전은 유지한다.

Supply는 동적 카트리지와 자유 동적 롤을 인피드 위에 생성한다. 유한 힘의 프리즘 구동이 체인 구동 카트리지를 이동시키며 롤은 V자 받침·포켓과의 접촉·마찰로 운반된다. 각 슬롯을 같은 고정 Pick 기준점으로 인덱싱하고 구동축 위치·정지 피드백을 확인한 뒤 티칭 사이클을 요청한다. 롤의 실제 위치·축·속도는 공정 입력이 아니다. 별도 Vision은 없다.

Configured-source Mission → Cell 예약 → `/task_execution/execute_configured_grasp` → 공통 GraspCycle BT → 운영 MotionCore·Physical Tool Runtime → Isaac ROS Backend를 사용한다. 수평 Approach·Insert·Grasp·Lift·Extract 후 운반 통로에서 수직으로 회전하여 고정 아웃피드 수령점에 Place·Release·Retreat한다. 티칭은 인피드·아웃피드 또는 base 기준의 희소 6D TCP 포인트다. 각 포인트에 관절/직선 이동을 지정하며 모든 슬롯은 공통 순서를 사용한다. 실제 조립 URDF를 사용하는 운영 Motion이 관절·속도·가속도와 도달 가능성을 검사한다. 기본 MoveJ·MoveL은 환경 충돌 검사와 자동 우회를 하지 않는다. 물리 충돌 자체는 유지한다. 시작 시 슬롯별 IK·조밀한 관절 경로를 생성하지 않는다.

성공한 티칭 시퀀스와 같은 예약의 장치 Release를 확인하면 해당 슬롯의 명목 순서를 퇴역한다.
실제 이송 완료 수량은 집계하지 않는다. 아웃피드 벨트는 설정 피치/속도에 따른 시간 동안 운전한 뒤
다음 슬롯을 인덱싱한다. 마지막 슬롯 후 빈 카트리지를 회수하고 다음 카트리지를 공급한다.
실패 시 자동 재시도하지 않으며 실제 물체 상태를 보존한다.

모든 롤의 중력·충돌·마찰을 유지하며 Playing 중 강제 Attach·pose/velocity 재설정은 없다.
파지는 기존 주축 위치의 목표 미도달·지속 정지로 추정하고 해제는 수축 위치로 확인한다.
자재의 Stage identity·접촉 상대·정확한 물체/TCP 변환이나 동적 GT Planning Scene은 운전에 사용하지 않는다.
Stop은 생성 자재와 대기 요청을 제거하고 초기 빈 컨베이어·새 epoch로 복원하며 Pause는 초기화하지 않는다.

작업 완료 검증 기준은 충돌 없는 티칭 1사이클이다. 런타임 시퀀스 종료와 독립적으로 세션의
`diagnostic/simulation/tape_cycles/`에 충돌 GT와 PASS/FAIL/UNAVAILABLE를 남긴다.
관측 누락·저장 실패를 PASS로 간주하지 않으며 검증 결과는 다음 동작의 입력이 아니다.

설비 이동은 인피드와 아웃피드 각각의 위치·자세를 변경한다. 연결된 티칭 포인트·물리 지지면·컨베이어 관측·계획 형상은 같은 변환을 사용하며 바닥과 Robot 지지대는 고정된다. 정지 상태에서 빈 장면에 변경을 적용한다. 티칭 포인트와 순서는 편집·캡처·추가·삭제할 수 있고 저장 경로는 기본 `config/teaching/<model_id>.json`이다. 저장 전 기본 티칭은 자재·설비·Tool의 기하 의미에서 생성한다. 저장한 프로그램은 다음 시작에 복원하며, 실제 경로의 물리 실행 검증은 별도로 필요하다. HCR-14는 정적 모델 검증 범위이며 운영 배포로 등록하지 않는다. 실행 기록은 `.run_records/`와 ROS/Kit 로그에 남긴다.


### Teaching 화면 사용

새 ROS 인터페이스와 Motion 코드를 포함하므로 `bash ros2_ws/build-local.bash`로 빌드한 뒤
Isaac과 ROS를 다시 시작한다. 공통 Hub의 **Teaching → Manual Teaching**을 선택하면 자동
Mission 제출이 멈춘다. 포인트를 선택해 기준 좌표와 6D 자세(mm/degree), 이동 방식,
속도·가속도를 편집하고 **Update point**로 초안에 반영한다.

**Capture current TCP**는 현재 관절 피드백의 FK 자세를 선택 프레임에 기록한다.
**Preview**는 목표 축과 명목 포인트 연결선을 표시한다. **Test move**와 **TCP step jog**는
기존 Motion 실행 경로를 사용하고 **Stop**은 수동 작업 취소를 요청한다. 수동 이동 속도·가속도
비율은 최대 10%이며 조그는 한 번의 클릭으로 지정 거리를 이동하는 방식이다.

**Save**는 원본 프로그램을 영속화하고 **Apply**는 확인된 프로그램을 현재 자동 실행에
반영한다. **Load**는 저장 원본을 편집기로 읽고 **Revert**는 현재 적용본으로 되돌린다.
설비 재배치는 공급 자재를 비우고 Timeline을 Stop한 뒤 Equipment frames에서 수정하거나
Scene의 `/World/TapeRollCell/Infeed`, `/Outfeed`를 옮기고 **Read Scene Layout**으로 읽는다.
저장한 뒤 새 세션의 정상 시작 순서를 따른다. 포인트는 설비 프레임을 따라가므로 같은
작업 형상에 대한 배치 변경만으로 재티칭할 필요는 없다. 도달 가능성과 실제 물리 동작은
라이브에서 확인한다.
