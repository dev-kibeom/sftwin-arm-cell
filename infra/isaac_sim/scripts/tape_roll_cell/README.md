# 수평 Tape Roll 실험

M0609 + C10/Long의 단일 롤용 실험 플러그인이다. 운영 Cell Profile과 Vision을 추가하지 않는다.
공통 `scene_entrypoint`, Tool Runtime 선택, `internal_radial` 기하 해와 native Task Execution
`GraspCycle` BT를 재사용한다. CNC Scene Builder와 2F-85 Runtime에 의존하지 않는다.

`experiment.yaml`이 자재·장비 위치·실험 티칭·구동/관측 설정을 소유한다.
임시 롤은 내경 120 mm, 외경 150 mm, 폭 17 mm, 질량 80 g이다. 두 핑거 길이 모두
선택 URDF의 수축 통과/확장 접촉 범위 안에 있는 치수다. 실제 테이프 제품의 측정값은 아니다.
관성은 기본적으로 균질 중공 원통식이며 `diagonal_inertia`로 세 주관성모멘트를 지정할 수 있다.
롤은 하나의 동적 Rigid Body 아래 별도 convex-hull sector들로 구성한다. 중앙 구멍을
가로지르는 단일 convex hull은 사용하지 않는다. 내측 chord도 지정 내경을 침범하지 않는다.

## 실행

ROS overlay와 최신 native 시험 브리지를 준비한다. 저장소 README의 빌드 절차를 따른 후:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
infra/isaac_sim/scripts/tape_roll_cell/run.sh \
  ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10_long.yaml \
  "$PWD/ros2_ws/build/arm_cell_task_execution/libgrasp_cycle_fixture_bridge.so" \
  --headless --cycles 1
```

기본형은 profile을 `m0609_hand_e_c10.yaml`로 바꾼다. `ISAAC_SIM_ROOT` 기본값은
`~/isaac_sim`이다. `--headless`를 생략하면 Isaac 화면을 볼 수 있다.
현재 실험은 한 번의 Pick/Release만 허용하며 반복 공급/컨베이어 구동은 제공하지 않는다.

## 실행과 판정

기존 Task BT의 접근·삽입·GRASP·Holding 확인·Grasp Relation 확정·이송·RELEASE·
Release 확인·Relation 해제·후퇴 순서를 사용한다. 시험 MotionPort의 transfer는 슬롯 밖으로
Extract한 다음 가이드 상부의 Clearance를 거쳐 수령 지지대로 이동한다.
지지면 4 mm 위에서 Release하고 중력에 의한 안착을 관측한다. 이 실험 MotionPort는 기존 #104의 live fixture
backend이며 운영 MotionPort/Safety 통합을 검증하지 않는다. 최초 physics step 전에 로봇을
티칭 Approach로 초기화하므로 HOME에서 셀까지의 이동은 검증 범위 밖이다. 실행 중 IK나 경로 계획은 없다.
모든 Scene collision은 유지하고 Robot/Fixture 및 비핑거 Robot/Roll 접촉을 실패로 판정한다.

C10 Tool은 실제 prismatic drive target만 변경한다. Finger별 대상 롤 접촉 반력과
확장 중 측정 obstruction이 dwell 동안 유지되어야 HELD다. 접촉이 없거나 관측이 누락되면
UNKNOWN이다. RELEASED는 실제 수축 위치 도달과 접촉 소실이 모두 필요하다.
STOP은 측정 위치를 drive target으로 유지하며 해제를 명령하지 않는다.
Grasp Relation은 측정 TCP→Roll 변환이며 물리 구속을 생성하지 않는다.
Fixed Joint, kinematic body, 중력 해제, transfer 중 pose/velocity 재설정은 사용하지 않는다.

`.run_records/tape-roll-*/`의 configuration/result JSON에 revision, 입력·소스 hash,
위상별 TCP/Roll 변환, 접촉 반력, 측정 finger 위치, Holding, Relation 및 결과를 보존한다.
명령 수락이나 단위 테스트 통과만으로 물리 파지 성공을 판정하지 않는다. 실제 이송 거리,
수령 위치, 후퇴 후 선속도·각속도·수평 축 방향까지 확인해야 PASS다. 실패한 실행은 FAIL을 유지한다.
