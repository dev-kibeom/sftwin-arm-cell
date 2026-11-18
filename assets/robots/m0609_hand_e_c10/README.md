# M0609 + Hand-E C10

기존 M0609와 표준 Tool Adapter를 재사용하는 조립 모델입니다.
기존 M0609·RB5·HCR-14 및 Robotiq 2F-85 조립 모델은 변경하지 않습니다.
C10 하우징 뒷면을 `tool_mount`에 직접 장착하며, 제조사 Coupling이나 긴 전용 Finger는
모델링하지 않습니다. 시뮬레이션용 장착 모델이며, 제작이나 실제 장착 적합성을 승인하는
모델은 아닙니다.

세 Prismatic Joint는 공식 모델의 관절 범위 **0–0.0192 m**, 속도 **0.0535 m/s**,
손가락별 구동력 한계 **175 N**, 직접적인 1:1 Mimic 관계를 유지합니다.
관절 축은 각 손가락의 로컬 `-X`이며, 하우징 축 주위로 각각 0°·120°·240° 회전해 있습니다.
관절 위치가 증가하면 손가락이 수축하여 **외경(OD) 파지** 방향으로 움직이고,
감소하면 확장하여 **내경(ID) 파지** 방향으로 움직입니다.
운영 Tape Cell에서는 내경 팽창 파지와 실제 수축 해제를 사용합니다.

`sf_grasp_tcp`는 공식 `gripper_tcp_link`와 위치·방향이 같은 별칭 프레임입니다.
제조사 CAD에서 큰 지름의 손가락 단차 상단에 해당하며, 하우징 중심축 위에서
**하우징 뒷면 기준 +Z 방향으로 154.77 mm** 떨어져 있습니다.
+Z는 기존과 동일하게 도구 삽입 방향입니다. 매뉴얼의 로봇 플랜지부터 TCP까지의 거리는
이 모델에 없는 Coupling을 포함하므로, 해당 거리를 하우징 기준 위치에 다시 더하면 안 됩니다.

시각화·분리 충돌 STL, 관성 및 TCP 데이터의 출처는
[고정 버전의 공식 소스](../../tools/hand_e_c10/third_party/robotiq/UPSTREAM.md)입니다.
Base의 충돌 형상은 하우징·전면부로, 각 손가락의 충돌 형상은 Slider·Tip으로 나뉩니다.
Isaac Import 후에도 각각 분리된 볼록 충돌 형상으로 유지합니다.
기존 Importer는 의미를 갖는 고정 프레임을 보존하고 PhysX Mimic을 비활성화합니다.
Tool Runtime은 URDF의 Mimic 배율·Offset을 사용해 세 Linear Drive에 명령을 전달합니다.
2F-85의 파지 폭 변환이나 양측 접촉 로직은 사용하지 않습니다.

## 실행과 지원 경계

Launcher에서 M0609 + C10/Long + Tape Handling Cell + Physical Real을 선택하고
일반 Isaac/ROS 시작 흐름으로 실행합니다. 공통 Tool Plugin, 운영 ROS Backend,
공통 Task GraspCycle BT와 configured-source Mission을 사용합니다.
[상세 실행 안내](../../../infra/isaac_sim/scripts/tape_roll_cell/README.md)를 참조하세요.

이전 모델·관절 왕복 및 Logical 고리 실험 실행기·버튼은 제거했습니다.
HCR-14는 정적 조립·기하 검증 범위로 유지하며 운영 배포로 등록하지 않습니다.
기본형과 [100 mm 연장형](../../tools/hand_e_c10/README.md)은 같은 내경 전략을 사용하되
선택된 실제 URDF Collision과 TCP로 각각 기하를 검증합니다. 핑거의 실제 강성·허용 하중이나
보정된 파지력은 이 시뮬레이션 실행만으로 보장하지 않습니다.
