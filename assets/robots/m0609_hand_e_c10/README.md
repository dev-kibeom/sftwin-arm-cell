# M0609 + Hand-E C10 — 1단계

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
1단계에서는 이동만 검증하며, 어느 방향의 파지 동작도 검증하지 않습니다.

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
실험에서는 URDF의 Mimic 배율·Offset을 사용해 세 Linear Drive에 명령을 전달합니다.
2F-85의 파지 폭 변환이나 양측 접촉 로직은 사용하지 않습니다.

ROS 2 Humble 환경을 적용한 뒤 저장소 루트에서 실행합니다.

```bash
bash infra/isaac_sim/scripts/run_model_experiment.sh \
  ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10.yaml
# 한 번의 왕복 동작으로 종료하는 라이브 검증:
bash infra/isaac_sim/scripts/run_model_experiment.sh \
  ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10.yaml \
  --headless --cycles 1
```

Launcher에는 C10 모델 실험임을 명시한 별도 버튼이 있습니다.
독립 모델에서 수축·확장을 반복하며, ROS 시작은 비활성화됩니다.
운영 Runtime Profile Loader는 모델 실험 설정을 거부합니다.
Mission, GRASP/RELEASE, Vision, Payload, Physical Grasp, 신규 Cell 및 자재 통합은
지원하지 않습니다. Runtime 구동 설정은 실험용이며, 보정된 파지력이나 자재 유지 성능을
보장하지 않습니다. 내부 Self-collision은 기존 Importer 정책에 따라 비활성화합니다.
라이브 검사에서는 가져온 충돌 형상의 존재를 확인하며, 실제 접촉 정확성이나 자재 취급을
검증하지는 않습니다.

각 실행은 조립된 설정과 `result.json`을 `.run_records/model-experiment-*/`에 기록하고,
기존 모델 출처 추적 정보와 함께 파생 USD를 생성합니다.
측정 항목은 관절 한계, 목표 끝점 도달, 동기화, 방사형 변위, TCP 및 분리 충돌 형상의
존재입니다. PASS에는 완전한 수축·확장 왕복이 필요합니다.
창을 닫아 중단했거나 검사에 실패한 실행은 검증 완료로 보고하지 않습니다.

사진을 참고한 100 mm 연장 핑거는 기본 모델과 별도인 `m0609_hand_e_c10_long` 실험입니다.
[연장부 치수·물성 가정과 실행법](../../tools/hand_e_c10/README.md)을 참조하세요.
