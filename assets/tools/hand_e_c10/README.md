# Hand-E C10 100 mm 연장 핑거

사용자가 제공한 현장 사진의 긴 금속 핑거 형태를 참고한 시뮬레이션 초안입니다.
실측 치수가 없어 사용자가 선택한 **기존 손끝 기준 100 mm 연장**을 적용했습니다.
사진 속 장치의 기구를 복제하는 대신 C10의 방사형 세 손가락 구조를 유지합니다.

- 기본 `hand_e_c10`과 별도 `hand_e_c10_long` Tool·Model Assembly를 제공합니다.
  공식 Xacro·STL·관절 한계·Mimic·기본 관성은 수정하지 않았습니다.
- 각 연장부는 방사 방향 두께 3 mm × 접선 방향 폭 12 mm × 축 방향 길이 100 mm인
  손톱 형태의 납작한 사각 판입니다.
  단면·색상은 시각화를 위한 가정이며 사진에서 측정한 값이 아닙니다.
- 부착 위치는 공식 `finger_tip.stl` 최상단 면의 중심입니다. 손가락 로컬 좌표로
  `(0.06271698, 0, 0.15476571)` m에 Fixed Joint로 연결합니다.
  체결 구멍·볼트·브래킷이나 실제 가공·장착 적합성은 모델링하지 않습니다.
- 알루미늄 밀도 2700 kg/m³를 가정해 봉 하나의 질량을 9.72 g으로 두고,
  직육면체의 중심 관성을 계산했습니다. 실제 재질·물성이나 허용 하중 자료가 아닙니다.
- Visual과 Collision은 같은 크기의 Box입니다. 연장부는 각 손가락과 함께 움직이며,
  관절은 기존 0–19.2 mm 방사형 이동과 1:1 Mimic 관계를 유지합니다.
- `sf_grasp_tcp`는 하우징 중심축 위의 새 끝면 중심으로 이동합니다.
  하우징 뒷면부터 **254.76571 mm**이며, 기존 제조사 TCP와 연장부 장착면 사이의
  약 0.00429 mm 차이도 반영합니다. 도구 삽입 방향은 +Z로 유지합니다.

Launcher에서 **M0609/HCR-14 + Hand-E C10 + 100 mm 핑거 · 모델·관절 실험** 항목을
선택합니다. 기존 짧은 C10 모델도 별도로 실행할 수 있습니다.
ROS 2 Humble 환경을 적용한 뒤 저장소 루트에서 직접 실행할 수도 있습니다.

```bash
bash infra/isaac_sim/scripts/run_model_experiment.sh \
  ros2_ws/src/bringup/config/model_experiments/hcr14_hand_e_c10_long.yaml
# 한 번의 왕복 동작으로 종료하는 라이브 검증:
bash infra/isaac_sim/scripts/run_model_experiment.sh \
  ros2_ws/src/bringup/config/model_experiments/hcr14_hand_e_c10_long.yaml \
  --headless --cycles 1
```

M0609에서는 파일 이름을 `m0609_hand_e_c10_long.yaml`로 바꿉니다.
검증은 연장부의 방사형 왕복 이동·동기화, 관절 한계, 새 TCP와 연장 Collision의 존재를
확인합니다. Self-collision은 기존 정책대로 비활성입니다.
긴 봉의 휨·강성·체결부 응력, 보정된 파지력, 자재와의 실제 접촉·간섭·유지 성능은
검증하지 않습니다. 원형 자재, 파지 알고리즘, Vision, Physical Grasp 또는 신규 Cell은
포함하지 않으며 ROS Mission·파지는 지원하지 않습니다.

공식 C10 자산의 출처·라이선스는 [UPSTREAM](third_party/robotiq/UPSTREAM.md),
기본 모델의 동작·실행 제한은 [기본 C10 모델 안내](../../robots/m0609_hand_e_c10/README.md)를
참조하세요. 연장부와 위 치수·물성은 이 프로젝트의 시뮬레이션 설정이며 제조사 사양이 아닙니다.
