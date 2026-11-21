# HCR-14 실험적 시뮬레이션 모델

HCR-14 3G의 STEP 축선과 외형 치수를 사용한 시뮬레이션 모델이다. 제조사 URDF,
제어기 영점, 실물 보정 모델이 아니다. 기본 visual은 자체 작성한 회전체·타원 단면 프로파일로
만든 둥근 하우징·테이퍼 암·씰·Flange의 시각 근사이고, collision은 기존 원통 근사다.
정확한 CAD 외형이나 보수적인 collision envelope로 간주하지 않는다.

## 근거와 좌표계

- [HYRobotics HCR-14 CAD](https://www.hycobot.com/hcr-14):
  `HCR14_2D & 3D Drawings.zip`, SHA256
  `24e91bd8b39c1e34f26e8665b1bc64251073f9e4cd4d81eb6f82f750543719f0`.
  `HCR-14-ASSY_BODY_SOLID.STEP`의 PRODUCT 41개, 조립 관계/변환 46개,
  펼친 인스턴스 65개와 원/원통/평면을 분석했다. 조립 배치는 가동 관절 제약이 아니다.
- [한화 공식 3G 사용자 매뉴얼 V3.0, 2025-03](https://hanwharobotics.com/en/newsroom/download_view?bbsSeq=115&menuSeq=85):
  §1.4, §1.7, Appendix C/D/F. 관절 범위/최대 속도와 체결 형상을 대조했다.
  다른 HCR 모델의 치수나 물성은 복사하지 않았다. 매뉴얼의 방향 그림은 축 순서의
  보조 근거이며 수치 영점 보정의 근거로 사용하지 않았다.

CAD의 mm 좌표 `(x,y,z)`를 ROS m 좌표 `(x,-z,y)/1000`으로 변환한다.
Base 원점은 장착면 중심, +Z는 위쪽이다. X/Y의 제어기 방향 일치는 미확인이다.
CAD 조립 자세를 모든 관절의 시뮬레이션 `q=0`으로 정의한다. 각 관절 +회전은 아래
축에 대한 오른손 법칙이다. 제조사 영점/부호와의 대응은 미확인이다.

| Joint | CAD 축선 위 대표점 (mm) | CAD 축 방향 | 소속 가동 형상의 근사 |
|---|---|---|---|
| joint1 | (0,0,0) | +Y | 첫 1200W housing |
| joint2 | (0,207,0) | +Z | 둘째 1200W housing + 장완 |
| joint3 | (0,937,0) | +Z | 760W housing + 단완 |
| joint4 | (-538.8,937,0) | +Z | 첫 200W housing |
| joint5 | (0,937,184.7) | +X | 둘째 200W housing |
| joint6 | (-690,937,0) | +Z | 셋째 200W housing + Tool I/O |

대표점은 축선을 나타낸다. Xacro 원점은 축선 위 링 위치를 사용하며 PRODUCT 원점을
복사하지 않는다. 207 mm Base 높이, 730 mm 어깨–팔꿈치, 690 mm 말단 축까지의
도면 대응 투영 치수를 보존한다. 물리 Actuator 전체를 단일 가동 mesh로 배정하면
고정측/회전측이 잘못 움직일 수 있으므로 위 표는 상세 CAD 강체 분할의 증명이 아니다.

`flange`는 CAD `TOOL-IO_MOUNT`의 체결면 중심 `(-690,937,325.9)` mm에 있다.
체결면 +Z를 CAD +Z(ROS -Y)로 정한다. 면내 +X는 CAD +X로 선택한 시뮬레이션
규약이다. ISO 9409-1-50-4-M6, PCD 50, Ø31.5와 위치결정 홀에 대응한다.
`TOOL-IO_FLANGE`라는 부품명 자체는 실제 Tool 장착 프레임이 아니다.
`tool_mount`는 flange와 일치하는 이상적 강체 장착이며 제조 어댑터가 아니다.
기존 Robotiq Assembly와 `sf_grasp_tcp` 변환을 그대로 사용한다.

관절 범위는 J3 ±165°, 나머지 ±360°이고 최대 속도는
155/155/230/270/270/270 °/s이다. 실험 계획 속도/가속도는 각 0.5 rad/s,
0.5 rad/s²로 제한한다. 질량, box 관성, effort=1000 및 drive tuning은 전부
시뮬레이션 추정/설정값이다. 제조사 물성이나 정격 토크가 아니다.

## 시각 모델

[한화 공식 HCR-14 제품 사진](https://www.hanwharobotics.com/en/product/view?menuSeq=2&prdtSeq=13)의
짙은 회색 장완/대형 관절, 주황색 단완/손목, 금속 Flange와 녹색 표시 링을 참고했다.
공식 상세 사진의 둥근 어깨·팔꿈치 커버, 검은 접합 씰과 손목의 흰색 표시선을
반영했다. 장완 끝 J3 고정측과 단완 끝 J4 고정측 하우징을
각각 link2/link3에 보강하여 기존 링크 사이의 빈 연결부를 채웠다. 커버와
표시선의 치수는 사진에 근거한 시각 근사이며 도면 측정값이 아니다.
참고한 [공식 상세 사진](https://www.hanwharobotics.com/uploads/product/202601/202601121015390001.png)은
2026-11-16 확인했다. 사진/텍스처/제조사 CAD는 포함하지 않는다. 시각 파일 `visual/link0.dae`…`link6.dae`는
공통 도구 `tools/visual_meshes.py`가 `visuals.json`의 프로파일·재질 정의를 읽어
회전 형상과 타원 단면 연결·스윕 형상으로 생성한다. 제공된 현장 사진을 보조 참고해
선명한 주황색, 납작한 회색 장완 커버, 곧은 중심선의 주황색 단완 커버와
관절의 주황색 테두리를 반영했다. 사진 원근과 조명에서 추정한 외형이며
제조사 CAD 복원이나 도면 정밀도를 뜻하지 않는다. HCR-14 전용 생성 코드는 두지 않는다.
베이스 상단과 link1의 지름을 맞추고, 어깨 관절 둘레에 얇은 주황색 테두리와
방사 방향 체결부를 추가했다. link1 하우징도 연속 곡면으로 만들며 J1/J2 경계를
각각 분리한다. 이 장식 치수와 형상은 사진 기반 근사다.
장완과 팔꿈치, 단완 끝, 손목의 같은 강체에 속하는 꺾임은 원통을 겹치지 않고
연속 스윕 표면으로 만든다. 단완의 긴 중심선은 곧게 유지한다. 손목의 인접
링크 커버는 관절축에 수직인 경계면 양쪽으로 나누고 좁은 검은 씰로 경계를
표현한다. 이 시각 분할은 CAD의 실제 부품 분할을 검증한 결과는 아니다.
원본 CAD를 읽거나 변환하지 않으며 기구학·물성·collision의 별도 모델을 만들지 않는다.
생성기는 시각 mesh 전체의 SHA256을 `visual/revision.xacro`에도 기록한다. 이 정보는
확장 URDF provenance에 포함되어, 프로파일/재질 수정 후 재생성하면 기존 USD도
갱신된다. 시각 mesh를 직접 편집하지 말고 `visuals.json`에서 수정한다.
모든 시각 mesh는 기존 link-local 프레임의 m 단위다.

```bash
python3 tools/visual_meshes.py assets/robots/hcr14/visuals.json
```

collision과 관성은 외관 변경 전과 동일하다. 볼트·씰·측면 커넥터 등의 장식 치수는
시각 근사이며 제조 치수나 충돌 정밀도를 뜻하지 않는다. 표시 링은 고정된 외관이며
실제 장비 상태나 Safety 상태를 나타내지 않는다.

## 실행과 로컬 상세 형상

Launcher의 `HCR-14 (experimental · simplified geometry)`를 선택하거나:

```bash
source /opt/ros/humble/setup.bash
ARM_CELL_PROFILE="$PWD/ros2_ws/src/bringup/config/hcr14_profile.yaml" \
  infra/isaac_sim/scripts/isaac_arm_cell.sh
```

기존 Bringup 및 공통 Xacro→URDF→Isaac USD 경로를 사용한다.
이 Assembly는 `isaac.build_missing_on_startup: true`를 선택한다. Launcher/CLI의
Kit startup은 생성 URDF/USD/provenance가 없으면 공통 Pre Build producer를 먼저
실행한 뒤 scene을 구성한다. 기존 산출물이 있어도 공통 provenance가 선택된
Xacro/URDF/USD의 불일치를 확인하면 같은 producer로 갱신한다.
첫 실행과 모델 변경 후에는 자산 생성 시간이 추가된다. 기존 로봇의
Pre Build 실행 정책은 바꾸지 않는다. 별도 Importer,
Motion 또는 PnP 분기는 없다. HOME은 사용자가 지정한
`[180,0,0,0,90,0]` ° (`[pi,0,0,0,pi/2,0]` rad)이다.
Return은 기존 `[0,0,-pi/2,0,pi/2,0]` rad를 유지한다. 제조사 HOME이나
충돌 없는 복귀 경로를 뜻하지 않는다. 로봇마다 기존 HOME/Return 설정은 보존한다.

상세 형상이 필요한 경우 이용권이 확인된 `link0.stl`…`link6.stl`을 로컬 디렉터리에
준비하고 Model Assembly의 `mesh_root`를 그 절대 경로로 지정한 로컬 설정을 사용한다.
기본 `primitive` 디렉터리는 자체 시각 mesh + 기존 단순화 collision을 선택한다. 다른 디렉터리는 STL을 선택하며,
누락 mesh를 자동으로 대체하지 않는다. 로컬 Model YAML은
`urdf`/`semantic`을 원래 package URI로 유지하고 로컬 Deployment YAML의
`model_configuration`에 그 경로를 지정한다. Launcher 등록 파일을 수정할 필요는 없다.

STL 단위는 m이며 각 link의 Xacro `q=0` 프레임 기준이어야 한다. CAD에서 분리할 때
조립 변환을 적용하고 `(x,-z,y)/1000` 변환 후 해당 link Joint Origin을 빼야 한다.
회전부/고정부를 실제 인접 링크로 나눠야 한다. STL은 visual과 collision 모두에 사용되므로
상세 mesh 모드의 충돌 성능/정확성은 별도 확인해야 한다. 공개 소스는 로컬 STL의
정확성·존재·이용권을 보증하지 않는다.

CAD ZIP의 이용 허가서는 발견하지 못했다. 동봉 도면에는 복제/전달 제한이 있고,
공개 다운로드 제공만으로 로컬 변환·시뮬레이션이나 재배포 허용을 확정할 수 없다.
권리자 확인 전에는 상세 mesh 변환/배포를 수행하지 않는다. CAD와 파생 Mesh/USD를
Git에 넣지 않는다. 로컬 자산은 `.local_artifacts/` 등에 보관하고 로컬 설정도 공개하지
않는다. HCR-14 공통 경로의 생성 URDF/USD/provenance는 `.gitignore`로 제외한다.

## 검증 범위와 제한

- Bringup Model Validation, 축선/Flange 불변 조건, 공식 관절 제한, 공통 TCP/SRDF:
  자동 테스트로 확인한다. 제조사 FK 표본과의 비교는 아니다.
- Isaac 5.1 공통 Importer로 기본 형상 생성, Articulation Root와 HOME angular drive
  target 읽기를 확인했다. Headless PhysX에서 HOME 초기화 후 6축을 각각 +0.1 rad
  구동하고 HOME으로 복귀시켰다. 각 목표에서 180 step 후 최대 관절 오차는
  0.003 rad 미만이었다. 그래픽 전체 Cell PnP 인수 시험과 구분한다.
- MoveIt KDL에서 요청된 HOME `[180,0,0,0,90,0]` °의 FK/IK, 자체 충돌 및
  static Cell 비충돌을 확인했다.
  기존 Tray 중심 부근의 하향 TCP 자세에서 충돌 없는 IK를 찾았다.
- CNC loading의 기존 고정 TCP 자세는 IK가 성립하지만, 기본 근사 collision 형상과
  기존 static scene을 사용한 200 seed 탐색에서 충돌 없는 해를 찾지 못했다.
  도달 불가능의 수학적 증명도, PnP 성공도 아니다. 기존 CNC/셀이나 공통 planner를
  변경하거나 충돌을 임의로 허용하지 않았다.
- 상세 CAD collision, 실제 제조사 영점/부호/프레임, 실물 정확성 및 동역학은 미검증이다.
  기존 셀 전체의 픽/플레이스 경로와 Payload 부착 중 충돌도 미검증이다.

## Hand-E C10 정적 조립 모델

기존 HCR-14 팔과 `flange`→`tool_mount` 장착 프레임을 재사용하는
`hcr14_hand_e_c10` 조립 모델을 제공합니다. 기존 HCR-14 + 2F-85 조합은 유지합니다.
C10 하우징 뒷면을 `tool_mount`에 직접 장착하며, 실제 Coupling이나 제조 어댑터는
모델링하지 않습니다. HCR-14의 단순화 형상·물성·영점·실제 장착 적합성에 관한 제한은
위 설명과 동일합니다. 기존 HOME 설정도 그대로 사용합니다.

공식 C10 모델의 세 Prismatic Joint·Mimic과 하우징 뒷면 기준 +Z 154.77 mm TCP를
유지합니다. 손가락 수축은 외경 파지 방향, 확장은 내경 파지 방향입니다.
공식 자산의 출처·라이선스와 관절 구동 방향은 [C10 원본 자산 안내](../../tools/hand_e_c10/third_party/robotiq/UPSTREAM.md)를
참조하세요. TCP·관절 데이터는 두 로봇이 같은 C10 Tool Assembly를 사용합니다.

HCR-14 + C10 및 C10 Long은 정적 조립 모델로 유지합니다. Xacro의 장착 변환,
TCP와 관절·Mimic 구조를 검사할 수 있습니다. 별도 모델 실험 Launcher 버튼이나 실행 경로는 제공하지 않습니다.

HCR-14 + C10 조합에는 operational Cell·ROS Mission·Physical Real 실행 구성을
제공하지 않습니다. M0609 Tape Cell의 실행 지원을 이 조합에 적용하지 않습니다.
기존 셀에서의 충돌 없는 경로나 물리 파지 성공도 확인하지 않았습니다.

사진을 참고한 100 mm 연장 핑거의 정적 조립 모델은 `hcr14_hand_e_c10_long`입니다.
[연장부 치수·물성 설정](../../tools/hand_e_c10/urdf/assembly.xacro)을 참조하세요.
