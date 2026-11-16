# HCR-14 실험적 시뮬레이션 모델

HCR-14 3G의 STEP 축선과 외형 치수를 사용한 시뮬레이션 모델이다. 제조사 URDF,
제어기 영점, 실물 보정 모델이 아니다. 기본 visual/collision은 직접 작성한 원통
근사 형상이다. 정확한 CAD 외형이나 보수적인 collision envelope로 간주하지 않는다.

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

## 실행과 로컬 상세 형상

Launcher의 `HCR-14 (experimental · simplified geometry)`를 선택하거나:

```bash
source /opt/ros/humble/setup.bash
ARM_CELL_PROFILE="$PWD/ros2_ws/src/bringup/config/hcr14_profile.yaml" \
  infra/isaac_sim/scripts/isaac_arm_cell.sh
```

기존 Bringup 및 공통 Xacro→URDF→Isaac USD 경로를 사용한다. 별도 Importer,
Motion 또는 PnP 분기는 없다. HOME/Return은 `[0,0,-pi/2,0,pi/2,0]` rad이며
이 시뮬레이션 규약에서 선정한 셀 내 비충돌 시작 자세이다. 제조사 HOME이나
충돌 없는 복귀 경로를 뜻하지 않는다. 로봇마다 기존 HOME/Return 설정은 보존한다.

상세 형상이 필요한 경우 이용권이 확인된 `link0.stl`…`link6.stl`을 로컬 디렉터리에
준비하고 Model Assembly의 `mesh_root`를 그 절대 경로로 지정한 로컬 설정을 사용한다.
기본 `primitive` 디렉터리는 단순화 형상을 선택한다. 다른 디렉터리는 STL을 선택하며,
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
- MoveIt KDL에서 기본 형상의 HOME FK/IK, 자체 충돌 및 static Cell 비충돌을 확인했다.
  기존 Tray 중심 부근의 하향 TCP 자세에서 충돌 없는 IK를 찾았다.
- CNC loading의 기존 고정 TCP 자세는 IK가 성립하지만, 기본 근사 collision 형상과
  기존 static scene을 사용한 200 seed 탐색에서 충돌 없는 해를 찾지 못했다.
  도달 불가능의 수학적 증명도, PnP 성공도 아니다. 기존 CNC/셀이나 공통 planner를
  변경하거나 충돌을 임의로 허용하지 않았다.
- 상세 CAD collision, 실제 제조사 영점/부호/프레임, 실물 정확성 및 동역학은 미검증이다.
  기존 셀 전체의 픽/플레이스 경로와 Payload 부착 중 충돌도 미검증이다.
