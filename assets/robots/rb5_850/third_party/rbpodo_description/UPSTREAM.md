# RB5-850E 모델 출처

- 공식 저장소: https://github.com/RainbowRobotics/rbpodo_ros2
- 고정 commit: `5e8294a985e7ce5e20e70564c2681130afc5f502`
- 패키지: `rbpodo_description` version `0.0.1`, Apache License 2.0.
  원본 `LICENSE`와 `package.xml`을 보존한다. 해당 패키지의 모델·Mesh에는
  별도 제한 문구가 발견되지 않았으며 Apache-2.0의 수정·재배포 조건을 적용한다.
- 모델: `robots/rb5_850e.urdf`, `robots/rb5_850e/{joint,link}.yaml`,
  `meshes/rb5_850e/{visual,collision}/link[0-6].{dae,stl}`.
  원본 URDF와 YAML은 이 디렉터리, Mesh는 `../../meshes/`에 byte-identical로 보존한다.
  `SHA256SUMS.json`은 이 파일들을 고정한다.
- `robots/rb5_850e/kinematics.pdf`의 제조사 도면은 조사에만 사용했다.
  도면에 별도 재생산 제한 문구가 있어 이 저장소에 배포하지 않는다.

## 모델 정합성과 한계

공식 제품은 [RB5-850](https://rainbow-robotics.com/en/products/rb5-850/)이며
이번 구현은 그 제품군의 **E variant**만 선택한다. `_u`, EA/EN 등 다른 variant의
상호 호환성을 선언하지 않는다. 제조사 도면의 169.2/425/392/110.7/96.7 mm와
원본 URDF의 Joint 원점·flange centre가 일치한다. 웹 제품 페이지는 927.7 mm와
850 mm 표기를 함께 사용하므로 단일 reach 숫자를 근거로 작업 가능성을 판정하지 않는다.

`base, shoulder, elbow, wrist1, wrist2, wrist3` 순서, Z/Y/Y/Y/Z/Y 축,
각 Joint의 ±3.14 rad 위치·3.14 rad/s 속도·10 effort는 **원본 모델의 값**이다.
이를 실장비의 인증된 torque/limit 사양으로 재해석하지 않는다. 특히 effort=10과
동일 속도는 upstream simulation metadata이며 실장비 제어에 사용할 근거가 아니다.
관성·질량도 원본을 보존한다. 별도 acceleration 사양은 만들어 넣지 않는다.

## 로컬 변경

- `../../urdf/rb5_850_macro.xacro`: 원본 URDF를 Xacro macro로 감싼 derivative.
  Mesh URI를 인자로 바꾸고 vendor ros2_control hardware 블록만 제거한다.
  로봇 Link/Joint/Limit/Inertial은 변경하지 않는다.
- Assembly: `base_link`는 vendor `link0`와 같은 원점인 canonical alias다.
  vendor `tcp`는 `link6`에서 (0,-0.0967,0) m 떨어진 flange centre다.
  그 위치에서 Tool +Z를 flange 바깥쪽 -Y로 맞추는 +π/2 X 회전을 사용한다.
  이 결합은 flange 면에 직접 부착하는 이상적인 고정 simulation mount이며,
  실제 adapter plate의 두께·볼트 패턴·케이블 여유를 추정하지 않는다.
- 기존 Tool의 macro·mesh·TCP를 재사용한다. RB5 전용 Tool 구동 코드는 없다.
- SRDF는 canonical `arm` chain과 기존 Tool 의미를 사용한다. upstream의
  `mainpulation` 그룹과 Robot의 sampled `Never` 제외는 복사하지 않는다.
  Robot 인접 Link와 rigid tool mount만 제외한다.
- HOME/Return/Retract는 upstream controller teaching data가 아닌 로컬 simulation
  자세다. Profile의 실제 값에 대해 Limits·충돌·IK를 자동 검사한다.
  동치각 wrap은 비활성화한다(±3.14 bounds 안에서 완전한 2π 회전을 가정하지 않음).
  PLACE IK seed는 해당 모델의 base/elbow를 사용하며 공통 탐색 알고리즘을 재사용한다.
- Isaac importer 설정은 기존과 동일: fix_base=true, merge_fixed_joints=false,
  parse_mimic=false. Robot별 drive 보정은 넣지 않는다. 실제 HOME 안정화·Tool
  구동·충돌 접촉은 사용자 라이브 검증 대상이다.
