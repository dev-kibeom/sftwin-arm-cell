# 시각 mesh 제작 도구

`visual_meshes.py`는 자체 작성한 시각 형상 recipe를 COLLADA mesh로 변환하는
범용 개발 도구다. Python 표준 라이브러리만 사용한다. 렌더러나 CAD importer가
아니며, Isaac/ROS 실행 중에는 호출하지 않는다.

```bash
python3 tools/visual_meshes.py path/to/visuals.json --output path/to/visual
# --output 생략 시 recipe 파일 옆의 visual/ 디렉터리에 생성
python3 tools/visual_meshes.py assets/robots/hcr14/visuals.json
```

recipe는 link-local 시각 형상만 정의한다. 관절·기구학·관성·collision은 기존
URDF/Xacro가 소유한다. 링크 수, 이름, 색상, 재질명은 도구에 고정하지 않는다.

```json
{
  "segments": 32,
  "materials": {
    "blue_paint": {"diffuse": [0.1, 0.3, 0.8, 1], "shininess": 24}
  },
  "links": {
    "housing": [
      {"shape": "body", "center": [0, 0, 0.1], "axis": [0, 0, 1],
       "length": 0.2, "radius": 0.05, "material": "blue_paint"}
    ]
  },
  "revision": {"macro": "visual_revision", "label": "Visual meshes SHA256"}
}
```

길이·반경·중심 좌표는 m 단위이고 `axis`는 형상의 회전축이다.
각 shape의 공통 필드는 `center`, `axis`, `material`이다.

| shape | 형상별 필드 |
|---|---|
| `body` | `length`, `radius`, 선택적 모서리 크기 `edge` |
| `lathe` | `profile`: `[축 방향 위치, 반경]` 쌍의 배열 |
| `ring` | `length`, `outer`, `inner` |
| `bolts` | `pitch_radius`, `count`, 선택적 머리 반경 `size`와 높이 `head_length` |

재질은 `diffuse` RGBA를 정의하며 `specular`, `shininess`, `emission`을 선택할 수
있다. 특정 이름에 금속이나 발광 의미를 부여하지 않는다. `metadata.created`와
`metadata.modified`로 COLLADA의 고정 시간을 지정할 수 있다. 생략 시 epoch를
사용해 반복 생성 결과를 유지한다.

출력은 링크별 `<이름>.dae`와 `revision.xacro`다. revision macro를 모델 Xacro에서
호출하면 생성 mesh의 digest가 확장 URDF에 포함되어 기존 provenance 경로로
변경을 감지할 수 있다. recipe 변경 후 도구를 다시 실행하고 생성 산출물을 함께
갱신한다. 새로운 모델은 별도 Python 코드를 추가하지 않고 recipe를 작성한다.

```bash
python3 -m pytest -q tools/tests/test_visual_meshes.py
```
