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
| `loft` | `sections`: `[x,y,z,u반경,v반경]` 단면 배열 (`center` 불필요) |
| `sweep` | `sections`: `[x,y,z,u반경,v반경]` 경로·단면 배열, `normal`: 경로 평면의 법선 (`center`/`axis` 불필요) |
| `ring` | `length`, `outer`, `inner` |
| `bolts` | `pitch_radius`, `count`, 선택적 머리 반경 `size`와 높이 `head_length` |

재질은 `diffuse` RGBA를 정의하며 `specular`, `shininess`, `emission`을 선택할 수
있다. 특정 이름에 금속이나 발광 의미를 부여하지 않는다. `metadata.created`와
`metadata.modified`로 COLLADA의 고정 시간을 지정할 수 있다. 생략 시 epoch를
사용해 반복 생성 결과를 유지한다.

`loft`는 `axis`에 수직인 평행 타원 단면을 순서대로 연결한다. 단면 중심을
옆으로 이동해 꺾인 커버를 표현할 수 있다. 단면은 +axis 순서로 배치하고
첫/마지막 단면의 두 반경을 0으로 두어 닫는다. 단면의 u축은
`normalize(cross(reference, axis))`, v축은 `cross(axis,u)`다.
reference는 일반적으로 +Z이며 axis가 Z에 가까우면 +Y다.

`sweep`는 단면이 경로 접선을 따라 돌아가므로 관절의 굽은 하우징을 하나의
연속 표면으로 만든다. 경로는 `normal`에 수직인 평면 위에 두고, 첫 반경은
normal 방향, 둘째 반경은 `cross(tangent,normal)` 방향이다. 끝점 중심을
반복하고 그 반경을 0으로 두면 평평한 체결면으로 닫는다. 굽힘 반경은 해당
방향의 단면 반경보다 커야 안쪽 표면이 접히지 않는다. 이 형상은 같은 강체
안의 연결부용이며 가동 관절 경계 너머를 하나로 잇지 않는다.

출력은 링크별 `<이름>.dae`와 `revision.xacro`다. revision macro를 모델 Xacro에서
호출하면 생성 mesh의 digest가 확장 URDF에 포함되어 기존 provenance 경로로
변경을 감지할 수 있다. recipe 변경 후 도구를 다시 실행하고 생성 산출물을 함께
갱신한다. 새로운 모델은 별도 Python 코드를 추가하지 않고 recipe를 작성한다.

```bash
python3 -m pytest -q tools/tests/test_visual_meshes.py
```
