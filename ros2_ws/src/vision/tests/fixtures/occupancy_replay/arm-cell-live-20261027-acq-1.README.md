# 외곽 잔여 셀 교란 후보 회귀 fixture

2026-10-27 Isaac 세션 `97ad72f5-2b4c-4204-8ecf-6b1461ac8f7b`의 Vision
`f3ed6977-b8eb0f9d-ca09a153-acq-1`에서 저장한 실제 관측이다.
원본은 `private local diagnostic artifact (not included in the public package)`의 같은
Acquisition ID를 가진 Diagnostic Record, Heightmap 및 이미지 경계 마스크다.
원본 Depth stamp는 `54.650002850`, optical frame은
`camera_color_optical_frame`이다. Snapshot Metadata의 Acquisition ID도 같다.

- `height_grid.f64le`: 원본 129 × 193 float64 little-endian Heightmap 그대로.
  Tray origin `[-0.24, -0.16] m`, 셀 간격 `0.0025 m`, NaN은 미관측이다.
- `image_boundary.u8`: 원본 이미지 경계 마스크 그대로.
- `json`: 원본 기록의 식별자·관측 Header·Profile·보정·replay 항목을 발췌한
  provenance다. 원본 전체를 복제한 파일은 아니며 원본 경로와 SHA-256을 포함한다.
- `pick-occupancy.txt`: 시작 기준선 `319dfb9e7`의 Detector에 같은 원본 입력과
  Profile을 적용한 정상 PICK·후보별 Occupancy 수치다. 첫 값은 후보 수다.
  각 후보는 `grade x y z yaw length width thickness occupancy_count`, 각 점유는
  `class x y z qz qw dx dy dz` 순서다. 모두 Tray 좌표계다.

기준선 재생은 실제 기록과 같은 교란 후보 194개
(`INCLINED_SURFACE=3`, `ELEVATED_UNRESOLVED=184`, `OTHER_UNRESOLVED=7`)를
생성했다. 수정 후에는 19개이며 정상 PICK 4개와 각 후보의 Occupancy는 수치가
동일했다. 새 후보는 가설의 실제 LOW/UNRESOLVED 셀, 미해결 관측 외곽선 또는
관측 경사면에 연결된다. 잔여 Heightmap 셀의 재분할은 후보 생성에 사용하지 않는다.

같은 세션의 다른 저장 관측도 로컬에서 비교했다. acquisition 2–6의 교란 수는
각각 `201→19`, `188→19`, `163→11`, `158→8`, `168→1`이었다.
여섯 관측 모두 Result Code와 정상 PICK·후보별 Occupancy가 동일했다.
acquisition 2–6의 원본은 이 fixture에 복제하지 않았다.

이 검증은 저장 Heightmap 재생이다. 새 센서 취득, 원본 Depth 재투영, 실제 Push
접촉·경로 실행 또는 교란 성공률을 검증하지 않는다. 대표점은 관측 평균이며
접촉 가능 위치를 뜻하지 않는다. 새 합성 입력의 회귀 테스트와도 구분한다.

SHA-256:

- Heightmap: `b4722fdfc8009b590a98e41f355717faf04f1915c02ea74bcf5e86b36c51408b`
- 이미지 경계: `26b8d78a3d96247ee41c21c468f061508402189c66077c7ca9bccd08ee8e7310`
- PICK·Occupancy 기준값: `580c4b482e0e3b8dd962a8368bc223a33b9a2b5027d8468928753ab451eff8ce`

## 외곽선·Partition 재사용 후속 비교

위의 `194→19` 등은 #693 최초 구현의 역사적 관찰값이다. 후속 수정은 셀 집합의
구성·차집합·중첩 병합을 제거하고 기존 외곽선/독립 Partition과 최종 가설 판정을
직접 연결한다. 대표 XY는 LOW/UNRESOLVED Footprint 교집합의 면적 중심이며,
하나이면 그 중심, 가설이 없거나 교집합이 유효하지 않으면 기존 관측 외곽선의
면적 중심이다. Z는 해당 XY의 관측 높이, 없으면 원래 영역의 유효 높이 평균이다.

| Acquisition | #693 → 후속 | Footprint 교집합 / 단일 가설 / 외곽선 중심 |
|---|---:|---:|
| 1 | 19 → 9 | 4 / 2 / 3 |
| 2 | 19 → 8 | 2 / 3 / 3 |
| 3 | 19 → 8 | 2 / 3 / 3 |
| 4 | 11 → 5 | 1 / 1 / 3 |
| 5 | 8 → 4 | 1 / 1 / 2 |
| 6 | 1 → 1 | 0 / 0 / 1 |

[대표 좌표 CSV](arm-cell-live-20261027-disturbance-representatives.csv)는 여섯 관측의
수정 전후 모든 후보를 별도 행으로 기록한다. Tray 좌표와 기록된 보정/TF로 변환한
`base_link` 좌표, 후속 후보의 원본 외곽선·Partition·가설 ID 및 선정 방식을 포함한다.
`candidate_id`는 각 결과 안에서만 유일하므로 전후 같은 번호를 같은 대상이라고
연결하지 않는다. 전후 후보 수 자체는 정답이나 회귀 assertion으로 고정하지 않는다.
각 후보의 기존 영역 출처와 기하학적 계산을 테스트하며, 여섯 관측 모두 정상 PICK의
개수·순서·등급·Pose, 후보별 Occupancy 및 Result Code는 #693과 그 이전 기준선에서
변경되지 않았다. 원본 Heightmap·마스크 및 PICK/Occupancy 기준값은 수정하지 않았다.

후속 Z 조회 보완에서는 대표 XY와 실제 조회 Grid 셀 중심이 원본 Contour 및 해당
Partition에 포함될 때만 직접 높이를 사용한다. 영역 밖·무효 높이는 대상 영역 평균으로
대체한다. 여섯 관측의 후보 수·Tray XY·정상 검출 결과는 동일하다. 관측 1–5에서는
각각 한 점이 다른 Boundary의 높이 약 `0.13814 m` 대신 대상 평균 약 `0.06844 m`를
사용하며, 관측 6은 Z도 동일하다. CSV는 #694 최초 비교 기록으로 보존하며, 이번
Z 변경 상세는 PR의 검증 절에 기록한다. 기존 Transform을 유지하므로 Z 수정에 따른
base_link 좌표 변화는 같은 Transform으로 결정된다.
