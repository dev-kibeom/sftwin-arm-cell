# [SF-Twin] ARM Cell Milestone 1B Static-Scene Acceptance Report

- **Document ID:** `REPORT-ACCEPTANCE-ARM-CELL-1B-STATIC-SCENE`
- **Document Type:** `Acceptance Report`
- **Scope:** `ARM Cell / Milestone 1B bounded static planning scene`
- **Version:** `0.1.0`
- **Status:** `Draft`
- **Report Owner:** `ARM Cell Integration`
- **Acceptance Date:** `Historical acceptance; exact date not recorded in source documentation`
- **Baseline:** `Historical Milestone 1B baseline; exact commit/tag not recorded`
- **Related Design:** `ADR 0002`, `docs/components/arm-cell/motion/cds-static-scene.md`

## 1. Acceptance Scope

### 1.1 In Scope

- canonical projection of the approved bounded static ARM Cell scene into MoveIt;
- loader application/readback of selected static collision objects;
- PlanningScene reapplication while preserving unrelated world objects;
- deterministic obstacle-aware rerouting in an isolated test-only PlanningScene;
- one deterministic collision-free planning smoke request in the approved bounded real-cell
  static scene.

### 1.2 Out of Scope

- complete cell geometry;
- dynamic/perception-derived obstacles;
- trajectory execution;
- safety behavior;
- task orchestration;
- proof that the partial real-cell scene always contains a useful rerouting corridor.

## 2. Referenced Verification Requirements

Canonical VR identifiers had not yet been assigned in the legacy design at the time of this
acceptance. The Cluster 2 reconciliation maps the preserved claims to the current candidate
static-scene VRs for traceability only; it does not retroactively create validation evidence.

| Legacy Design Claim | Source Design |
|---|---|
| static scene is an explicit bounded profile over the robot-only baseline | legacy combined interface source; current mapping: `cds-static-scene.md` |
| selected canonical objects are applied and read back | legacy Motion design source; current mapping: `cds-static-scene.md` |
| reapplication preserves unrelated PlanningScene objects | ADR 0002 / integration test |
| obstacle-aware planning can reroute around a deterministic test-only obstacle | Motion design / synthetic rerouting test |
| one collision-free request succeeds in the approved bounded scene | Motion design / real-cell smoke test |

| Candidate VR | Reconciled evidence | Traceability limitation |
|---|---|---|
| `VR-MOT-STATIC-01` | historical selected-geometry projection/readback claim; current artifact-projection test | historical commit/tag is unavailable |
| `VR-MOT-STATIC-02` | historical sentinel reapplication claim; current focused loader reapplication test | current test is new evidence, not a historical test record |
| `VR-MOT-STATIC-03` | historical loader/readback claim; current focused loader missing-readback failure test | current test is new evidence, not a historical test record |
| `VR-MOT-STATIC-04` | historical bounded static-scene profile scope; current explicit-profile launch/configuration | exact historical run record is unavailable |

## 3. Procedure

Historical procedure/evidence sources:

- `docs/guides/arm-cell-isaac-sim.md`
- `tests/integration/arm_cell/test_static_scene_integration.py`
- `tests/integration/arm_cell/test_synthetic_rerouting.py`
- `tests/integration/arm_cell/test_real_cell_planning_smoke.py`

The documentation migration did not rerun these tests.

## 4. Baseline and Environment

| Item | Value |
|---|---|
| Commit / PR / Tag | exact historical identifier not recorded |
| ROS | ROS 2 Humble |
| Planning | MoveIt static-scene profile |
| Simulator | Isaac Sim ARM Cell environment used by the milestone |
| Static subset | 12 approved primitives in the preserved design/test sources |
| Evidence origin | historical Guide/design statements plus preserved integration test definitions |

No current test log containing an exact run timestamp/result count was found in the reviewed
documentation set. Therefore this report preserves the historical acceptance claim but does
not invent an execution record.

## 5. Evidence

| Claim | Evidence Reference | Result |
|---|---|---|
| selected static objects are present and geometry/poses can be read back | `test_static_scene_integration.py` acceptance logic; historical Guide states static-scene acceptance verified | PASS historically reported |
| reapplication preserves unrelated world object | sentinel-object path in `test_static_scene_integration.py`; historical Guide acceptance statement | PASS historically reported |
| deterministic test obstacle blocks direct interpolation and MoveIt returns collision-free rerouted trajectory | `test_synthetic_rerouting.py`; historical Guide states rerouting proven | PASS historically reported |
| deterministic small plan in approved bounded real-cell scene returns a collision-free trajectory | `test_real_cell_planning_smoke.py`; historical Guide states real-cell smoke verified | PASS historically reported |

## 6. Results

### 6.1 Passed

The preserved project documentation records Milestone 1B static-scene acceptance as verifying:

- canonical projection/readback of the selected bounded scene;
- PlanningScene reapplication without removing an unrelated object;
- collision-aware rerouting in a deterministic isolated test fixture;
- one collision-free planning smoke request in the approved bounded scene.

### 6.2 Failed

No failed Milestone 1B acceptance claim is recorded in the reviewed project documentation.

### 6.3 Partial / Not Verified

- exact historical test run output/count is not preserved in the reviewed documentation;
- exact commit/tag and acceptance date are not recorded;
- the static scene is partial and does not validate dynamic/perception obstacles or execution.

## 7. Limitations

- This is a historical evidence reconstruction, not a new test execution.
- Presence of integration test code proves the acceptance mechanism exists, not by itself that a
  particular historical run passed; PASS status above is grounded in the preserved Guide/design
  acceptance statements.
- The synthetic rerouting test intentionally uses a test-only obstacle and does not claim the
  partial real cell itself contains a convenient rerouting corridor.
- The real-cell smoke test proves one deterministic collision-free request, not general
  reachability or complete collision coverage.

## 8. Validated Baseline

Milestone 1B is preserved as prior project acceptance evidence for the bounded static planning
profile.

Because the exact historical commit/tag and run record are unavailable, this report does
**not** establish a newly identified repository `Validated Baseline`.

## 9. Invalidation Conditions

Re-evaluate the relevant claims when dependency surfaces change, including:

- canonical static-environment geometry;
- MoveIt static-scene selection policy;
- generated artifact schema/content;
- static-scene loader application/readback behavior;
- PlanningScene frame/transform semantics;
- collision-checking/planning configuration;
- integration-test acceptance logic.

## 10. Open Risks / Follow-up

- recover exact historical commit/test output if available from git/CI/local logs;
- rerun the accepted tests before using a current Reviewed Baseline as a scoped Validated
  Baseline.

## 11. References

- `docs/adr/0002-arm-cell-static-planning-scene.md`
- `docs/components/arm-cell/motion/cds-static-scene.md`
- `docs/guides/arm-cell-isaac-sim.md`
- `tests/integration/arm_cell/test_static_scene_integration.py`
- `tests/integration/arm_cell/test_synthetic_rerouting.py`
- `tests/integration/arm_cell/test_real_cell_planning_smoke.py`
