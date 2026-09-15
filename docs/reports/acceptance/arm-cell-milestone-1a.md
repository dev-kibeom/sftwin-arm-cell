# [SF-Twin] ARM Cell Milestone 1A Acceptance Report

- **Document ID:** `REPORT-ACCEPTANCE-ARM-CELL-1A`
- **Document Type:** `Acceptance Report`
- **Scope:** `ARM Cell / Milestone 1A robot-only Isaac planning profile`
- **Version:** `0.1.0`
- **Status:** `Draft`
- **Report Owner:** `ARM Cell Integration`
- **Acceptance Date:** `Historical acceptance; exact date not recorded in source documentation`
- **Baseline:** `Historical Milestone 1A baseline; exact commit/tag not recorded`
- **Related Design:** `ADR 0001`, `docs/interfaces/arm-cell/icd-sim-state-time-tf.md`

## 1. Acceptance Scope

### 1.1 In Scope

- canonical measured-state projection from Isaac into the ROS planning model;
- robot-model TF propagation and fixed ARM Cell base placement for the tested stage;
- ROS simulation-time participation;
- pause/resume behavior without backward time;
- backward-time/reset latch and complete ROS-profile restart behavior;
- robot-only MoveIt planning profile;
- component/package and live Isaac integration test results reported for the milestone.

### 1.2 Out of Scope

- production trajectory execution;
- safety supervision or safety-rated behavior;
- complete static cell collision geometry;
- permanent runtime reset/recovery policy;
- production gripper behavior;
- Vision/perception acceptance.

## 2. Referenced Verification Requirements

Canonical VR identifiers had not yet been assigned in the legacy design at the time of this
acceptance.

This report does **not** create new VRs. Formal VR mapping is deferred until the relevant
CDS/FDS/ICD migration establishes canonical identifiers.

| Legacy Design Claim | Source Design |
|---|---|
| measured state is projected without simulator-only joint leakage | legacy combined interface source; current mapping: `icd-sim-state-time-tf.md` |
| robot-model TF and fixed base transform have single authorities | ADR 0001; current mapping: `icd-sim-state-time-tf.md` |
| Isaac is simulation-time authority for the profile | ADR 0001; current mapping: `icd-sim-state-time-tf.md` |
| backward epoch change requires complete ROS-profile restart | ADR 0001; current mapping: `icd-sim-state-time-tf.md` |
| profile supports planning without command publication | legacy combined interface source; current mapping: `cds-moveit.md` |

## 3. Procedure

Historical procedure source:

- `docs/guides/arm-cell-isaac-sim.md`
- component/package `colcon test`
- `tests/integration/arm_cell --state-source=isaac`
- operator read-only TF/stage inspection

The documentation migration did not rerun the acceptance.

## 4. Baseline and Environment

| Item | Value |
|---|---|
| Commit / PR / Tag | exact historical identifier not recorded |
| ROS | ROS 2 Humble |
| Simulator | Isaac Sim 5.1 |
| Robot / Cell | M0609 / Robotiq ARM Cell development profile |
| Planning mode | robot-only |
| Evidence origin | user-provided local test and live Isaac verification results preserved in legacy Guide |

The missing exact commit/tag prevents this report from establishing a newly identified
Validated Baseline retroactively.

## 5. Evidence

| Claim | Evidence Reference | Result |
|---|---|---|
| component/package checks | historical Guide acceptance record: `14 tests, 0 failures, 0 errors, 0 skips` | PASS |
| live Isaac integration | historical Guide acceptance record: `9 passed, 1 skipped`; synthetic fault injection excluded in live mode | PASS with stated skip |
| pause/resume keeps monotonic simulation time | historical operator result in Guide | PASS |
| Stop/reset backward-time latch stops canonical publication | historical operator result in Guide | PASS |
| full ROS-profile restart restores publication | historical operator result in Guide | PASS |
| fixed `world -> base_link` matched tested stage | historical operator/OpenUSD result: `[0.150, -0.150, 0.680] m`, identity rotation | PASS for tested historical stage |
| joint-state → TF propagation and Isaac/RViz pose alignment | historical live operator result | PASS |

### 5.1 Cluster 1 reconciliation evidence

The following supplemental live handoff observations were reconciled during the Cluster 1
correction. They verify runtime behavior for the observed session but do not create a tagged
Validated Baseline because no exact commit/tag was captured.

| Claim | Observed runtime evidence | Reuse condition |
|---|---|---|
| Isaac `/clock` authority | one `/clock` publisher, Isaac authority | reusable until `/clock` composition changes |
| robot-model TF authority | one `/tf` publisher, `robot_state_publisher`; `/tf_static` publishers were `robot_state_publisher` and `world_to_base` | reusable until ROS/Isaac TF composition changes |
| `world -> base_link` and stage alignment | ROS `[0.15, -0.15, 0.68]`, identity; Isaac `[0.15000000596046448, -0.15000000596046448, 0.6799999475479126]`, identity; translation/rotation error below `1e-4` | reusable until placement, scene, world/base convention, USD hierarchy/root/base prim, or TF composition changes |
| monotonic pause/resume | historical operator result | `VERIFIED_BUT_TRACEABILITY_PARTIAL`: no exact historical baseline identifier |
| rollback, full ROS-profile restart, and canonical recovery | historical operator result plus adapter rollback tests | `VERIFIED_BUT_TRACEABILITY_PARTIAL`: no exact historical baseline identifier |
| generated robot USD provenance | current GPU-enabled Isaac build generated the provenance manifest from the intended canonical model inputs | reusable while the canonical model and generated-artifact workflow remain unchanged |
| importer visual-reference cleanup | current GPU-enabled Isaac build removed only dangling references for canonical visual-less links; no known dangling visual-reference warnings remained | re-evaluate when importer output or USD post-processing changes |
| simplified Robotiq import | current GPU-enabled Isaac build completed the existing six-joint validation | re-evaluate when gripper import or joint topology changes |

## 6. Results

### 6.1 Passed

The historical Milestone 1A record reports successful component checks and live Isaac
integration for the robot-only planning profile, including measured-state/TF alignment,
simulation-time behavior, and explicit restart recovery after backward time.

### 6.2 Failed

No failed acceptance claim is recorded in the preserved Milestone 1A acceptance record.

### 6.3 Partial / Not Verified

- synthetic fault injection was skipped in the live-mode suite;
- production execution and safety behavior were not part of the milestone;
- no exact historical commit/tag is available in the source documentation.

## 7. Limitations

- The report is reconstructed from preserved documentation and user-provided historical results.
- The documentation migration did not rerun the tests.
- The exact acceptance date and exact repository baseline identifier are not recorded.
- The fixed base-transform evidence applies to the historical inspected stage snapshot/session,
  not every future stage.
- This acceptance does not validate the Milestone 1B static scene or Milestone 1C RGB-D contract.

## 8. Validated Baseline

Historical Milestone 1A behavior is accepted as prior project evidence for the documented
robot-only development profile.

However, because the exact commit/tag is unavailable, this reconstructed report does **not**
promote a new repository state to `Validated Baseline`.

A future baseline may inherit these claims only after dependency-surface review and sufficient
current evidence.

## 9. Invalidation Conditions

The historical validation claims require re-evaluation when relevant dependency surfaces
change, including:

- canonical robot model/kinematics;
- state-projection semantics;
- TF authority or base placement;
- simulation-time/reset behavior;
- MoveIt robot-only planning composition;
- Isaac/ROS integration path.

## 10. Open Risks / Follow-up

- assign canonical VR IDs during MLD/ICD migration and map this report without changing its
  historical claims;
- identify the historical commit/tag if recoverable from git history;
- use current evidence before promoting a future Reviewed Baseline to a scoped Validated
  Baseline.
- regenerate and record derived robot-artifact provenance whenever the canonical robot Xacro
  changes; the simulator construction preflight rejects a missing or mismatched provenance record.

## 11. References

- `docs/adr/0001-arm-cell-model-state-and-time.md`
- `docs/guides/arm-cell-isaac-sim.md`
- `docs/interfaces/arm-cell/icd-sim-state-time-tf.md`
