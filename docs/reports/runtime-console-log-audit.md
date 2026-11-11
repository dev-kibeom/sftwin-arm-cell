# Runtime console log audit

Baseline: public `main` at `a68df29` (2026-11-11). Scope: presentation and observation only. No runtime algorithm, interface, result classification, budget, retry, stop condition, robot model, or diagnostic schema changes. After the initial audit, the user also requested removing hard-coded test expectations; the existing generator now synchronizes stale MoveIt Scene artifacts with the already-approved canonical manifest.

The supplied live transcript confirms repeated successful trajectory messages, duplicate gripper command messages, empty PLACE observation references, 12 successful PnP terminals without cycle summaries, a long PLACE candidate search, transient timestamp-quality edges, and a real PLACE accuracy warning. The excerpt ends at the twelfth PnP terminal; it does not establish the subsequent ExecuteCycle terminal result. Its launch timestamps are not precise action-duration evidence.

## Audit and disposition

| Producer / event | Console decision | Preserved evidence / rationale |
|---|---|---|
| Orchestration mission start / terminal | INFO start quantity, terminal count and reason; no IDs in normal summaries | Existing mission records, Action lifecycle, terminal failure trace retain IDs and raw Enum names |
| Orchestration mission phase / successful individual Motion / target acquired | INFO → DEBUG | Existing phase diagnostics and feedback; cycle terminal summary owns normal progress |
| Orchestration successful PnP | New single INFO after terminal observation and coordinator material-count commit | Outcome must be COMPLETED, Action SUCCEEDED, material placed. Local steady request-to-terminal duration excludes preceding Vision acquisition; includes Action admission and final HOME. Duplicate/non-advancing count callbacks produce no second summary |
| PnP failure / placed but final HOME incomplete | WARN with observed outcome, time, count, optional detail; never call a placed-only result a completed cycle | Existing coordinator still owns counting, recovery and mission outcome. Safety handoff and missing result remain distinguishable in Action records |
| Retry / controlled RETRACT completion | INFO on retry-count advancement / successful recovery result | Existing retry feedback, recovery diagnostics and policy unchanged |
| Material Handoff preparing / provider completion | INFO preparing quantity; actual valid MaterialReadiness transition owns supply-ready type/count; provider completion is DEBUG | Existing completion validation, readiness message and delivery identity. Supply ready requires both accepted provider completion and VDA UNLOADED evidence; the coordinator publishes it once per delivery |
| Gateway pending readiness / Orchestration admission waits | INFO → DEBUG | Readiness/admission state and causal diagnostic events remain; preparing/ready/mission start now own normal INFO narrative |
| Motion candidate selection / planning success / final strategy / generated trajectory | Consolidated into one DEBUG plan-ready line with task, phase, path and strategy | Existing candidate evaluation JSON, PICK records, selected path and continuity validation remain; rejected path and backend failures retain severity |
| Normal trajectory validation / execution-start DEBUG | Redundant console calls removed | Validation itself, rejection WARN, normalized operation, Action feedback and existing optional trajectory/profile trace retained |
| Motion state transitions / gripper HELD or RELEASED observation | Retain DEBUG | Actual observed execution/holding boundaries differ from command issuance; useful without dumping candidate internals |
| Backend gripper-open command | Remove duplicate DEBUG; Task Execution owns one command result line | Task Execution reports named result, while backend holding observations remain separate. Port calls/results and PICK/release records unchanged |
| MoveIt Planning Scene / RobotState conversions and state | Existing `planning_scene_log_level` controls all three namespaces; default INFO even with project DEBUG | WARN/ERROR retained; `planning_scene_log_level:=debug` restores raw details. Project DEBUG and diagnostic selection unchanged |
| Vision acquisition, rejection, snapshot writer | Retain existing DEBUG / WARN / ERROR | Already terminal-focused; snapshots, capture queue limits, JSON and image evidence unchanged. Only redundant Orchestration acquisition INFO demoted |
| Safety source timestamp quality edge | WARN/INFO → DEBUG, edge-deduplicated as before | Existing diagnostic `source_stamp_plausible` retained; the Safety FDS and Safety–Motion ICD severity rows are minimally aligned with the new console severity. Current tracker has no sustained-duration policy; no new threshold or stop. Receipt freshness, missing service, rejected stop and Safety output WARN unchanged |
| Motion recipe accuracy outside tolerance | Retain WARN and measurements | Existing pose/error computation, tolerances and accuracy JSON unchanged; supplied 29.903mm / 2.941deg is meaningful |
| Gateway failed transfer / Motion backend or planning failure / Vision actual request failure | Retain existing WARN/ERROR | Actual confirmed failures; no logging call controls execution. Normal IK/candidate rejection stays in structured attempts rather than a new ERROR |
| VDA mock ARRIVED/DOCKED/UNLOADED/DEPARTED | Retain INFO | Distinct external handoff transitions, not proof of material supply; only four transitions per delivery |
| Joint State adapter startup / recovery / invalid input | Retain existing INFO / WARN | Actual state availability changes, bounded startup readiness messages |
| Isaac material sidecar, gripper runtime, scene setup and UI Hub | No changes | Provider failures and simulator lifecycle output have separate owners; Isaac runtime behavior and RawPart pose / gripper evidence unchanged |

Candidate budgets and diagnostic evidence remain at their original producer sites in `isaac_ros_motion_transport.cpp`, `pick_execution_record.cpp`, `vision_detector_snapshot.cpp`, `safety_node.cpp`, and `orchestration_diagnostic_record.hpp`. Console Enum projection does not alter JSON Enum names. No diagnostic producers or persistence calls were removed. No new normal-path file I/O, timer threads, services or logging framework were introduced.

## Delayed planning

PLACE uses its existing candidate callback boundaries to observe local steady elapsed time. After 30 seconds it emits one WARN per search and an INFO on successful recovery after the existing interruption-discard check. Counts are actual callback invocations. Qualification and trajectory planning are named separately. Short searches are silent. A single blocking IK/planner call cannot emit a message until its boundary returns; the warning is observational, not a timeout. No planner budget or control condition consumes these observations.

## External warnings and time

- The canonical `m0609_macro.xacro` gives root `base_link` an inertial element; KDL cannot model root inertia. The warning is retained. Adding a dummy root would change the model/TF contract and is outside this log change.
- `motion/config/moveit/joint_limits.yaml` explicitly leaves the planning-only acceleration limits empty. TOTG can therefore report its default acceleration; this is retained. Runtime tuning and validation acceleration policy are not replaced with an invented robot limit.
- Existing raw OMPL/constraint/collision logger overrides are unchanged. This change adds no WARN/ERROR suppression and selectively handles the independently named RobotState namespaces from the supplied transcript.
- `ConsoleOutputFormat` prefixes Launch-observed output using local monotonic time and writes the existing `ros_output_nanoseconds.log` with a shared session origin. It explicitly identifies collection time, not ROS event time. Formatting instances also have individual elapsed origins, so merged lines may appear out of order (as in the supplied 81.312s / 81.104s example). Do not sort that output to infer Action causality.
- Message source stamps, node ROS `/clock` time and local steady receipts have distinct semantics under `icd-observability-records.md`. Existing JSON timestamps and correlation IDs remain authoritative within their stated domains. No clock synchronization or central collector added.

## Before / after examples

Before (supplied transcript):

```text
[INFO] mission phase started phase=DETECT_TARGET processed=0/12
[INFO] Vision target acquired
[INFO] mission phase started phase=PICK processed=0/12
[DEBUG] ExecutePnp terminal ... outcome=0 material_placed=true action=SUCCEEDED
[INFO] mission phase started phase=DETECT_TARGET processed=1/12
```

New format (illustration; durations below are test-fixture values, not a new live run):

```text
[INFO] [Material] Supply ready: RawPart 12/12
[INFO] [Mission] Started: 12 materials
[INFO] [Mission] Initial HOME completed
[INFO] [PnP] Cycle 1/12 completed: 44.4s
[INFO] [PnP] Cycle 2/12 completed: 22.1s
```

A terminal quantity-reached mission emits `[Mission] Completed: 12/12 reason=quantity_reached` only after the actual mission result. Runtime durations always come from measured steady time; no example timing is hard-coded.

Before, one normal phase could print candidate selection, planning valid, generated trajectory, final strategy, validation passed and execution began. Now its planning success is one DEBUG line, for example `Motion plan ready: task=PLACE phase=execute path=CARTESIAN strategy=CARTESIAN`. Phase preparation and measured completion remain separate DEBUG observations. Empty references are omitted, e.g. `PLACE preparing phase=approach`.

## Follow-up test maintenance requested by the user

- Geometry/count expectations are derived from canonical manifest and selection; required collision-coverage IDs and exclusion contracts remain explicit. Removed duplicate numeric geometry expectations, including the obsolete camera post coordinate.
- Regenerated standard and PnP MoveIt Scene artifacts using the existing generator. Only CameraRig PostRight x and the associated TopBeam width/center, plus manifest hashes, change to match current main's canonical geometry. This is a generated-artifact correction, not a collision algorithm or new installation geometry policy.
- Moved the two Action/Safety integration launch tests to Task Execution, the current owner of the legacy ExecuteTask Action endpoint, and launch its executable rather than Motion's backend-only node. Test dependencies are registered there, avoiding a Motion → Task Execution dependency cycle.
- Updated log-boundary assertions for the new readable output and derive material-ready expected text from the input completion instead of duplicating material/quantity literals.

## Validation

- Affected packages built successfully: Motion, Task Execution, Orchestration, Safety, Gateway, Bringup. Unchanged ROS interfaces and other dependencies came from the existing workspace installation.
- Sequential ROS-enabled CTest run: **96/96 selected test entries passed** (Orchestration 17, Safety 6, Gateway 3, Motion 41, Task Execution 13, Bringup 16). This includes the relocated Action/Safety launch tests, material handoff, production material flow, diagnostic records, interruption/handoff races, duplicate material accounting, trajectory/candidate policy and Snapshot/Trace-related tests.
- Separate launch/MoveIt configuration pytest run: **52 passed** (overlaps the CTest configuration coverage; not additive).
- Repository CI source checks passed: Python compilation for Isaac tooling, MIT package metadata for all 10 packages; `git diff --check` passed.
- The first broad CTest attempt also ran `test_full_cell_synthetic_cpp`: it failed nominal scenario expectations and then timed out. This unchanged test wires only `execute_task`, whereas the current Coordinator uses `execute_pnp`; its older whole-cell seam requires a separate migration, beyond removing hard-coded test values. The final 96-entry run explicitly excludes this known failing test; it is not counted as passing.
- `uncrustify` failures already exist on unchanged main (`safety_node.cpp` and `test_safety_input_tracker.cpp` reproduced from a clean `git archive origin/main`). `xmllint` could not load the external ROS package schema. These lint entries are excluded from the 96-entry run and are not claimed green.
- Earlier ROS tests failed inside the sandbox because UDP socket creation was prohibited. Final ROS tests ran with communication permitted, a dedicated ROS domain and sequential package execution. Log files were redirected to `/tmp` because the default ROS log location is read-only.
- Live Isaac execution is not claimed: the supplied transcript is baseline evidence, not a post-change replay. Actual live log-volume reduction and delayed-search output timing remain to be measured in Isaac.
