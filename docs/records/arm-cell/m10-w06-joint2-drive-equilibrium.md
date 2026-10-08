# joint_2 Drive-Equilibrium Investigation Record

> Historical provenance only. This record does not establish or change a
> requirement, design contract, verification, or acceptance meaning.

## Integration identity

- Scope: Production runtime integration and end-to-end path investigation.
- Effective ROS domain: `0`; GUI was not used for these two instrumented runs.
- Both runs used the same canonical Isaac scene/runtime scripts, production
  operational ROS composition, separate camera-TF overlay, and Integration
  material request → `MATERIAL_READY` → Orchestration auto-admission path.
  Neither run directly invoked ExecuteCycle or Motion.
- Production operational profile at run time was a pre-existing configuration
  change, not edited by this diagnostic. Its captured copy is in the
  local evidence directory below (SHA-256
  `0e5f0838e13a2a5f7c0ccd09581522a60074077628a2ced94ee5cabb8aa52fa5`). The
  camera snapshot used by the overlay is also retained there.

## Causal-debugging history

The first reported PICK failure was `MOTION_PROGRESS_STALLED` during the
global Approach. At a 50 mm Approach distance, the Object-center Target left
the TCP only about 10 mm above the RawPart top. The gripper envelope interfered
with the part; GUI observation showed the part being pushed/tilted before
insertion. Increasing the production Approach distance to 150 mm separated
that bounded geometry problem: the later GUI observation showed adequate
visible clearance at the Approach pose.

The 150 mm geometry did not remove the joint_2 completion residual. In the
production-drive baseline below, the robot reached the same global Approach
target but joint_2 stayed about 0.023 rad short of the command. This isolated
the residual from the earlier 50 mm clearance failure.

Telemetry then established that the commanded endpoint was present in both
`/joint_commands` and Isaac's drive target, and that ROS position matched the
articulation's measured position (ROS publishes the position rounded to four
decimal places). Effort was far below the configured maximum. The articulation
position was stationary while both the Isaac-reported velocity and ROS
velocity remained near −0.02 rad/s. This is a separate velocity telemetry
discrepancy; it does not explain away the actual position residual.

These observations motivated a one-variable temporary experiment: retain the
production scene, gravity, motion target, damping, effort/velocity limits, and
trajectory behavior, and increase only joint_2 position-drive stiffness by a
factor of four. The experiment was performed in memory through Isaac Dynamic
Control. No production configuration or source was changed.

## Baseline and treatment

The two RGB-D detections produced effectively the same Object and Approach
positions. The baseline Approach joint_2 target was `0.934025228 rad`; the
treatment target was `0.934049428 rad`, a difference of `0.000024200 rad`.
Both runs used the 150 mm Approach seed.

| Observation | Production-drive baseline | Temporary stiffness ×4 |
| --- | ---: | ---: |
| joint_2 stiffness, applied/read back | 35,809.863281 | 143,239.453125 |
| joint_2 damping | 28,647.890625 | 28,647.890625 |
| joint_2 max effort | 9,600 | 9,600 |
| joint_2 max velocity | 2.618 | 2.618 |
| joint_2 drive mode | 1 | 1 |
| joint_2 target | 0.934025228 rad | 0.934049428 rad |
| joint_2 articulation actual, settled | 0.957189083 rad | 0.939721942 rad |
| target-to-actual residual magnitude | 0.023163855 rad | 0.005672514 rad |
| articulation reported velocity near settled point | −0.0214028 rad/s | −0.02001898 rad/s mean in last simulation second |
| finite-difference velocity near settled point | 0 rad/s | 0.00000358–0.00002861 rad/s in last simulation second |
| measured effort near settled point | −107.4237 | −106.6447 mean |

The residual decreased by **75.51%** and the treatment Approach completed
within the existing `0.01 rad` tolerance. Both measured efforts were about
1.1% of the configured max effort, so neither run showed effort saturation.
The products `stiffness × residual magnitude` were approximately `829.49` for
the baseline and `812.53` for the treatment. Their similarity, alongside the
nearly unchanged measured effort, is consistent with a similar static load
balanced at different drive stiffnesses.

The stiffness result strongly supports the drive/load equilibrium hypothesis
for the global Approach residual. It does not identify the exact load source,
and it does **not** promote the 4× diagnostic value to a production setting or
calibration recommendation. The velocity discrepancy remained in the
treatment: actual finite-difference velocity was near zero while the
articulation-reported value remained near −0.02 rad/s.

### Continuation coverage clarification

The treatment run's retained logs show more than the initial Approach
comparison. The temporary stiffness was restored to the production value
immediately after Approach completion and before the insertion command. With
production stiffness restored, the Cartesian insertion reported `COMPLETED`,
the gripper CLOSE command was accepted, and a fresh `HELD` observation with
`grasp=1, attached=1` was received. A retract-phase task was then submitted.

The simulator was stopped before the full PICK/retract mission completed.
Motion subsequently reported `STALE_FEEDBACK`, and Orchestration ended the
cycle with a backend error. Therefore the logs **do** verify insertion and
CLOSE/HELD in this run, but do **not** verify completed RETRACT or a successful
full PICK mission. The terminal stale-feedback result after simulator stop is
not attributed here to drive equilibrium. No end-to-end acceptance claim is
made.

## Bounded tuning and production PICK continuation

The next tuning runs kept the target, material flow, damping, max effort,
gravity, motion tolerances, watchdogs, speed, and trajectory semantics fixed;
only joint_2 stiffness changed. The 2× run still stalled at an Approach joint_2
residual of `0.0115441 rad`. At 2.5×, that residual fell to `0.0092520 rad`,
but the measured TCP position residual was `10.410349 mm`, outside the existing
`10 mm` Approach acceptance limit. The temporary drive was restored after the
rejected Approach. At 3×, Approach acceptance passed at `9.641684 mm` and
insertion/CLOSE/fresh HELD passed; restoring production stiffness before
Retract caused phase 8 to stall at a joint_2 residual of `0.0193246 rad`. This
showed that the drive value must persist through Retract, rather than only
through the global Approach.

The next two runs used the external production setting in
`operational_profile.yaml`; no run-time stiffness treatment or restoration was
applied. Isaac authored the imported joint_2 stiffness of `625.0` at `1875.0`
for 3× and `1718.75` for 2.75×. The applied articulation readbacks were
`107429.585938` and `98477.125`, respectively. Damping stayed `28647.890625`,
max effort `9600`, and max velocity `2.618` in both runs.

| Production profile scale | Approach TCP position error | joint_2 target residual | PICK result |
| ---: | ---: | ---: | --- |
| 2.5× diagnostic | 10.410349 mm | 0.0092520 rad | Approach rejected by existing 10 mm position limit |
| 2.75× production | 9.951426 mm | 0.0082518 rad | Approach, insertion, CLOSE, fresh HELD, and Retract completed |
| 3× production | 9.644898 mm | 0.0075516 rad | Approach, insertion, CLOSE, fresh HELD, and Retract completed |

Therefore `simulation.joint_2_stiffness_scale: 2.75` is the smallest
sufficient value in the tested bounded sequence. It is an initial
production integration value, not final drive tuning. Both full-PICK runs
used the normal Integration material request → `MATERIAL_READY` → Orchestration
auto-admission path; neither directly invoked ExecuteCycle or Motion. The
2.75× run observed a fresh `grasp=1, attached=1` HELD handshake after the 70 mm
CLOSE, followed by a completed phase 8 Retract.

Both production-profile runs automatically entered the next PLACE approach.
Orchestration reported Motion `result_code=9` (`task_type=2`) before a PLACE
approach trajectory was submitted; the PLACE approach diagnostic reported
the current TCP orientation IK candidate infeasible at the configured
mission target. This is a downstream PLACE/configuration limitation, not a
joint_2 PICK failure. The PLACE path was not changed in this tuning work.
The articulation-reported velocity versus finite-difference discrepancy also
remains a separate residual observation. The 2.75× evidence confirms full
PICK, but does not establish the broader integration exit criteria:
PLACE/nominal vertical path continuation and the combined fault/recovery,
Hub-producer, and provenance claims still require their governing evidence on
one baseline/runtime.

These follow-up observations supplement the historical record above. They do
not promote a diagnostic treatment or add requirement, acceptance, or policy
authority.

## Evidence preservation and provenance

The compact CSV/log excerpts, captured runtime profile, and camera snapshot are
retained as local evidence and are not part of this public package. The raw ROS
bag databases were not copied. The decoded ROS position/command,
material-readiness, action status, and feedback excerpts were retained instead.
The local artifact directory is ignored by Git; only this historical record is
intended for version control.

The bounded tuning and production PICK raw logs, low-bandwidth ROS bags, and
their concise outcome summaries are retained as local evidence and are not
part of this public package.

| Follow-up evidence file | SHA-256 |
| --- | --- |
| `tuning_3x_temporary_approach/summary.txt` | `8a5940855500ae8f8d7be15e6b439a340a7720f318b75bffe173782bd407c811` |
| `tuning_3x_temporary_approach/isaac_runtime.log` | `3e06e86dfba21231ff7daed78ddadb167bd535770025e7da83d29cdc6e18ac97` |
| `tuning_3x_temporary_approach/motion.log` | `b0767e6a88c022f1d09d6e85d830e1d706b0eab9469cf46daabef5cbe42cf831` |
| `production_3x_full_pick/summary.txt` | `be29ce69b28e8ddb6282a893fe989b290a66ccad9ffa16f66ee97fdacd2e2b33` |
| `production_3x_full_pick/motion.log` | `1df62ec5167bc4596f4168ca0d9bd57ff7ccc6048ff9783556e7346ab6f5dda8` |
| `production_3x_full_pick/orchestration.log` | `473b262e7b71dee9562d5afd0e15708d19a867434641469c7da7c4bb35d45b8f` |
| `production_3x_full_pick/lowbw_0.db3` | `94e4f6c05be6df935b6519b045cabf5e8972fd481f1dba4670ceb6a556632198` |
| `production_2_75x_full_pick/summary.txt` | `72f934292c564a945507b660cf27e580255ee5311dc6eb10531844b85c16776e` |
| `production_2_75x_full_pick/isaac_runtime.log` | `0aeba51b87e38013e1abf70409ae3ede05fb21671cfbeafb034b9e0252464052` |
| `production_2_75x_full_pick/motion.log` | `30540c5e40db3e8f27289eaa6f5c28fd335638a00fd45071fc69181927e712fb` |
| `production_2_75x_full_pick/orchestration.log` | `86d83eb27f21942421ce4bb79382c2b6dcd9fd757a8385fcc003691a6570d09a` |
| `production_2_75x_full_pick/lowbw_0.db3` | `4ad6785d9f09adcdfef1bc8e03af1c0018f47de760d1ac7820dcf952f2596322` |

| Local evidence file | SHA-256 |
| --- | --- |
| `baseline_joint2_approach.csv` | `9ae14a622527dcee0a2fa903a8eef1d4112674a4cb9c7374b2d3a27c3cd31183` |
| `baseline_isaac_runtime.log` | `1c81697820cb261df4f61fdc7e5764c679b6e112670fd8a7dd3f485d4be2fa33` |
| `baseline_motion_excerpt.log` | `365d2945a333d2f497e78e156bb7b2e8188e853df9a2465a9318266fdcb347de` |
| `baseline_ros_joint2_crosscheck.txt` | `6baff933303b6f6af41aff87883d4bb4d6f5b8dba1fc0d2cbae66a66f29f107f` |
| `treatment_joint2_approach.csv` | `926b80558436d227b80959b9211a016c6170170a2354370a2d1fd8e736c910d9` |
| `treatment_isaac_runtime.log` | `c3b867bf27ce2346e8f89078894401503e692fa039bf9dfe65eba209952f36c6` |
| `treatment_motion_excerpt.log` | `8a1dbc0cb2dde75e5e355fdcf8a0f86a8eca6f5ac701e906dc020a829017afae` |
| `treatment_ros_joint2_crosscheck.txt` | `452185bbda07216bb6625cd037724cd0ca024c906fa72dc2f1f345ceca7a868c` |
| `runtime_operational_profile.yaml` | `0e5f0838e13a2a5f7c0ccd09581522a60074077628a2ced94ee5cabb8aa52fa5` |
| `camera_snapshot.json` | `63d8ee2da51dc909f8ba0ab45851b55021f06f818a25aef25e25a5ffe30cbd10` |

Additional capture limitation: the baseline telemetry gate opened after the
Approach command had started, so its CSV does not cover the initial movement.
It does cover the stationary target-residual interval through the stall. The
treatment CSV covers the target approach and settled interval. This historical
record preserves those limits and is not a new acceptance or verification
authority.
