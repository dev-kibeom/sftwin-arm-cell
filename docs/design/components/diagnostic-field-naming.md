# Diagnostic field naming and compatibility

The shared meaning owner is [Observability Records ICD](../interfaces/icd-observability-records.md); payload generation remains with each producer. These are additive names for existing evidence, not new observations or control inputs. Existing schema versions and legacy keys remain readable. New consumers should use the canonical names below; old records require the stated fallback. Do not count canonical fields and their aliases as separate evidence.

| Canonical field | Legacy field / interpretation |
|---|---|
| Motion `task_result_code`, `task_result_name` | Numeric `task_executor_result_code_value`, string `task_executor_result_code`. Both canonical results are null during planning progress. |
| Motion `record_phase` | `record_status`: `planning_in_progress` or `terminal`; this is execution phase, not persistence success. |
| PICK summary `insertion_started` | `pick_grasp_insertion_started`; `insertion_execution_count` is a legacy 0/1 indicator, not an event counter. |
| PICK summary `material_candidate_search.multiple_material_candidates_searched` | `candidate_switch_occurred`: more than one attempt has `precise_search_began` (legacy missing flag defaults true). This does not establish an executed physical switch. |
| PICK summary `selected_grasp_pose_index` | `selected_candidate`, zero based; -1 means no selection. Scope remains GRASP_POSE. |
| PICK summary `last_evaluated_grasp_pose_number` | `last_evaluated_candidate_number`, one based, paired with `last_evaluated_material_candidate_id`; null means unavailable. |
| Task accuracy `selected_grasp_or_place_pose_index` + `selected_pose_scope` | Zero based for PICK and PLACE. Legacy `selected_planning_candidate_index` remains zero based for PICK and one based for PLACE. |
| Task accuracy `feedback_fk_tcp_pose` | `actual_tcp_pose`: model FK from measured joint feedback, not an independent physical TCP measurement. Existing measurement status governs availability; unavailable pose remains an empty object. |
| Trajectory `final_command_attempt_simulation_time_s` | `final_command_attempt_simulation_time`, simulation seconds; do not compare across simulation epochs without explicit alignment. |
| Orchestration `result_code`, `result_name` | `failure_code`, `failure_name` also contain successful Motion action results; unavailable terminal results remain null / UNAVAILABLE. Read with failure_domain and event; canonical fields are null / UNAVAILABLE without a result domain or available result, while legacy fields retain their defaults. |
| Orchestration `motion_feedback_gap_elapsed_ns` | `feedback_gap_elapsed_ns`, emitted canonically only for feedback gap/recovered events. Legacy field also appears on release/communication events and must not be interpreted as a measured feedback gap there. |
| Safety/Orchestration `capture_complete` | Summary `complete`: writer drained without recorded drops or truncation. This does not mean mission success or full sensor coverage. Missing summary means completeness unknown. |
| Vision ingress `depth_cache.available/reused/status` | `cached_pair_available/reused/status`, Depth cache independent of RGB, explicitly scoped outside legacy `pairing`; explicit RGB-D calibration pair diagnostics retain pair naming. |

`ARM_CELL_MOTION_CLOCK_CAPTURE_ID` overrides Motion Clock capture ID. Nonempty canonical values take precedence; an absent/empty canonical value falls back to `ARM_CELL_OBSERVABILITY_CAPTURE_ID`. This ID belongs to the Clock capture, not a global cross-component trace.

Vision internally names plane collections `cuboid_planes`; serialized `cuboid_height_patches` remains a compatibility key containing plane evidence. The large geometry array is not duplicated. Likewise material candidate IDs, grasp pose indices and attempt order are different identities: generic legacy `candidate_id` keys must be interpreted within their enclosing scope, never globally joined by name alone.

Existing duplicate `final_result`/`mission_exit_reason`, Vision top-level/nested result codes and PICK selected orientation remain compatibility projections. Mission exit reason, Vision `terminal_result`, and PICK `candidate_evaluation` are the authoritative locations for new readers. No legacy field is removed in this migration.

New runs write `diagnostic/session.json` and `diagnostic/<component>/` beneath the reserved run directory. `diagnostic_records` paths remain supported for reading explicitly associated older sessions; directory proximity is not evidence of session membership.
