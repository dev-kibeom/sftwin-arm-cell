# Unified diagnostic recording implementation plan

Goal: unify operational records under diagnostic_records with explicit recording sessions, common snapshot session keys, and lazy creation of data outputs.

Approved design: user approved the structure proposed in conversation. Keep snapshots/YYYYMMDD/HHMMSS/NN with detection.png and occupancy-NN.png. Keep acquisition generation folder contract, asynchronous best-effort recording, and execution behavior. Configuration stays outside disposable records.

- [x] Bringup: join explicit Isaac session or create standalone ROS recording session; publish actual recording paths, use the same date/time key for snapshots, finalize owned session on shutdown. Test no eager snapshot/component directories and session linkage.
- [x] Writers: open event output only for first record, preserve partial successes and warn without blocking execution, unify standalone fallback paths. Test empty, successful and failed writes.
- [x] Isaac producers: put gripper and simulation pose evidence under session/component paths; preserve independent recording segments and session identities. Run affected unit tests.
- [x] Integration: inspect all references, build affected ROS packages, run relevant regression tests, self-review, commit and push new PR with validation and limitations.

Files: bringup operational launch/tests; shared/live_test_session.py/tests; orchestration and safety event writers; motion and vision diagnostic default paths and reference publication; Isaac setup_scene/pose recorder and gripper fallback defaults/tests.

Validation: 10-package build and final 5-package rebuild succeeded; operational/session Python 43 passed; Isaac pose/Gripper/bootstrap 108 passed and related capture/material tests 54 passed; Orchestration storage 3, Safety storage 2, Clock 11, Pick record 3 passed. Bringup ordinary pytest had 131 pass and 3 launch-only failures; rerun with launch harness and isolated DDS domains passed all three (material retry needed a fresh-domain retry; unchanged main test also passed). Existing writer-local limits retained.
