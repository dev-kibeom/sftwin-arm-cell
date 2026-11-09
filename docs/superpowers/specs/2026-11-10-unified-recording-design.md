# Unified operational recording

Approved scope: consolidate diagnostic output beneath diagnostic_records/YYYYMMDD/HHMMSS, using one session descriptor across Isaac and ROS participants. A ROS-only launch creates its own session. Session creation records a meaningful start event; component folders, event files and snapshot folders materialize only on data writes. Snapshots remain snapshots/YYYYMMDD/HHMMSS/NN with detection.png, metadata.json and occupancy-NN.png; the Generation numbering contract is unchanged.

Use one local timezone-aware session timestamp for both directory roots. Preserve existing record IDs for joins between missions, acquisitions and motion goals, and record process identities and actual destinations in session metadata. Standalone components retain diagnostic_records/unassociated/component fallbacks. Attach camera/configuration provenance where available; calibration/configuration remains outside disposable artifacts.

Keep asynchronous bounded recording, partial file success tracking, atomic record writes and warning-only failure behavior. Queued references must describe pending output rather than saved files. Generic sessions are independent of mission start. Closing a participant must not close an Isaac-owned session; ROS owns closure only for its own descriptor. Unknown/abnormal closure is not success.

Alternatives: renaming observability alone leaves split paths and missing ROS-only session context; mission-owned roots lose pre-mission and calibration evidence. The approved approach uses the existing session mechanism and per-component writers without adding a directory service or new synchronization layer.

Compatibility: keep existing motion subdirectory and diagnostic schemas where possible, retaining UUIDs as record identifiers while snapshot filenames remain readable. Existing historical artifacts are not migrated or deleted. Retention and drop limits remain writer-local; a global destructive pruning service is outside this change.
