# Diagnostic record semantics implementation plan

**Goal:** Make diagnostic names describe their value, scope, units and observation source while preserving existing readers.

**Architecture:** Add canonical scalar fields beside legacy JSON aliases. Rename stale internal Depth-cache and plane collection identifiers without changing legacy wire keys. Keep control decisions and capture limits unchanged.

**Tech stack:** ROS 2 C++, nlohmann JSON, Python tooling.

**Approved scope:** Conversation audit and naming recommendations approved by the user on 2026-11-11. No new sensor collection or runtime decision changes.

## Tasks

- [x] Extend existing serialization tests for numeric result/name, phase and candidate semantics, retaining legacy fields; observe failures.
- [x] Add canonical Motion fields, normalized zero-based candidate index, feedback FK pose name, explicit simulation seconds and capture environment variable with legacy fallback.
- [x] Clarify Orchestration result field names and elapsed-time scope, preserving old fields.
- [x] Rename internal Vision Depth-cache/plane identifiers; retain existing JSON keys and document their compatibility semantics.
- [x] Correct stale documentation ownership/path references and document canonical versus legacy fields.
- [x] Run affected C++ and Python tests, source checks and diff checks. Request independent review, resolve findings, commit, push and open PR.

## Review focus

Legacy JSON keys retain values; unknown progress results remain null; canonical candidate indexes use one base across PICK/PLACE; missing FK remains unavailable; legacy environment values remain supported with canonical precedence. No bulk duplication of plane geometry arrays.

## Verification

Affected Motion (planning capture and transport), Vision (cuboid, ingress adapter and snapshot), Orchestration (serialization and handoff), and Safety capture C++ suites pass. Python tooling/bringup: 49 passed. Source syntax, package licensing and diff checks pass. Independent static review found no confirmed defect; coverage suggestions for progress, bounded summaries, PLACE and cache compatibility were incorporated.

Internal renames preserve legacy wire keys. approach_execution_count is an actual aggregate and stays unchanged; insertion_execution_count is retained only as a legacy indicator. New sensor evidence, capture-loss manifests and wholesale event-payload redesign are outside this naming migration.
