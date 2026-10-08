# ADR 0010: Separate ARM Cell Motion recipe intent from planning strategy

- **Document ID:** `ADR-ARM-CELL-0010`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Motion planning architecture`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `2026-10-08`
- **Related Design:** MissionRecipe and Motion PLACE planning design
- **Supersedes:** `None`
- **Superseded By:** `None`

## Context

Domain/process intent such as a desired object pose and its nominal constraints/preferences
must reach execution without requiring the planner to rediscover that intent or embedding one
workpiece's assumptions in a planner. Planning policies can change independently as feasibility
and backend capabilities evolve. Object orientation intent and the robot/tool TCP realization
are also different concerns.

## Decision

Recipes express target/process intent and nominal constraints or preferences. Motion planning
realizes that intent through replaceable strategies, including fixed, bounded, or free
orientation policies. Object pose/orientation intent remains separate from TCP/tool geometry
and preference used to realize it. A future planner, backend, or search policy may change
without changing recipe/domain meaning.

RawPart PLACE's transition from a free search to a recipe nominal orientation with bounded
search is an application of this separation. The discovered orientation and its numeric
tolerance are profile decisions, not invariant values established by this ADR.

### Considered alternatives

- **Planner discovers or hard-codes domain intent on each request:** keeps recipes smaller,
  but makes process meaning depend on planner implementation and target-specific branches.
- **Recipe specifies a particular planner algorithm/search realization:** makes execution
  behavior explicit, but ties domain configuration to a replaceable planning policy.
- **Recipe intent realized by replaceable planning strategies (selected):** requires a defined
  translation between intent and feasible candidates, while allowing strategy changes beneath
  stable process semantics.

## Consequences and trade-offs

Recipe schemas must express intent without leaking strategy internals, and Motion must reject
or diagnose intents it cannot realize. Different strategies may find different feasible
realizations while preserving the same accepted process intent.

## Authority boundary

This ADR preserves the architecture rationale only. Current recipe fields, orientation modes,
candidate generation and ranking, planning budgets, and execution behavior remain owned by the
MissionRecipe, Motion CDS/FDS, and relevant ICDs.

## References

- [`docs/components/arm-cell/motion/cds.md`](../components/arm-cell/motion/cds.md)
- [`docs/interfaces/arm-cell/icd-orchestration-motion.md`](../interfaces/arm-cell/icd-orchestration-motion.md)
- PR #242
- PR #246
- PR #270
