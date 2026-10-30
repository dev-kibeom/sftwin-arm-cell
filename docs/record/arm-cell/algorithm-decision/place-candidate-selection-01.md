# From Free Search to Bounded Knowledge: Eliminating PLACE Planning Latency

*Algorithm Record; historical implementation, not a normative specification. Current behavior is owned by Motion CDS/FDS and the applicable recipe and interface contracts.*

## Problem

The production PnP path could complete PICK and then spend too long trying to plan PLACE.
MoveIt reported planning success in some traces, but that did not guarantee an executable
trajectory: the dedicated candidate path had also retained raw, untimed plans. The team had to
separate search latency, candidate validity, and execution readiness instead of treating
“planner succeeded” as the end-to-end result.

## Investigation and change in judgment

The initial PLACE work first repaired the semantic and execution path: propagate recipe
geometry into Motion, align the planning scene with the cell, and ensure selected candidates
were time-parameterized and validated. That established that a plan could be executable, but
did not explain the long candidate search.

The key distinction was between the desired **object orientation** and the **TCP orientation**
used by the robot. A free search kept comparing many object orientations even though live
placement experience had already revealed a stable process orientation for this RawPart. The
useful knowledge belonged in the recipe as a nominal object intent; copying a TCP preference
there would have conflated process geometry with robot/tool realization.

## Resolution

At that stage, the RawPart recipe used the discovered nominal object orientation with a bounded
primary search of ±10° around it. Motion evaluated the nominal and axis-perturbed candidates
first and used position-tolerance candidates only as a fallback if the primary set failed.
That implementation compared successful primary plans by planning duration. It retained the
existing planning budgets and did not make the discovered quaternion or tolerance a universal
rule.

## Result

After that change, live operation was observed entering PLACE immediately after PICK, confirming
the specific transition that had previously been blocked by planning latency. That observation
is historical and is not a general guarantee for other recipes or workcells.

## Algorithm update in PR #430

The current implementation keeps the Recipe-generated `FIXED`, `BOUNDED`, and `FREE` candidate
sets, the existing IK seed order, and the `BOUNDED` nominal-primary / position-fallback groups.
Before IK, candidates are ordered by applicable tool-orientation preference, pose deviation,
then the existing seed order. Motion then evaluates one candidate at a time: 6D IK, joint
continuity and bounds, Planning Scene state validity, Global Planning, and trajectory
time-parameterization. The first executable trajectory is selected and ends candidate
evaluation. Later candidates receive no IK call after that success. The previous all-candidate
IK/collision precomputation and comparison of successful plans by planning duration have been
removed. Post-IK joint delta remains in candidate diagnostics but does not reorder the Recipe
priority. `BOUNDED` falls back to its position group only after the primary group has no
executable trajectory.

This sequence does not establish global optimality for joint travel or path duration. Each
candidate Planning call retains its 15 s limit and all candidate Planning calls share the
existing 60 s planning-time budget; IK and state-validity checking are not counted as Planning
time. Safety preemption terminates evaluation and discards late results. Focused offline tests
cover candidate priority and early termination. BOUNDED group limits apply to the actual
pose-candidate × IK-seed combinations; a terminal Safety interruption in the primary group
does not enter position fallback. Live Planning Scene and Isaac execution remain separate
runtime verification.

## What we learned

Search can become expensive when a planner is asked to rediscover stable process knowledge on
every request. Put that knowledge at its domain owner, keep object and TCP intent distinct, and
measure candidate behavior separately from trajectory executability.

## References

- PR #225, PR #240, PR #241
- PR #245, PR #246, PR #253, PR #270
- [`Motion CDS`](../../../design/arm-cell/components/motion/cds.md)
