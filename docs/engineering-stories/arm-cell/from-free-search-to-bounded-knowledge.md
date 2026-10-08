# From Free Search to Bounded Knowledge: Eliminating PLACE Planning Latency

*Portfolio narrative; non-authoritative. Current behavior is owned by Motion CDS/FDS and the applicable recipe and interface contracts.*

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

The RawPart recipe now uses that discovered nominal object orientation with a bounded primary
search of ±10° around it. Motion evaluates the nominal and axis-perturbed candidates first and
uses position-tolerance candidates only as a fallback if the primary set fails. Candidate
planning duration ranks successful primary candidates. The change retained existing planning
budgets and did not make the discovered quaternion or tolerance a universal rule.

## Result

After merge, live operation was observed entering PLACE immediately after PICK, confirming the
specific transition that had previously been blocked by planning latency. That observation is
not a general guarantee for other recipes or workcells.

## What we learned

Search can become expensive when a planner is asked to rediscover stable process knowledge on
every request. Put that knowledge at its domain owner, keep object and TCP intent distinct, and
measure candidate behavior separately from trajectory executability.

## References

- [`Motion CDS`](../../components/arm-cell/motion/cds.md)
