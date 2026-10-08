# What Does PLACE Success Actually Mean in a Robot Cell?

*Portfolio narrative; non-authoritative. Current manipulation and result semantics are owned by Product, Motion, and Orchestration contracts.*

## Problem

An action returning success is not enough to establish that an object was actually grasped or
released. Early deterministic PnP validation exposed timing and lifecycle gaps between a
gripper command, physical capture/attachment, the state reported to ROS, and the later release
of an object.

## Investigation and resolution

The validation work made held state a fresh observation rather than an inference from a close
command. Grasp capture became deterministic through a configured capture volume and contact
width. Attachment occurs only after gripper-close completion, and downstream execution waits
for fresh `HELD`; PLACE similarly requires fresh `RELEASED` and confirmed detach.

The physical fixture lifecycle was reconciled against observed PlanningScene state instead of
assuming a requested attach/detach had taken effect. This made the simulation's physical
semantics observable through the same success boundary expected of execution.

A later question was whether PLACE should also wait for the detached object to fall and settle
inside a position tolerance. Live behavior showed gravity/settling occurs after the accepted
release boundary. That observation was kept separate: manipulation success covers commanded
motion, fresh release, and detach; post-release pose/process quality is a distinct observation.

## Result

The validation path can distinguish command completion from actual held/released state and
can test those physical semantics deterministically. Settling remains useful evidence for a
separate process-quality capability, not a retroactive condition on manipulation success.

## What we learned

Translate physical events into fresh owner-published observations and make their ordering
explicit. Simulation is valuable when it validates the same semantic boundary consumers use,
not when it reports command intent as physical fact.

## References

- [`Motion task execution FDS`](../../components/arm-cell/motion/fds-task-execution.md)
