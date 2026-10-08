# Distinguishing a Safety Startup Transient from a Runtime Fault

*Portfolio narrative; non-authoritative. Current Safety cause, latch, supervision, and reset semantics are owned by Safety FDS/CDS and ICD.*

## Problem

Safety supervision inputs may be absent, stale, or invalid while the system is still starting.
Treating that interval as an established runtime communication fault can latch Safety before
normal supervision ever existed. Ignoring the same loss after supervision was established,
however, would hide a real runtime fault.

## Investigation and resolution

The analysis separated “normal supervision has never been established” from “previously
healthy supervision disappeared.” Startup readiness transients retain an unknown/non-permitted
state without creating the communication/readiness latch. Once healthy supervision has been
established, invalid or stale required inputs retain their runtime fault meaning and may invoke
the Safety-owned stop path for active execution.

This exception was deliberately narrow. E-Stop and STO remain genuine Safety causes even
during startup and retain their latch/reset requirements. Clearing their physical input does
not itself reset the latch or authorize motion.

## Result

The cell can become available after ordinary startup input readiness recovers without a false
fault latch, while runtime input loss remains fail-safe. The same distinction preserves
startup E-Stop/STO behavior instead of weakening safety response for convenience.

## What we learned

Fail-safe behavior requires knowing whether supervision was ever established. Readiness and
fault are not synonyms, and a narrow startup accommodation must not swallow explicit Safety
causes.

## References

- PR #257, PR #258, PR #259
- [`Safety supervision FDS`](../../components/arm-cell/safety/fds-supervision.md)
- [`Safety stop/recovery FDS`](../../components/arm-cell/safety/fds-stop-recovery.md)
