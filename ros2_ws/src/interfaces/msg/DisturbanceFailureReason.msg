# Failure classification for a disturbance operation. PLAN_EXHAUSTED means no
# complete approach/push/retract plan was found; EXECUTION_FAILURE means a
# planned operation started but did not meet its measured completion contract.
uint8 FAILURE_REASON_NONE=0
uint8 FAILURE_REASON_PLAN_EXHAUSTED=1
uint8 FAILURE_REASON_EXECUTION_FAILURE=2
# Explicit skeleton result; it is not a runtime execution failure.
uint8 FAILURE_REASON_NOT_IMPLEMENTED=3

uint8 value
