# Motion Profile Follow-up Record

## Integration identity

- Scope: Post-acceptance investigation; the acceptance claim was not reopened.

## Baseline and diagnosis

For baseline run `GLOBAL_MOVEIT-1790524038948984-2`, planned duration was
5.38597 s. Joint_1 peak velocity was approximately 0.2618 rad/s at MoveIt and
sampler, and 0.264 rad/s in Isaac actual state. The MoveIt waypoint acceleration
peak was approximately 0.5 rad/s² against the configured 5.0 rad/s² absolute
planning bound. This evidence did not identify sampler command loss or Isaac
tracking as the primary low-speed cause.

The validation profile omitted explicit MoveGroupInterface request scaling.
The observed effective profile was approximately 0.1, identifying implicit
request scaling as the primary cause.

## Correction and live result

The validation profile now explicitly sets velocity and acceleration request
scaling to 1.0. Existing URDF velocity limits and the configured absolute
acceleration bound were not raised.

Post-change live captures `GLOBAL_MOVEIT-1790548909937311-2` and
`GLOBAL_MOVEIT-1790549079398445-2` had planned duration about 1.009 s and
sample span about 1.017 s. Joint_1 peak velocity was approximately
2.500 rad/s at MoveIt, 2.295 rad/s at the sampler, and 2.268 rad/s actual.
Live Isaac execution showed GLOBAL motion materially faster than baseline and
appropriate for the validation scenario. The captures matched the intended
scenario and endpoints; their waypoint counts differed, so they are not
represented as byte-identical waypoint paths.

## Residual and evidence limits

The sampler still does not directly preserve TOTG acceleration semantics, but
this was not observed as the primary cause of baseline low speed; sampler
interpolation was left unchanged. GO_HOME remains a direct position command.

Raw CSVs/manifests were stored temporarily and are not durable. Runtime identity
was not reported. Therefore the retained figures are a compact historical
summary, not independently replayable raw evidence.

This record does not establish a new acceptance qualification.
