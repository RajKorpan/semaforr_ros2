# SemaFORR plan explanations

`why_plan` is the ROS 2 adapter for the inherited plan-explanation
engine. It consumes the current SemaFORR `decision_log` compatibility topic
and the `crowd_density` and `crowd_risk` diagnostic grids produced by
`semaforr_crowd`.

All topic names and the `text_config` path are ROS parameters. Configuration
is resolved through the ament package index. Crowd grids are optional: plan
explanations continue with the legacy non-social objectives when no valid
grid has arrived. Malformed decision records are rejected, and a missing
alternate planner selects the deterministic distance fallback.

The positional decision log is a compatibility boundary for the inherited
language generator. New planners and advisors should exchange typed domain
results rather than extending this format.
