# SemaFORR plan explanations

`why_plan` is the ROS 2 plan-explanation adapter. It consumes
`semaforr_msgs/msg/DecisionRecord` and publishes an explanation on
`plan_explanations` whenever a decision selected a planner.

The explanation names the selected planner objective, task target, decision
latency, and action outcome from typed fields. Crowd density, encounter risk,
and flow semantics are already reflected in the selected planner and do not
need to be reconstructed from visualization grids. Topic names are ROS
parameters; there is no positional decision-log parser.
