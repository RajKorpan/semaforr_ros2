# SemaFORR action explanations

`why` is the ROS 2 action-explanation adapter. It consumes
`semaforr_msgs/msg/DecisionRecord` from `decision_records` and publishes a
human-readable explanation on `explanations`.

The node is callback-driven and derives its rationale, veto count, confidence,
task, and execution outcome from typed fields. The topic names are ROS
parameters. There is no positional decision-log parser or runtime text file.
