# SemaFORR action explanations

`why` is the ROS 2 adapter for the inherited action-explanation engine.
It consumes the current SemaFORR `decision_log` compatibility topic and
publishes human-readable `explanations` plus tabular `explanations_log`
diagnostics.

The input and output topic names and `text_config` path are ROS parameters.
The default text configuration is resolved through the ament package index,
so the node does not depend on a source checkout. Malformed decision records
are rejected before the legacy explanation code parses positional fields.

The tab-separated input is intentionally isolated here as a compatibility
boundary. New navigation behavior should use the structured
`DecisionResult` domain API and should not add dependencies on this wire
format.
