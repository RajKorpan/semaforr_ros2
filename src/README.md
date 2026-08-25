
# Social-SemaFORR Packages

This folder contains the ROS 2 Social-SemaFORR workspace:

- `semaforr`: ROS-independent navigation core plus the ROS node.
- `semaforr_msgs`: structured navigation diagnostics.
- `social_context_msgs`: the canonical social observation and learned crowd
  field contracts.
- `social_context`: social observation and trajectory prediction producers.
- `semaforr_bridge`: adapters from supported upstream tracking messages.
- `why`: unified typed action and plan explanations based on
  `semaforr_msgs/msg/DecisionRecord`.
- `hunav_msgs`: HuNavSim interfaces.
- `examples`: reproducible workspace examples.

All packages are ROS 2 packages. Superseded ROS 1 implementations and private
message copies have been removed; Git history remains the migration archive.
See each package README for its runtime contract.
