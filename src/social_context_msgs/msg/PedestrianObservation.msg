# Stable identity assigned by the upstream tracker.
string id

# Current state in SocialObservation.header.frame_id at header.stamp.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Future positions and their absolute ROS timestamps. Arrays must have equal
# length and timestamps must be strictly increasing.
geometry_msgs/Point[] predicted_positions
builtin_interfaces/Time[] prediction_stamps

# Detection/prediction confidence in [0, 1] and row-major 2x2 XY covariance.
float64 confidence
float64[4] position_covariance
