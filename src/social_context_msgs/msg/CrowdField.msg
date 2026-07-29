# Derived learned social state. This is an output snapshot, not an alternative
# pedestrian-observation input.
std_msgs/Header header
float64 width_m
float64 height_m
float64 resolution_m
float64 origin_x_m
float64 origin_y_m
uint32 columns
uint32 rows
string estimator
uint64 version
CrowdFieldCell[] cells
