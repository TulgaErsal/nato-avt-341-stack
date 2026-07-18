# Measure compute time of code sections
# Can be arranged in hierarchical fashion based on section_id

# Code section being measured. Possible hierarchical id based on slash delimiter, such as <section_1>/<section_1_1>
string section_id

# Mean elapsed compute time of code section over window (if configured) or entire runtime of node
float32 time

# Standard deviation over compute time samples
float32 time_std

# If value > 0, ony use window_num_samples last samples when calculating compute time statistics
int32 window_num_samples -1

# If value > 0, only use samples over last window_time seconds when calculating compute time statistics
float32 window_time -1

# If true, parent node stats are automatically calculated from child node stats.
# window_num_samples and window_time ignored in this case and uses smallest positive value of child elements.
bool auto_parent_stats true

# Compute time children of this node.
ComputeTime[] children