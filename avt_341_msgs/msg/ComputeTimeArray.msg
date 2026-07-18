# Root compute times message containing all configured sections' compute times in a node.
# Section compute times may be organized in a hierarchical fashion based on the section_id slash delimiter.

# Message header stamped on tracker info publishing callback.
std_msgs/Header header

# Optional category tag
string tag

# Compute times payload
ComputeTime[] compute_times
