# Aggregate status for the whole object-tracking module: the per-target tracker
# statuses plus a coarse module-level state.

# Module state enumeration.
# - UNINITIALIZED: the module has no active child trackers.
# - ACTIVE:        the module has one or more active child trackers.
uint8 MODULE_STATE_UNINITIALIZED = 0
uint8 MODULE_STATE_ACTIVE = 1

# Message header stamped on the module status publishing callback.
std_msgs/Header header

# Coarse module state: UNINITIALIZED when `trackers` is empty, ACTIVE otherwise.
uint8 module_state

# Per-target tracker statuses, one per active child tracker.
avt_341_msgs/TrackerStatus[] trackers
