# Overall mission module status

# Stamped header for the message.
std_msgs/Header header

# Change that produced this snapshot (NONE = unspecified, e.g. publishers other than the mission
# manager). The topic is latched: a late joiner receives the last change, not a fresh event.
uint8 NONE=0          # Unspecified
uint8 RESET=1         # Mission manager reset; task list and completed history cleared
uint8 TASK_ADDED=2    # Task queued, preempted in, or inserted
uint8 TASK_STARTED=3  # Front task began executing
uint8 TASK_DONE=4     # Task completed or cancelled and removed from the list
uint8 TASK_UPDATED=5  # Active task changed in place (e.g. speed setpoint)
uint8 change_type 0

# Active task information
avt_341_msgs/MissionTaskStatus active_task

# List of queued task descriptions
string[] queued_tasks

# Descriptions of the tasks that ran to completion since the last reset, in completion order.
string[] completed_tasks

# Run time [s] of each completed task, from its first execution start to its completion.
# Parallel to completed_tasks.
float64[] completed_task_durations
