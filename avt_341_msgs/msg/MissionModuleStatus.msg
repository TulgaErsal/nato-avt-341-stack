# Overall mission module status

# Stamped header for the message.
std_msgs/Header header

# Active task information
avt_341_msgs/MissionTaskStatus active_task

# List of queued task descriptions
string[] queued_tasks

# Descriptions of the tasks that ran to completion since the last reset, in completion order.
string[] completed_tasks

# Run time [s] of each completed task, from its first execution start to its completion.
# Parallel to completed_tasks.
float64[] completed_task_durations
