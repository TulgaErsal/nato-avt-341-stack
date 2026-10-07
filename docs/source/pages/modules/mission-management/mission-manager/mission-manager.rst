Mission manager
===============

- The costmap clearing method can be set using the `clear_method` parameter.
- See additional parameters in `base.launch` / `base.launch.py` file.

Communication Handlers
----------------------

Formation Request
^^^^^^^^^^^^^^^^^

A formation request communication message contains the requested `formation`,
the `leader_name`, the `follower1_name`, the `follower2_name`, the
`follower3_name`, the `objective` and the `desired_speed`. The receiving vehicle
determines which position it is in (leader, follower1, follower2, follower3, or
none) and updates its current task and speed accordingly. The resulting
formation role and offsets are published on the latched task-change topic.

Acknowledgement
^^^^^^^^^^^^^^^

Not fully implemented. Handler simply prints an acknowledgement that another
vehicle has acknowledged a previous message sent out by the ego vehicle. This
should be used to help track task acceptance by other vehicles.

Arrival
^^^^^^^

Not fully implemented. Handler receives an announcement from another vehicle
that it has arrived at a location. This should be used to help track task
performance by other vehicles.

Task Complete
^^^^^^^^^^^^^

Not fully implemented. Handler simply prints an acknowledgement that another
vehicle has acknowledged a previous message sent out by the ego vehicle. This
should be used to help track task performance by other vehicles.

Wait Until Complete
^^^^^^^^^^^^^^^^^^^

`<sender>,<msg_id>,WAIT_UNTIL_COMPLETE,<receiver>,<target_vehicle>,<target_msg_id>,<priority>`

Queues a task on the receiving vehicle that holds its task queue until
`<target_vehicle>` reports `TASK_COMPLETE` for its task `<target_msg_id>`. In the
`Communication` message the target vehicle is carried in `objective_name` and the
task id in `target_msg_id`. Only a completion received after the wait message
counts, so a reused task id does not release a new wait. Send the wait before the
target task can finish (for example, together with it); a wait sent after the
target task already finished does not release and has to be cancelled.
Cancelling the target task releases the wait, since a cancelled task reports
`TASK_COMPLETE`. A `CANCEL` or `CANCEL_ALL` message also reports `TASK_COMPLETE`
under its own message id. A wait matches only the target vehicle and message id,
not the commander, so message ids sent to a vehicle must be unique across all
commanders and message types: a task or cancel sent to the target vehicle under a
waited-on id releases the wait when it completes.

When the wait starts, the vehicle is given a hold goal at its current position,
so a `PREEMPT` wait stops a moving vehicle. The preempted task resumes after the
wait. A wait whose target vehicle is the receiving vehicle itself is rejected.

A formation command creates a task on every vehicle in the formation, so to hold
a formation send a `WAIT_UNTIL_COMPLETE` to each of its vehicles.

Move To
^^^^^^^

If the ego vehicle is a leader, the moveto task sets a new goal. This currently
supports only named mission points defined in the `mission_definition_file`.
This should be extended to handle other kinds of objectives including explicit
x,y locations.

Hold
^^^^

Not fully implemented. Handler should stop movement tasks and order the vehicle
to hold current position. Initial implementation likely to only apply to leaders
(as in MoveTo).

Shutdown
^^^^^^^^
The receiving vehicle switches to shutdown state.

Set Speed
^^^^^^^^^

This sets the receiving vehicle's `desired_speed`.

Contact Handling
----------------

- The Mission Manager expects an object detector to publish a list of contacts on `avt_341/target_contacts`. The system reuses the `Path` message to store the contacts as a list of Poses.
- The Mission Manager tracks contacts as they come in and notes whether contacts are new, have been investigated, are currently being investigated.
- The Mission Manager handles contacts by detecting that there is a new contact that has not been investigated and triggers a MoveTo to the x,y of the contact.
- For testing, the `test_target_detection_node` is defined in `src/perception/test_target_detection_node.cpp`. See [Test Target Detection Node](test_target_node.md).

Test Target Detection Node
^^^^^^^^^^^^^^^^^^^^^^^^^^

The `test_target_detection_node` receives the ego vehicle's odometry and
publishes a list of visual contacts as a set of poses stored in a `Path`
message.

The `test_target_detection_node` has a `detection_range` parameter that defines
how far away it can detect contacts. Potential contacts are stored in three
arrays: `targets_name`, `targets_x`, and `targets_y` defined in the launch file.

The name of the contact is stored in the pose's header's `frame_id`. The x and y
are stored in `pose.position`.


Subscriptions
-------------

.. csv-table:: Subscriptions
   :file: subscriptions.csv
   :header-rows: 1

Published Topics
----------------

.. csv-table:: Published topics
   :file: publishers.csv
   :header-rows: 1

Parameters
----------

.. csv-table:: Parameters
   :file: parameters.csv
   :header-rows: 1
