# Mission Manager

__Contents__: 
- [Parameters](#mission-manager-parameters)
- [Handlers](#communication-handlers)
- [Contact Handling](#contact-handling)

## Mission Manager Parameters

- The costmap clearing method can be set using the `clear_method` parameter.
- See additional parameters in `base.launch` / `base.launch.py` file.

| Parameter    | Description                                                                                                                                                                                                         |
|---------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| name                  | unique string identifier for the vehicle                                                                                                                                                            |   
| mission_definition_file | loads data on mission points mission                                                                                                                                                              |   
| follow_scale_x          | Scaling factor on x axis. Formation definitions are defined based on 1 'unit'. Follow scale allows user to increase separation between vehicles.                                                  |   
| follow_scale_y          | Same as follow_scale_x but on the y axis.                                                                                                                                                         |
| same_object_distance_threshold  | Minimum distance between two objects for them to be perceived as two separate objects. Used by the object detection/contact management system to avoid revisiting objects.                |

| External Interfaces | Description |  |
|---------------------|-------------|--|
| avt_341/recv_comms  | avt_341::msg::Communication | Channel for sending Communication messages to the mission manager. |
| avt_341/odometry    | avt_341::msg::Odometry | Receives ego odometry |
| avt_341/state       | avt_341::msg::Int32 | Mission manager tracks state updates |
| avt_341/target_contacts | avt_341::msg::Path | List of contact positions stored in a Path message |
| /'veh num'/avt_341/odometry | avt_341::msg::Odometry | Receives odometry of team members. |
|                             |               |
| /avt_341/desired_speed_factor | avt_341::msg::Float54 | |
| /avt_341/leader_odometry | avt_341::msg::Odometry | Publishes the leader's odometry |

## Communication Handlers
### Formation Request
A formation request communication message contains the requested `formation`, the `leader_name`, the `follower1_name`, the `follower2_name`, the `follower3_name`, the `objective` and the `desired_speed`. The receiving vehicle determines which position it is in (leader, follower1, follower2, follower3, or none) and updates its leader status, follower status, current task, and speed accordingly. 

### Acknowledgement
Not fully implemented. Handler simply prints an acknowledgement that another vehicle has acknowledged a previous message sent out by the ego vehicle. This should be used to help track task acceptance by other vehicles. 

### Arrival 
Not fully implemented. Handler receives an announcement from another vehicle that it has arrived at a location. This should be used to help track task performance by other vehicles.

### Task Complete
Not fully implemented. Handler simply prints an acknowledgement that another vehicle has acknowledged a previous message sent out by the ego vehicle. This should be used to help track task performance by other vehicles. 

### Move To
If the ego vehicle is a leader, the moveto task sets a new goal. This currently supports only named mission points defined in the `mission_definition_file`. This should be extended to handle other kinds of objectives including explicit x,y locations. 

### Hold
Not fully implemented. Handler should stop movement tasks and order the vehicle to hold current position. Initial implementation likely to only apply to leaders (as in MoveTo). 

### Shutdown
The receiving vehicle switches to shutdown state. 

### Set Speed
This sets the receiving vehicle's `desired_speed`. 

## Contact Handling
- The Mission Manager expects an object detector to publish a list of contacts on `avt_341/target_contacts`. The system reuses the `Path` message to store the contacts as a list of Poses. 
- The Mission Manager tracks contacts as they come in and notes whether contacts are new, have been investigated, are currently being investigated. 
- The Mission Manager handles contacts by detecting that there is a new contact that has not been investigated and triggers a MoveTo to the x,y of the contact. 
- For testing, the `test_target_detection_node` is defined in `src/perception/test_target_detection_node.cpp`. See [Test Target Detection Node](test_target_node.md). 


### Test Target Detection Node
The `test_target_detection_node` receives the ego vehicle's odometry and publishes a list of visual contacts as a set of poses stored in a `Path` message. 

The `test_target_detection_node` has a `detection_range` parameter that defines how far away it can detect contacts. 
Potential contacts are stored in three arrays: `targets_name`, `targets_x`, and `targets_y` defined in the launch file. 

The name of the contact is stored in the pose's header's `frame_id`. The x and y are stored in `pose.position`. 
