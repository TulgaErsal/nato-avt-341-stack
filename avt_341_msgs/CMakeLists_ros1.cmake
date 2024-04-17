find_package(catkin
  REQUIRED
  COMPONENTS
    message_generation
    nav_msgs
    roscpp
    rospy
    std_msgs)

add_message_files(
  FILES
    BoundingBox2d.msg
    Communication.msg
    Detection2d.msg
    Detection2dArray.msg
    FollowerStatus.msg
    Hypothesis.msg
    Obstacles.msg
    OccupiedCell.msg
    OccupiedCells.msg
    Sinkage.msg)

generate_messages(
  DEPENDENCIES
    nav_msgs
    std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    rospy
    std_msgs)