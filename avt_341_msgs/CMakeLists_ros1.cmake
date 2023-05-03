set(REQUIRED_ROS_PACKAGES
    roscpp
    rospy
    std_msgs
    nav_msgs
    message_generation
)

#########################
## add custom messages ##
#########################

find_package(catkin REQUIRED COMPONENTS
  ${REQUIRED_ROS_PACKAGES}) 

add_message_files(
    FILES
    Sinkage.msg
    Obstacles.msg
    Communication.msg
    FollowerStatus.msg
    OccupiedCell.msg
    OccupiedCells.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

#############
## Install ##