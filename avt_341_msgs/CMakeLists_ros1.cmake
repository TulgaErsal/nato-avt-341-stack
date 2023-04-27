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

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

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


#############
## Install ##
