# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "avt_341_msgs: 14 messages, 1 services")

set(MSG_I_FLAGS "-Iavt_341_msgs:/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(avt_341_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg" ""
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg" ""
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg" "std_msgs/Header:avt_341_msgs/BoundingBox2d:avt_341_msgs/Hypothesis"
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg" "std_msgs/Header:avt_341_msgs/BoundingBox2d:avt_341_msgs/Hypothesis:avt_341_msgs/Detection2d"
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg" "nav_msgs/Path:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:std_msgs/Header:avt_341_msgs/DwaObjective:geometry_msgs/Pose:avt_341_msgs/DwaTrajectory"
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg" ""
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg" "nav_msgs/Path:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:std_msgs/Header:avt_341_msgs/DwaObjective:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg" ""
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg" ""
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg" ""
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg" ""
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:std_msgs/Header:nav_msgs/MapMetaData:geometry_msgs/Pose:avt_341_msgs/OccupiedCell"
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg" ""
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg" "sensor_msgs/PointField:std_msgs/Header:sensor_msgs/PointCloud2"
)

get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv" NAME_WE)
add_custom_target(_avt_341_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avt_341_msgs" "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)

### Generating Services
_generate_srv_cpp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
)

### Generating Module File
_generate_module_cpp(avt_341_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(avt_341_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(avt_341_msgs_generate_messages avt_341_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_cpp _avt_341_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avt_341_msgs_gencpp)
add_dependencies(avt_341_msgs_gencpp avt_341_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avt_341_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)

### Generating Services
_generate_srv_eus(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
)

### Generating Module File
_generate_module_eus(avt_341_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(avt_341_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(avt_341_msgs_generate_messages avt_341_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_eus _avt_341_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avt_341_msgs_geneus)
add_dependencies(avt_341_msgs_geneus avt_341_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avt_341_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)

### Generating Services
_generate_srv_lisp(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
)

### Generating Module File
_generate_module_lisp(avt_341_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(avt_341_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(avt_341_msgs_generate_messages avt_341_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_lisp _avt_341_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avt_341_msgs_genlisp)
add_dependencies(avt_341_msgs_genlisp avt_341_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avt_341_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)

### Generating Services
_generate_srv_nodejs(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
)

### Generating Module File
_generate_module_nodejs(avt_341_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(avt_341_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(avt_341_msgs_generate_messages avt_341_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_nodejs _avt_341_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avt_341_msgs_gennodejs)
add_dependencies(avt_341_msgs_gennodejs avt_341_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avt_341_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)
_generate_msg_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)

### Generating Services
_generate_srv_py(avt_341_msgs
  "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
)

### Generating Module File
_generate_module_py(avt_341_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(avt_341_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(avt_341_msgs_generate_messages avt_341_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv" NAME_WE)
add_dependencies(avt_341_msgs_generate_messages_py _avt_341_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avt_341_msgs_genpy)
add_dependencies(avt_341_msgs_genpy avt_341_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avt_341_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avt_341_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(avt_341_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(avt_341_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(avt_341_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avt_341_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(avt_341_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(avt_341_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(avt_341_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avt_341_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(avt_341_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(avt_341_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(avt_341_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avt_341_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(avt_341_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(avt_341_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(avt_341_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avt_341_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(avt_341_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(avt_341_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(avt_341_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
