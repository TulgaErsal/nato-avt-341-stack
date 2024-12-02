message(STATUS "Building object detector node ...")

# Download the TorchScript traced models YOLOv8n CPU 640x640
message(STATUS "Downloading YOLOv8n TorchScript CPU model ...")
file(
  DOWNLOAD
  https://www.dropbox.com/scl/fi/9q395le8f3xu186tinpcl/yolov8n-cpu-640x640.torchscript?rlkey=93aok5puu4kxx7r0b4uc5r782&st=nsiph3ln&dl=1
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/yolo/yolov8n-cpu-640x640.torchscript"
  EXPECTED_HASH
    SHA256=50751423d8d350559056bfe70272f009a5474d45bc15c767f49316d868aeee72)

# Install the custom models
message(STATUS "Installing custom models ...")
install(
  DIRECTORY "${PROJECT_SOURCE_DIR}/data/detection/models/"
  DESTINATION
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/detection/models/"
  FILES_MATCHING
  PATTERN "*.torchscript"
  PATTERN "*.names")

if(USING_CATKIN)
  message(
    WARNING
      "Object detector node not yet supported in the catkin build - skipping..."
  )
endif() # if(USING_CATKIN)

if(USING_AMENT)
  find_package(OpenCV REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(vision_msgs REQUIRED)

  # > Lookup CUDA
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    find_package(Torch REQUIRED PATHS ${Torch_CUDA_DIR})
  else()
    find_package(Torch REQUIRED PATHS ${Torch_DIR})
  endif() # if(CMAKE_CUDA_COMPILER)

  add_executable(
    ${PROJECT_NAME}_object_detector_node
    "object_detection_node.cpp" "object_detection_node_executor.cpp"
    "object_detector.cpp")

  ament_target_dependencies(${PROJECT_NAME}_object_detector_node cv_bridge
                            OpenCV rclcpp sensor_msgs vision_msgs)

  target_link_libraries(${PROJECT_NAME}_object_detector_node
                        ${PROJECT_NAME}_detection_library ${TORCH_LIBRARIES})

  install(
    TARGETS ${PROJECT_NAME}_object_detector_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
