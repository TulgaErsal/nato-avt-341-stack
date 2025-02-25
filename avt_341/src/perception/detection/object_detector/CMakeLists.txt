message(STATUS "Building object detector node ...")

if(GET_AVT341_DETECTION_MODELS)
  # Download the TorchScript traced models YOLOv8n CPU 640x640
  message(STATUS "Downloading YOLOv8n TorchScript CPU model ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/9q395le8f3xu186tinpcl/yolov8n-cpu-640x640.torchscript?rlkey=93aok5puu4kxx7r0b4uc5r782&st=nsiph3ln&dl=1
    "${PROJECT_SOURCE_DIR}/data/detection/models/yolov8n-cpu-640x640.torchscript"
    EXPECTED_HASH
      SHA256=50751423d8d350559056bfe70272f009a5474d45bc15c767f49316d868aeee72)
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/7z1bpjty9iwlin5ky4fdz/yolov8n-cpu-640x640.names?rlkey=4u6vtcfr1vvutnrj25tmkxpk8&st=y2sju2xy&dl=1
    "${PROJECT_SOURCE_DIR}/data/detection/models/yolov8n-cpu-640x640.names"
    EXPECTED_HASH
      SHA256=634a1132eb33f8091d60f2c346ababe8b905ae08387037aed883953b7329af84)
  # FED Alpha TorchScript traced models YOLOv8m GPU 640x640
  message(STATUS "Downloading FED-A TorchScript GPU object detection model ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/8k6sari7lmhqh1tbznmjd/fedav8m-cuda-640x640.torchscript?rlkey=mrpjx2wbpsb0en4w09cht3dqt&st=3axcy84m&dl=1
    "${PROJECT_SOURCE_DIR}/data/detection/models/fedav8m-cuda-640x640.torchscript"
    EXPECTED_HASH
      SHA256=634b735eb0e91bf88df8bdca8e93b1310783a3c6d96f276a39a8dc9f78659600)
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/1r6q2q34ianpgpmjw1l0t/fedav8m-cuda-640x640.names?rlkey=ykmfd5iexasghuuw41ecwo6k3&st=1qvojmk4&dl=1
    "${PROJECT_SOURCE_DIR}/data/detection/models/fedav8m-cuda-640x640.names"
    EXPECTED_HASH
      SHA256=9d9d7c04edb688ffb3bad0b6307945e7f86aa314c161b52dc58bb04699130f9b)

  # MRZR TorchScript traced models YOLOv8m GPU 640x640
  message(STATUS "Downloading MRZR TorchScript GPU object detection model ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/pfvht4borsie7oww93s3a/mrzrv8m-cuda-640x640.torchscript?rlkey=uhu81fg92g0r02u020cv6trcl&st=il7ex9l2&dl=1
    "${PROJECT_SOURCE_DIR}/data/detection/models/mrzrv8m-cuda-640x640.torchscript"
    EXPECTED_HASH
      SHA256=24ffb7663409865054450e5152de13e5411b210e5a7c421154b86ad34d00584b)
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/icljk1oidyyeo3oxcjcty/mrzrv8m-cuda-640x640.names?rlkey=ghs972f4hrpo2q6ch887zovq7&st=iuxv1xmn&dl=1
    "${PROJECT_SOURCE_DIR}/data/detection/models/mrzrv8m-cuda-640x640.names"
    EXPECTED_HASH
      SHA256=f6ed0f280f9bc2166e5f5ab30de164516ac6f6bf6f440a0a47df302c7dfd07aa)
endif() # if(GET_AVT341_DETECTION_MODELS)

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
  find_package(image_transport REQUIRED)
  find_package(Boost REQUIRED)

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
                            OpenCV rclcpp sensor_msgs vision_msgs image_transport)

  target_include_directories(
          ${PROJECT_NAME}_object_detector_node
          PUBLIC $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>)

  target_link_libraries(${PROJECT_NAME}_object_detector_node
                        ${PROJECT_NAME}_detection_library ${TORCH_LIBRARIES})

  install(
    TARGETS ${PROJECT_NAME}_object_detector_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
