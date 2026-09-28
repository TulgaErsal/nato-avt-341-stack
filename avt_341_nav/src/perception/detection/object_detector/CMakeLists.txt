message(STATUS "Building object detector node ...")

set(Torch_DIR $ENV{Torch_DIR} CACHE PATH "Path to libtorch (CPU) install")
set(Torch_CUDA_DIR $ENV{Torch_CUDA_DIR} CACHE PATH "Path to libtorch (CUDA) install")

option(GET_AVT341_DETECTION_MODELS "Download the object detection models at build time" ON)

# Download a detection model file when it is missing or does not match the
# expected hash. Should not crash if download cannot be completed (ex: no internet)
function(avt_341_get_detection_model_file FILE_NAME URL SHA256)
  set(MODEL_FILE "${PROJECT_SOURCE_DIR}/ml_weights/detection/models/${FILE_NAME}")
  if(EXISTS "${MODEL_FILE}")
    file(SHA256 "${MODEL_FILE}" MODEL_HASH)
    if(MODEL_HASH STREQUAL SHA256)
      return()
    endif()
    message(STATUS "Detection model ${FILE_NAME} hash mismatch "
                   "(got ${MODEL_HASH}); re-downloading ...")
  endif()

  message(STATUS "Downloading detection model ${FILE_NAME} ...")
  file(
    DOWNLOAD "${URL}" "${MODEL_FILE}.tmp"
    INACTIVITY_TIMEOUT 30
    STATUS MODEL_DOWNLOAD_STATUS)
  list(GET MODEL_DOWNLOAD_STATUS 0 MODEL_DOWNLOAD_ERROR)
  if(MODEL_DOWNLOAD_ERROR EQUAL 0)
    file(SHA256 "${MODEL_FILE}.tmp" MODEL_HASH)
    if(MODEL_HASH STREQUAL SHA256)
      file(RENAME "${MODEL_FILE}.tmp" "${MODEL_FILE}")
    else()
      file(REMOVE "${MODEL_FILE}.tmp")
      message(WARNING "Detection model ${FILE_NAME} download hash mismatch "
                      "(got ${MODEL_HASH}); the next build will retry.")
    endif()
  else()
    file(REMOVE "${MODEL_FILE}.tmp")
    list(GET MODEL_DOWNLOAD_STATUS 1 MODEL_DOWNLOAD_MESSAGE)
    message(WARNING "Detection model ${FILE_NAME} download failed "
                    "(${MODEL_DOWNLOAD_MESSAGE}); the next build will retry.")
  endif()
endfunction()

if(GET_AVT341_DETECTION_MODELS)
  # YOLOv8n TorchScript CPU 640x640
  avt_341_get_detection_model_file(
    yolov8n-cpu-640x640.torchscript
    "https://www.dropbox.com/scl/fi/9q395le8f3xu186tinpcl/yolov8n-cpu-640x640.torchscript?rlkey=93aok5puu4kxx7r0b4uc5r782&st=nsiph3ln&dl=1"
    50751423d8d350559056bfe70272f009a5474d45bc15c767f49316d868aeee72)
  avt_341_get_detection_model_file(
    yolov8n-cpu-640x640.names
    "https://www.dropbox.com/scl/fi/7z1bpjty9iwlin5ky4fdz/yolov8n-cpu-640x640.names?rlkey=4u6vtcfr1vvutnrj25tmkxpk8&st=y2sju2xy&dl=1"
    634a1132eb33f8091d60f2c346ababe8b905ae08387037aed883953b7329af84)

  # FED Alpha TorchScript YOLOv8m GPU 640x640
  avt_341_get_detection_model_file(
    fedav8m-cuda-640x640.torchscript
    "https://www.dropbox.com/scl/fi/8k6sari7lmhqh1tbznmjd/fedav8m-cuda-640x640.torchscript?rlkey=mrpjx2wbpsb0en4w09cht3dqt&st=3axcy84m&dl=1"
    634b735eb0e91bf88df8bdca8e93b1310783a3c6d96f276a39a8dc9f78659600)
  avt_341_get_detection_model_file(
    fedav8m-cuda-640x640.names
    "https://www.dropbox.com/scl/fi/1r6q2q34ianpgpmjw1l0t/fedav8m-cuda-640x640.names?rlkey=ykmfd5iexasghuuw41ecwo6k3&st=1qvojmk4&dl=1"
    9d9d7c04edb688ffb3bad0b6307945e7f86aa314c161b52dc58bb04699130f9b)

  # MRZR TorchScript YOLOv8m GPU 640x640
  avt_341_get_detection_model_file(
    mrzrv8m-cuda-640x640.torchscript
    "https://www.dropbox.com/scl/fi/pfvht4borsie7oww93s3a/mrzrv8m-cuda-640x640.torchscript?rlkey=uhu81fg92g0r02u020cv6trcl&st=il7ex9l2&dl=1"
    24ffb7663409865054450e5152de13e5411b210e5a7c421154b86ad34d00584b)
  avt_341_get_detection_model_file(
    mrzrv8m-cuda-640x640.names
    "https://www.dropbox.com/scl/fi/icljk1oidyyeo3oxcjcty/mrzrv8m-cuda-640x640.names?rlkey=ghs972f4hrpo2q6ch887zovq7&st=iuxv1xmn&dl=1"
    f6ed0f280f9bc2166e5f5ab30de164516ac6f6bf6f440a0a47df302c7dfd07aa)
endif() # if(GET_AVT341_DETECTION_MODELS)

# Install the custom models
message(STATUS "Installing custom models ...")
install(
  DIRECTORY "${PROJECT_SOURCE_DIR}/ml_weights/detection/models/"
  DESTINATION "share/${PROJECT_NAME}/ml_weights/detection/models/"
  FILES_MATCHING
  PATTERN "*.torchscript"
  PATTERN "*.names")

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
  object_detector_node
  "object_detection_node.cpp" "object_detection_node_executor.cpp"
  "object_detector.cpp")

ament_target_dependencies(object_detector_node cv_bridge
                          OpenCV rclcpp sensor_msgs vision_msgs image_transport)

target_include_directories(
        object_detector_node
        PUBLIC $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>)

target_link_libraries(object_detector_node
                      object_detector_params_service
                      ${PROJECT_NAME}_detection_library ${TORCH_LIBRARIES}
                      ament_index_cpp::ament_index_cpp)

# libtorch vendors its own fmt headers in its include directory, which shadow
# the system fmt that fmt::fmt links against. Build fmt header-only here so the
# generated params service resolves against the headers actually included.
target_compile_definitions(object_detector_node
                           PRIVATE FMT_HEADER_ONLY)

install(
  TARGETS object_detector_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
