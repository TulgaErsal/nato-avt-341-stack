message(STATUS "Building detection training node ...")

if(USING_CATKIN)
  message(
    WARNING
      "Skipping detection training node build - not yet supported in ROS 1.")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  # Download the base weights YOLOv8n
  message(STATUS "Downloading YOLOv8n model weights ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/6y5p8mowkkj187vkr8rk2/yolov8n.pt?rlkey=hp637nm1ltxvjp7dfl91jifq3&st=jci0vnwg&dl=1
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/yolo/yolov8n.pt"
    EXPECTED_HASH
      SHA256=31e20dde3def09e2cf938c7be6fe23d9150bbbe503982af13345706515f2ef95)

  # YOLOv8s
  message(STATUS "Downloading YOLOv8n model weights ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/9d90gm7609t1lwe6kcz70/yolov8s.pt?rlkey=0yv6kfabaowp7rzi8un5vq9aj&st=t8dems0q&dl=1
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/yolo/yolov8s.pt"
    EXPECTED_HASH
      SHA256=268e5bb54c640c96c3510224833bc2eeacab4135c6deb41502156e39986b562d)

  # YOLOv8m
  message(STATUS "Downloading YOLOv8m model weights ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/ru90hopfqmyfp3ixpumuz/yolov8m.pt?rlkey=3mrx68j2gegbcnbcs8ltrku6d&st=lrh79a2a&dl=1
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/yolo/yolov8m.pt"
    EXPECTED_HASH
      SHA256=6c25b0b63b1a433843f06d821a9ac1deb8d5805f74f0f38772c7308c5adc55a5)

  # YOLOv8l
  message(STATUS "Downloading YOLOv8l model weights ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/mz48k74q4uhdm20c3ykr7/yolov8l.pt?rlkey=kge4tmfyantomvlwjhd38ulnf&st=htijicbj&dl=1
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/yolo/yolov8l.pt"
    EXPECTED_HASH
      SHA256=18218ea4798da042d9862e6029ca9531adbd40ace19b6c9a75e2e28f1adf30cc)

  # YOLOv8x
  message(STATUS "Downloading YOLOv8x model weights ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/zywlo95lyzd6d7903qzqp/yolov8x.pt?rlkey=n4wu78kmn4z81nltukxtcrpft&st=0w1zwrqj&dl=1
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/yolo/yolov8x.pt"
    EXPECTED_HASH
      SHA256=c4d5a3f000d771762f03fc8b57ebd0aae324aeaefdd6e68492a9c4470f2d1e8b)

  # Download the TorchScript traced models YOLOv8n CPU 640x640
  message(STATUS "Downloading YOLOv8n TorchScript CPU model ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/scl/fi/9q395le8f3xu186tinpcl/yolov8n-cpu-640x640.torchscript?rlkey=93aok5puu4kxx7r0b4uc5r782&st=nsiph3ln&dl=1
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data/yolo/yolov8n-cpu-640x640.torchscript"
    EXPECTED_HASH
      SHA256=50751423d8d350559056bfe70272f009a5474d45bc15c767f49316d868aeee72)

  install(
    PROGRAMS gym_executor.py
    RENAME ${PROJECT_NAME}_gym_node
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
