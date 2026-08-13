message(STATUS "Building UAB SAE segmentation python node ...")

# Generated python parameter module uab_perception_py_parameters.py, installed
# into the avt_341_nav python package.
avt_341_generate_python_parameters(${PROJECT_SOURCE_DIR}/parameters/uab_perception_py.yaml)

install(
  PROGRAMS ${PROJECT_SOURCE_DIR}/avt_341_nav/uab_segmentation/sae_net_node.py
  DESTINATION "lib/${PROJECT_NAME}")

# Download the SAE segmentation network weights when the file is missing or
# does not match the latest expected hash. Should not crash if download cannot be completed (ex: no internet)
set(SAE_WEIGHTS_FILE
    "${PROJECT_SOURCE_DIR}/ml_weights/uab_segmentation/sae_net_5class_RUGD_GOD_544_688.pt")
set(SAE_WEIGHTS_SHA256
    "0fe30b66b287bbbe65fc9ede19f3037f378752cb29070754c8dfda8f919079a2")
set(SAE_WEIGHTS_URL
    "https://www.dropbox.com/scl/fi/xxs1fpjf7lglj9zp5kh4q/sae_net_5class_RUGD_GOD_544_688.pt?rlkey=maxpvar3kctukndosovgxzses&st=blng12z4&dl=1")

set(SAE_WEIGHTS_OK FALSE)
if(EXISTS "${SAE_WEIGHTS_FILE}")
  file(SHA256 "${SAE_WEIGHTS_FILE}" SAE_WEIGHTS_HASH)
  if(SAE_WEIGHTS_HASH STREQUAL SAE_WEIGHTS_SHA256)
    set(SAE_WEIGHTS_OK TRUE)
  else()
    message(STATUS "SAE segmentation network weights hash mismatch "
                   "(got ${SAE_WEIGHTS_HASH}); re-downloading ...")
  endif()
endif()

if(NOT SAE_WEIGHTS_OK)
  message(STATUS "Downloading SAE segmentation network weights ...")
  file(
    DOWNLOAD "${SAE_WEIGHTS_URL}" "${SAE_WEIGHTS_FILE}.tmp"
    INACTIVITY_TIMEOUT 30
    STATUS SAE_DOWNLOAD_STATUS)
  list(GET SAE_DOWNLOAD_STATUS 0 SAE_DOWNLOAD_ERROR)
  if(SAE_DOWNLOAD_ERROR EQUAL 0)
    file(SHA256 "${SAE_WEIGHTS_FILE}.tmp" SAE_WEIGHTS_HASH)
    if(SAE_WEIGHTS_HASH STREQUAL SAE_WEIGHTS_SHA256)
      file(RENAME "${SAE_WEIGHTS_FILE}.tmp" "${SAE_WEIGHTS_FILE}")
    else()
      file(REMOVE "${SAE_WEIGHTS_FILE}.tmp")
      message(WARNING "SAE segmentation network weights download hash mismatch "
                      "(got ${SAE_WEIGHTS_HASH}); the next build will retry.")
    endif()
  else()
    file(REMOVE "${SAE_WEIGHTS_FILE}.tmp")
    list(GET SAE_DOWNLOAD_STATUS 1 SAE_DOWNLOAD_MESSAGE)
    message(WARNING "SAE segmentation network weights download failed "
                    "(${SAE_DOWNLOAD_MESSAGE}); the next build will retry.")
  endif()
endif()

install(
  DIRECTORY "${PROJECT_SOURCE_DIR}/ml_weights/uab_segmentation/"
  DESTINATION "share/${PROJECT_NAME}/ml_weights/uab_segmentation"
  FILES_MATCHING
  PATTERN "*.pt"
  PATTERN "*.yaml")
