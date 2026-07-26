# Copyright 2022 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# avt_341_generate_cpp_parameter_file(<base_name> <yaml_file> [<validate_header>] [INCLUDE_SUBFOLDER <sub/folders>])
#
# Generates <base_name>_params_dto.hpp and <base_name>_params_service.hpp from
# the given template yaml. The corresponding INTERFACE targets have the same
# names without the .hpp extension.
macro(avt_341_generate_cpp_parameter_file BASE_NAME YAML_FILE)
  cmake_parse_arguments(avt341_gp "" "INCLUDE_SUBFOLDER" "" ${ARGN})

  # Sub-path of the generated header below the include root
  set(LIB_INCLUDE_SUBDIR ${PROJECT_NAME})
  if(avt341_gp_INCLUDE_SUBFOLDER)
    string(REGEX REPLACE "^/+|/+$" "" avt341_gp_INCLUDE_SUBFOLDER "${avt341_gp_INCLUDE_SUBFOLDER}")
    if(IS_ABSOLUTE "${avt341_gp_INCLUDE_SUBFOLDER}")
      message(FATAL_ERROR
        "avt_341_generate_cpp_parameter_file: INCLUDE_SUBFOLDER must be a relative path, got '${avt341_gp_INCLUDE_SUBFOLDER}'")
    endif()
    string(APPEND LIB_INCLUDE_SUBDIR "/${avt341_gp_INCLUDE_SUBFOLDER}")
  endif()

  # Make the include directory
  set(LIB_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/${LIB_INCLUDE_SUBDIR})
  file(MAKE_DIRECTORY ${LIB_INCLUDE_DIR})

  # Optional positional parameter for the user defined validation header
  set(VALIDATE_HEADER "")
  set(VALIDATE_HEADER_FILENAME "")
  if(avt341_gp_UNPARSED_ARGUMENTS)
    list(GET avt341_gp_UNPARSED_ARGUMENTS 0 avt341_gp_validate_header_arg)
    cmake_path(SET IN_VALIDATE_HEADER ${CMAKE_CURRENT_SOURCE_DIR})
    cmake_path(APPEND IN_VALIDATE_HEADER ${avt341_gp_validate_header_arg})

    cmake_path(GET IN_VALIDATE_HEADER FILENAME VALIDATE_HEADER_FILENAME)
    cmake_path(SET VALIDATE_HEADER ${LIB_INCLUDE_DIR})
    cmake_path(APPEND VALIDATE_HEADER ${VALIDATE_HEADER_FILENAME})

    # Copy the header file into the include directory
    file(COPY ${IN_VALIDATE_HEADER} DESTINATION ${LIB_INCLUDE_DIR})
    # necessary so that #include <param_file.hpp> can be used in the local package (deprecated)
    file(COPY ${IN_VALIDATE_HEADER} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include)
  endif()

  # Resolve the yaml file relative to the current source dir (absolute paths are kept)
  set(avt341_gp_yaml_file ${YAML_FILE})
  cmake_path(ABSOLUTE_PATH avt341_gp_yaml_file BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_VARIABLE YAML_FILE_PATH)

  set(DTO_LIB_NAME "${BASE_NAME}_params_dto")
  set(SERVICE_LIB_NAME "${BASE_NAME}_params_service")
  set(DTO_HEADER_FILE ${LIB_INCLUDE_DIR}/${DTO_LIB_NAME}.hpp)
  set(SERVICE_HEADER_FILE ${LIB_INCLUDE_DIR}/${SERVICE_LIB_NAME}.hpp)

  # Templates may pull in mixin files (__include_mixins) from a sibling mixins/ folder; depend
  # on all of them (conservative) so mixin edits regenerate the headers.
  cmake_path(GET YAML_FILE_PATH PARENT_PATH avt341_gp_yaml_dir)
  file(GLOB avt341_gp_mixin_files CONFIGURE_DEPENDS
    "${avt341_gp_yaml_dir}/mixins/*.yaml" "${avt341_gp_yaml_dir}/mixins/*.yml")

  # Parse the yaml once and generate both headers.
  set(PARAM_GENERATION_STAMP
    ${CMAKE_CURRENT_BINARY_DIR}/${BASE_NAME}_params_generation.stamp)
  add_custom_command(
    OUTPUT ${PARAM_GENERATION_STAMP}
    BYPRODUCTS ${DTO_HEADER_FILE} ${SERVICE_HEADER_FILE}
    COMMAND ${Python3_EXECUTABLE} -m avt_341_param_lib.generate_cpp_header
      ${DTO_HEADER_FILE} ${SERVICE_HEADER_FILE} ${YAML_FILE_PATH}
      ${VALIDATE_HEADER_FILENAME}
      --mixin-include-prefix ${LIB_INCLUDE_SUBDIR}
    COMMAND ${CMAKE_COMMAND} -E touch ${PARAM_GENERATION_STAMP}
    DEPENDS ${YAML_FILE_PATH} ${avt341_gp_mixin_files} ${VALIDATE_HEADER}
    COMMENT "Generating ${DTO_LIB_NAME}.hpp and ${SERVICE_LIB_NAME}.hpp"
    VERBATIM
  )
  add_custom_target(${BASE_NAME}_params_generation ALL
    DEPENDS ${PARAM_GENERATION_STAMP})

  # Preserve the build-local short include path for both new headers.
  set(LOCAL_DTO_HEADER_FILE ${CMAKE_CURRENT_BINARY_DIR}/include/${DTO_LIB_NAME}.hpp)
  set(LOCAL_SERVICE_HEADER_FILE
    ${CMAKE_CURRENT_BINARY_DIR}/include/${SERVICE_LIB_NAME}.hpp)
  foreach(avt341_gp_header_kind IN ITEMS DTO SERVICE)
    set(avt341_gp_lib_name ${${avt341_gp_header_kind}_LIB_NAME})
    set(avt341_gp_header_file ${${avt341_gp_header_kind}_HEADER_FILE})
    set(avt341_gp_local_header_file
      ${LOCAL_${avt341_gp_header_kind}_HEADER_FILE})
    set(avt341_gp_pragma_file
      ${CMAKE_CURRENT_BINARY_DIR}/${avt341_gp_lib_name}_pragma_warning)
    file(WRITE ${avt341_gp_pragma_file}
      "#pragma message(\"#include \\\"${avt341_gp_lib_name}.hpp\\\" is deprecated. Use #include <${LIB_INCLUDE_SUBDIR}/${avt341_gp_lib_name}.hpp> instead.\")\n")
    add_custom_command(
      OUTPUT ${avt341_gp_local_header_file}
      COMMAND ${CMAKE_COMMAND} -E cat ${avt341_gp_pragma_file}
        ${avt341_gp_header_file} > ${avt341_gp_local_header_file}
      DEPENDS ${avt341_gp_header_file}
      COMMENT "Creating deprecated header file ${avt341_gp_local_header_file}"
      VERBATIM
    )
  endforeach()

  # The DTO target supplies only generated standard-library data types.
  add_library(${DTO_LIB_NAME} INTERFACE
    ${DTO_HEADER_FILE}
    ${LOCAL_DTO_HEADER_FILE}
  )
  target_include_directories(${DTO_LIB_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(${DTO_LIB_NAME} INTERFACE cxx_std_17)
  add_dependencies(${DTO_LIB_NAME} ${BASE_NAME}_params_generation)
  # Shared mixin DTO headers referenced by the generated dto header
  # (conservative: link all mixin DTO libraries; they are header-only)
  if(AVT341_MIXIN_DTO_LIBS)
    target_link_libraries(${DTO_LIB_NAME} INTERFACE ${AVT341_MIXIN_DTO_LIBS})
  endif()

  # The service target owns all ROS, validation, and formatting dependencies.
  add_library(${SERVICE_LIB_NAME} INTERFACE
    ${SERVICE_HEADER_FILE}
    ${VALIDATE_HEADER}
    ${LOCAL_SERVICE_HEADER_FILE}
  )
  target_include_directories(${SERVICE_LIB_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  add_dependencies(${SERVICE_LIB_NAME} ${BASE_NAME}_params_generation)
  target_link_libraries(${SERVICE_LIB_NAME} INTERFACE
    ${DTO_LIB_NAME}
    fmt::fmt
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    avt_341_param_lib::avt_341_param_lib
  )
  install(
    FILES ${DTO_HEADER_FILE} ${SERVICE_HEADER_FILE}
    DESTINATION include/${LIB_INCLUDE_SUBDIR}
  )
  if(VALIDATE_HEADER)
    install(
      FILES ${VALIDATE_HEADER}
      DESTINATION include/${LIB_INCLUDE_SUBDIR}
    )
  endif()
  ament_export_dependencies(
    fmt rclcpp rclcpp_lifecycle avt_341_param_lib
  )
endmacro()


# Internal driver shared by avt_341_generate_cpp_parameters and
# avt_341_generate_python_parameters: resolves <glob_pattern>, filters the
# matches to yaml parameter files (.yaml/.yml) and invokes <generator_command>
# once per matched file as
#   <generator_command>(<stem><suffix> <yaml_file> <remaining args...>)
# where <suffix> uses <default_name_suffix> unless overridden with NAME_SUFFIX.
# RECURSE switches to recursive matching. All remaining arguments are
# forwarded to <generator_command> untouched.
macro(_avt_341_generate_parameters_glob GENERATOR_COMMAND GLOB_PATTERN DEFAULT_NAME_SUFFIX)
  cmake_parse_arguments(avt341_gpm "RECURSE" "NAME_SUFFIX" "" ${ARGN})

  if(NOT DEFINED avt341_gpm_NAME_SUFFIX)
    set(avt341_gpm_NAME_SUFFIX "${DEFAULT_NAME_SUFFIX}")
  endif()

  set(avt341_gpm_pattern ${GLOB_PATTERN})
  cmake_path(ABSOLUTE_PATH avt341_gpm_pattern BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  # Convenience: a plain directory means everything directly inside it
  if(IS_DIRECTORY ${avt341_gpm_pattern})
    string(APPEND avt341_gpm_pattern "/*")
  endif()

  if(avt341_gpm_RECURSE)
    file(GLOB_RECURSE avt341_gpm_matched_files CONFIGURE_DEPENDS ${avt341_gpm_pattern})
  else()
    file(GLOB avt341_gpm_matched_files LIST_DIRECTORIES false CONFIGURE_DEPENDS ${avt341_gpm_pattern})
  endif()

  # Keep only yaml parameter files from the matched set
  set(avt341_gpm_yaml_files "")
  foreach(avt341_gpm_matched_file IN LISTS avt341_gpm_matched_files)
    cmake_path(GET avt341_gpm_matched_file EXTENSION LAST_ONLY avt341_gpm_extension)
    if(avt341_gpm_extension STREQUAL ".yaml" OR avt341_gpm_extension STREQUAL ".yml")
      list(APPEND avt341_gpm_yaml_files ${avt341_gpm_matched_file})
    endif()
  endforeach()

  if(NOT avt341_gpm_yaml_files)
    message(WARNING
      "${GENERATOR_COMMAND}: no yaml parameter files matched '${avt341_gpm_pattern}'")
  endif()

  foreach(avt341_gpm_yaml_file IN LISTS avt341_gpm_yaml_files)
    cmake_path(GET avt341_gpm_yaml_file STEM avt341_gpm_lib_name)
    string(APPEND avt341_gpm_lib_name "${avt341_gpm_NAME_SUFFIX}")
    cmake_language(CALL ${GENERATOR_COMMAND}
      ${avt341_gpm_lib_name} ${avt341_gpm_yaml_file} ${avt341_gpm_UNPARSED_ARGUMENTS})
  endforeach()
endmacro()


# avt_341_generate_cpp_mixin_file(<base_name> <yaml_file>)
#
# Generates the shared type-only DTO header and INTERFACE library for one
# mixin template. Including templates reference the mixin's structs through
# these headers instead of re-defining them inline.
macro(avt_341_generate_cpp_mixin_file BASE_NAME YAML_FILE)
  set(LIB_INCLUDE_SUBDIR ${PROJECT_NAME})
  set(LIB_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/${LIB_INCLUDE_SUBDIR})
  file(MAKE_DIRECTORY ${LIB_INCLUDE_DIR})

  set(avt341_gp_yaml_file ${YAML_FILE})
  cmake_path(ABSOLUTE_PATH avt341_gp_yaml_file BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_VARIABLE YAML_FILE_PATH)

  set(DTO_LIB_NAME "${BASE_NAME}_params_dto")
  set(DTO_HEADER_FILE ${LIB_INCLUDE_DIR}/${DTO_LIB_NAME}.hpp)

  add_custom_command(
    OUTPUT ${DTO_HEADER_FILE}
    COMMAND ${Python3_EXECUTABLE} -m avt_341_param_lib.generate_cpp_mixin_header
      ${DTO_HEADER_FILE} ${YAML_FILE_PATH}
    DEPENDS ${YAML_FILE_PATH}
    COMMENT "Generating mixin DTO ${DTO_LIB_NAME}.hpp"
    VERBATIM
  )
  add_custom_target(${BASE_NAME}_params_generation ALL
    DEPENDS ${DTO_HEADER_FILE})

  add_library(${DTO_LIB_NAME} INTERFACE ${DTO_HEADER_FILE})
  target_include_directories(${DTO_LIB_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(${DTO_LIB_NAME} INTERFACE cxx_std_17)
  add_dependencies(${DTO_LIB_NAME} ${BASE_NAME}_params_generation)
  install(
    FILES ${DTO_HEADER_FILE}
    DESTINATION include/${LIB_INCLUDE_SUBDIR}
  )
endmacro()


# avt_341_generate_cpp_parameters(<glob_pattern> [<validate_header>] [RECURSE]
#                                 [INCLUDE_SUBFOLDER <sub/folders>] [NAME_SUFFIX <suffix>])
#
# Generates one C++ parameter library per yaml file matched by <glob_pattern>
# (relative to the current source dir, or absolute; e.g. "config/*" or
# "src/params/*.yaml"). Matches are filtered to yaml parameter files
# (.yaml/.yml); a bare directory is treated as <directory>/*. RECURSE matches
# the pattern recursively (file(GLOB_RECURSE) semantics). Each matched file
# <name>.yaml produces the INTERFACE library target and header <name><suffix>,
# where <suffix> defaults to empty and can be overridden with NAME_SUFFIX.
# The fixed output suffixes are "_params_dto" and "_params_service".
#
# Mixin templates in a "mixins" subfolder of the pattern directory each
# generate a shared type-only DTO header (see avt_341_generate_cpp_mixin_file)
# before the node templates are processed; every node template DTO library
# links the mixin DTO libraries.
macro(avt_341_generate_cpp_parameters GLOB_PATTERN)
  # Generate shared mixin DTO fragments first so the template DTO libraries
  # can link them.
  set(avt341_gpm_mixin_dir ${GLOB_PATTERN})
  cmake_path(ABSOLUTE_PATH avt341_gpm_mixin_dir BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  if(NOT IS_DIRECTORY ${avt341_gpm_mixin_dir})
    cmake_path(GET avt341_gpm_mixin_dir PARENT_PATH avt341_gpm_mixin_dir)
  endif()
  set(AVT341_MIXIN_DTO_LIBS "")
  file(GLOB avt341_gpm_mixin_yamls CONFIGURE_DEPENDS
    "${avt341_gpm_mixin_dir}/mixins/*.yaml" "${avt341_gpm_mixin_dir}/mixins/*.yml")
  foreach(avt341_gpm_mixin_yaml IN LISTS avt341_gpm_mixin_yamls)
    cmake_path(GET avt341_gpm_mixin_yaml STEM avt341_gpm_mixin_stem)
    avt_341_generate_cpp_mixin_file(${avt341_gpm_mixin_stem} ${avt341_gpm_mixin_yaml})
    list(APPEND AVT341_MIXIN_DTO_LIBS ${avt341_gpm_mixin_stem}_params_dto)
  endforeach()

  _avt_341_generate_parameters_glob(
    avt_341_generate_cpp_parameter_file "${GLOB_PATTERN}" "" ${ARGN})
endmacro()


function(avt_341_generate_python_parameter_file LIB_NAME YAML_FILE)

  # Optional 3rd parameter for the user defined validation header
  if(${ARGC} EQUAL 3)
    set(VALIDATE_HEADER_FILENAME ${ARGV2})
  endif()

  # Resolve the yaml file relative to the current source dir (absolute paths are kept)
  cmake_path(ABSOLUTE_PATH YAML_FILE BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  # Create a build-local output directory for the generated Python module
  set(PY_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/generate_parameter_module_temp)
  file(MAKE_DIRECTORY ${PY_BUILD_DIR})

  find_package(ament_cmake_python)
  ament_get_python_install_dir(python_install_dir)

  # Generate into the build tree, not into CMAKE_INSTALL_PREFIX
  set(PARAM_HEADER_FILE ${PY_BUILD_DIR}/${LIB_NAME}.py)

  # Templates may pull in mixin files (__include_mixins) from a sibling mixins/ folder; depend
  # on all of them (conservative) so mixin edits regenerate the module.
  cmake_path(GET YAML_FILE PARENT_PATH avt341_gp_yaml_dir)
  file(GLOB avt341_gp_mixin_files CONFIGURE_DEPENDS
    "${avt341_gp_yaml_dir}/mixins/*.yaml" "${avt341_gp_yaml_dir}/mixins/*.yml")

  # Generate the module for Python
  add_custom_command(
          OUTPUT ${PARAM_HEADER_FILE}
          COMMAND ${Python3_EXECUTABLE} -m avt_341_param_lib.generate_python_module ${PARAM_HEADER_FILE} ${YAML_FILE} ${VALIDATE_HEADER_FILENAME}
          DEPENDS ${YAML_FILE} ${avt341_gp_mixin_files} ${VALIDATE_HEADER}
          COMMENT
          "Running `${Python3_EXECUTABLE} -m avt_341_param_lib.generate_python_module ${PARAM_HEADER_FILE} ${YAML_FILE} ${VALIDATE_HEADER_FILENAME}`"
          VERBATIM
  )

  # Create the library target
  add_library(${LIB_NAME} INTERFACE ${PARAM_HEADER_FILE} ${VALIDATE_HEADER})
  target_include_directories(${LIB_NAME} INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  # Install the generated module into the Python site-packages dir
  install(FILES ${PARAM_HEADER_FILE} DESTINATION ${CMAKE_INSTALL_PREFIX}/${python_install_dir}/${PROJECT_NAME})
endfunction()


# avt_341_generate_python_parameters(<glob_pattern> [<validate_module>] [RECURSE] [NAME_SUFFIX <suffix>])
#
# Python counterpart of avt_341_generate_cpp_parameters: generates one python
# parameter module per yaml file matched by <glob_pattern> (relative to the
# current source dir, or absolute). Matches are filtered to yaml parameter
# files (.yaml/.yml); a bare directory is treated as <directory>/*. RECURSE
# matches the pattern recursively. Each matched file <name>.yaml produces the
# module <name><suffix>.py, where <suffix> defaults to "_parameters" and can
# be overridden with NAME_SUFFIX. The remaining optional arguments are
# forwarded to avt_341_generate_python_parameter_file().
macro(avt_341_generate_python_parameters GLOB_PATTERN)
  _avt_341_generate_parameters_glob(
    avt_341_generate_python_parameter_file "${GLOB_PATTERN}" "_parameters" ${ARGN})
endmacro()

# create custom test function to pass yaml file into test main
function(add_rostest_with_parameters_gtest TARGET SOURCES YAML_FILE)
  add_executable(${TARGET} ${SOURCES})
  _ament_cmake_gtest_find_gtest()
  target_include_directories(${TARGET} PUBLIC "${GTEST_INCLUDE_DIRS}")
  target_link_libraries(${TARGET} ${GTEST_LIBRARIES})
  set(executable "$<TARGET_FILE:${TARGET}>")
  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}.gtest.xml")
  ament_add_test(
    ${TARGET}
    COMMAND ${executable} --ros-args --params-file ${YAML_FILE} --
    --gtest_output=xml:${result_file}
    OUTPUT_FILE ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}.txt
    RESULT_FILE ${result_file}
  )
endfunction()

function(add_rostest_with_parameters_gmock TARGET SOURCES YAML_FILE)
  add_executable(${TARGET} ${SOURCES})
  _ament_cmake_gmock_find_gmock()
  target_include_directories(${TARGET} PUBLIC "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(${TARGET} ${GMOCK_LIBRARIES})
  set(executable "$<TARGET_FILE:${TARGET}>")
  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}.gtest.xml")
  ament_add_test(
    ${TARGET}
    COMMAND ${executable} --ros-args --params-file ${YAML_FILE} --
    --gtest_output=xml:${result_file}
    OUTPUT_FILE ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}.txt
    RESULT_FILE ${result_file}
  )
endfunction()

