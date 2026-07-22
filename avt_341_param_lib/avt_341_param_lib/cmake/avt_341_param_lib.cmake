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


# avt_341_generate_cpp_parameter_file(<lib_name> <yaml_file> [<validate_header>] [INCLUDE_SUBFOLDER <sub/folders>])
#
# Generates the parameter library header <lib_name>.hpp from the given template yaml
# file and exposes it through the INTERFACE library target <lib_name>. The header is
# placed under include/<project>/[<sub/folders>/]<lib_name>.hpp in both the build and
# install trees; the optional INCLUDE_SUBFOLDER keyword inserts additional sub-folders
# after the project name.
macro(avt_341_generate_cpp_parameter_file LIB_NAME YAML_FILE)
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

  # Set the output parameter header file name
  set(PARAM_HEADER_FILE ${LIB_INCLUDE_DIR}/${LIB_NAME}.hpp)

  # Generate the header for the library
  add_custom_command(
    OUTPUT ${PARAM_HEADER_FILE}
    COMMAND ${Python3_EXECUTABLE} -m avt_341_param_lib.generate_cpp_header ${PARAM_HEADER_FILE} ${YAML_FILE_PATH} ${VALIDATE_HEADER_FILENAME}
    DEPENDS ${YAML_FILE_PATH} ${VALIDATE_HEADER}
    COMMENT
    "Running `${Python3_EXECUTABLE} -m avt_341_param_lib.generate_cpp_header ${PARAM_HEADER_FILE} ${YAML_FILE_PATH} ${VALIDATE_HEADER_FILENAME}`"
    VERBATIM
  )
  # necessary so that #include <param_file.hpp> can be used in the local package (deprecated)
  set(LOCAL_PARAM_HEADER_FILE ${CMAKE_CURRENT_BINARY_DIR}/include/${LIB_NAME}.hpp)
  set(LOCAL_PARAM_HEADER_PRAGMA_WARNING_FILE ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}_pragma_warning)
  file(WRITE ${LOCAL_PARAM_HEADER_PRAGMA_WARNING_FILE}
    "#pragma message(\"#include \\\"${LIB_NAME}.hpp\\\" is deprecated. Use #include <${LIB_INCLUDE_SUBDIR}/${LIB_NAME}.hpp> instead.\")\n")
  add_custom_command(
    OUTPUT ${LOCAL_PARAM_HEADER_FILE}
    COMMAND ${CMAKE_COMMAND} -E cat ${LOCAL_PARAM_HEADER_PRAGMA_WARNING_FILE} ${PARAM_HEADER_FILE} > ${LOCAL_PARAM_HEADER_FILE}
    DEPENDS ${PARAM_HEADER_FILE}
    COMMENT
    "Creating deprecated header file ${LOCAL_PARAM_HEADER_FILE}"
    VERBATIM
  )

  # Create the library target
  add_library(${LIB_NAME} INTERFACE ${PARAM_HEADER_FILE} ${VALIDATE_HEADER} ${LOCAL_PARAM_HEADER_FILE})
  target_include_directories(${LIB_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(${LIB_NAME} INTERFACE
    fmt::fmt
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    avt_341_param_lib::avt_341_param_lib
  )
  install(DIRECTORY ${LIB_INCLUDE_DIR}/ DESTINATION include/${LIB_INCLUDE_SUBDIR})
  ament_export_dependencies(
    fmt rclcpp rclcpp_lifecycle avt_341_param_lib
  )
endmacro()


# Internal driver shared by avt_341_generate_cpp_parameters and
# avt_341_generate_python_parameters: resolves <glob_pattern>, filters the
# matches to yaml parameter files (.yaml/.yml) and invokes <generator_command>
# once per matched file as
#   <generator_command>(<stem><suffix> <yaml_file> <remaining args...>)
# where <suffix> defaults to "_parameters" (override with NAME_SUFFIX) and
# RECURSE switches to recursive matching. All remaining arguments are forwarded
# to <generator_command> untouched.
macro(_avt_341_generate_parameters_glob GENERATOR_COMMAND GLOB_PATTERN)
  cmake_parse_arguments(avt341_gpm "RECURSE" "NAME_SUFFIX" "" ${ARGN})

  if(NOT DEFINED avt341_gpm_NAME_SUFFIX)
    set(avt341_gpm_NAME_SUFFIX "_parameters")
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


# avt_341_generate_cpp_parameters(<glob_pattern> [<validate_header>] [RECURSE]
#                                 [INCLUDE_SUBFOLDER <sub/folders>] [NAME_SUFFIX <suffix>])
#
# Generates one C++ parameter library per yaml file matched by <glob_pattern>
# (relative to the current source dir, or absolute; e.g. "config/*" or
# "src/params/*.yaml"). Matches are filtered to yaml parameter files
# (.yaml/.yml); a bare directory is treated as <directory>/*. RECURSE matches
# the pattern recursively (file(GLOB_RECURSE) semantics). Each matched file
# <name>.yaml produces the INTERFACE library target and header <name><suffix>,
# where <suffix> defaults to "_parameters" and can be overridden with
# NAME_SUFFIX. The remaining optional arguments are forwarded to
# avt_341_generate_cpp_parameter_file().
macro(avt_341_generate_cpp_parameters GLOB_PATTERN)
  _avt_341_generate_parameters_glob(avt_341_generate_cpp_parameter_file "${GLOB_PATTERN}" ${ARGN})
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

  # Generate the module for Python
  add_custom_command(
          OUTPUT ${PARAM_HEADER_FILE}
          COMMAND ${Python3_EXECUTABLE} -m avt_341_param_lib.generate_python_module ${PARAM_HEADER_FILE} ${YAML_FILE} ${VALIDATE_HEADER_FILENAME}
          DEPENDS ${YAML_FILE} ${VALIDATE_HEADER}
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
  _avt_341_generate_parameters_glob(avt_341_generate_python_parameter_file "${GLOB_PATTERN}" ${ARGN})
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

