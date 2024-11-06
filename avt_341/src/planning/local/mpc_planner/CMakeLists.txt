find_package(Julia 1.5.4 EXACT)
if(NOT ${Julia_FOUND})
  message(WARNING "Julia libraries not found. The MPC wrapper node will not be built.")
  set(BUILD_MPC OFF)
else()
  message(STATUS "Julia libraries found. Adding MPC wrapper node to the build.")
  set(BUILD_MPC ON)
endif() # if(${Julia_FOUND})

if(BUILD_MPC)

  if(USING_CATKIN)
    add_executable(${PROJECT_NAME}_mpc_planner_node
      "avt_341_mpc_planner_node.cpp")
    add_dependencies(${PROJECT_NAME}_mpc_planner_node ${catkin_EXPORTED_TARGETS})
    target_include_directories(${PROJECT_NAME}_mpc_planner_node
      PUBLIC
        "$<BUILD_INTERFACE:${Julia_INCLUDE_DIRS}>")
    target_link_libraries(${PROJECT_NAME}_mpc_planner_node
        ${PROJECT_NAME}_proxy
        $<BUILD_INTERFACE:${Julia_LIBRARY}>)
    target_compile_definitions(${PROJECT_NAME}_mpc_planner_node
      PUBLIC
        "MPC_PLANNER_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/mpc_commands.jl\""
        "MPC_PARAMETERS_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/mpc_parameters.jl\""
        "MPC_MODELS_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/mpc_models.jl\""
        "MPC_SYSIMAGE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/sysimage.so\""
        -DJULIA_ENABLE_THREADING)
    install(
      TARGETS ${PROJECT_NAME}_mpc_planner_node
      RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
      LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
    # Try to generate a Julia sysimage (if not already present).
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sysimage.so")
      message(STATUS "Generating Julia sysimage to improve MPC node startup time - this may take a while...")

      # Set the input template and output file absolute paths as environment
      # variables so the Julia REPL can access them.
      set(ENV{JULIA_SYSIMAGE_OUTPUT_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/sysimage.so")
      set(ENV{JULIA_SYSIMAGE_TEMPLATE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/mpc_sysimage_template.jl")

      # Set the path to the Julia sysimage generator script.
      set(JULIA_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/mpc_sysimage_generator.jl")

      # Run the Julia REPL with the sysimage generator script.
      execute_process(COMMAND ${Julia_EXECUTABLE} -L ${JULIA_SCRIPT})
    endif() # if(NOT EXISTS "(...)/sysimage.so")
  endif() # if(USING_CATKIN)

  if(USING_AMENT)
    add_executable(${PROJECT_NAME}_mpc_planner_node
      "avt_341_mpc_planner_node.cpp")
    ament_target_dependencies(${PROJECT_NAME}_mpc_planner_node
      ${dependencies})
    target_include_directories(${PROJECT_NAME}_mpc_planner_node
      PUBLIC
        "$<BUILD_INTERFACE:${Julia_INCLUDE_DIRS}>")
    target_link_libraries(${PROJECT_NAME}_mpc_planner_node
        ${PROJECT_NAME}_proxy
        $<BUILD_INTERFACE:${Julia_LIBRARY}>)
    target_compile_definitions(${PROJECT_NAME}_mpc_planner_node
      PUBLIC
        "MPC_PLANNER_MODULE_PATH=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/mpc_commands.jl\""
        "MPC_PARAMETERS_MODULE_PATH=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/mpc_parameters.jl\""
        "MPC_MODELS_MODULE_PATH=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/mpc_models.jl\""
        "MPC_SYSIMAGE_PATH=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/sysimage.so\""
        -DJULIA_ENABLE_THREADING)
    install(TARGETS ${PROJECT_NAME}_mpc_planner_node
            EXPORT export_${PROJECT_NAME}
            DESTINATION lib/${PROJECT_NAME})
    # Copy the Julia modules to the install folder.
    install(FILES
              "mpc_models.jl"
              "mpc_commands.jl"
              "mpc_parameters.jl"
            DESTINATION share/${PROJECT_NAME})
    # Try to generate a Julia sysimage (if not already present).
    if(NOT EXISTS "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/sysimage.so")
      message(STATUS "Generating Julia sysimage to improve MPC node startup time - this may take a while...")

      # Set the input template and output file absolute paths as environment
      # variables so the Julia REPL can access them.
      set(ENV{JULIA_SYSIMAGE_OUTPUT_PATH} "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/sysimage.so")
      set(ENV{JULIA_SYSIMAGE_TEMPLATE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/mpc_sysimage_template.jl")

      # Set the path to the Julia sysimage generator script.
      set(JULIA_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/mpc_sysimage_generator.jl")

      # Run the Julia REPL with the sysimage generator script.
      execute_process(COMMAND ${Julia_EXECUTABLE} -L ${JULIA_SCRIPT})
    endif() # if(NOT EXISTS "(...)/sysimage.so")
  endif() # if(USING_AMENT)
endif() # if(BUILD_MPC)