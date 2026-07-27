if(AVT341_DISABLE_MPC)
  message(STATUS "MPC build disabled.")
  return()
endif()

find_package(Julia 1.5.4 EXACT)
if(NOT "${Julia_FOUND}")
  message(WARNING "Julia 1.5.4 libraries not found. The MPC wrapper node will not be built.")
  set(BUILD_MPC OFF)
else()
  message(STATUS "Julia 1.5.4 libraries found. Adding MPC wrapper node to the build.")
  set(BUILD_MPC ON)
endif() # if(${Julia_FOUND})

if(BUILD_MPC)

  add_executable(mpc_planner_node
    "mpc_planner_node.cpp")
  ament_target_dependencies(mpc_planner_node
    ${dependencies})
  target_include_directories(mpc_planner_node
    PUBLIC
      "$<BUILD_INTERFACE:${Julia_INCLUDE_DIRS}>")
  target_link_libraries(mpc_planner_node
      ${PROJECT_NAME}_proxy
      $<BUILD_INTERFACE:${Julia_LIBRARY}>)
  target_compile_definitions(mpc_planner_node
    PUBLIC
      "MPC_PLANNER_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/mpc_commands.jl\""
      "MPC_PARAMETERS_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/mpc_parameters.jl\""
      "MPC_MODELS_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/mpc_models.jl\""
      "MPC_SYSIMAGE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/sysimage.so\""
      -DJULIA_ENABLE_THREADING)
  install(TARGETS mpc_planner_node
          EXPORT export_${PROJECT_NAME}
          DESTINATION lib/${PROJECT_NAME})
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
endif() # if(BUILD_MPC)
# Generated parameter library (avt_341_param_lib templates)
if(TARGET mpc_planner_node)
  target_link_libraries(
    mpc_planner_node mpc_local_planner_params_service)
endif()
