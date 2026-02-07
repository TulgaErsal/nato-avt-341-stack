message(STATUS "Building mission command panel plugin...")

if(USING_CATKIN)
    find_package(catkin REQUIRED COMPONENTS
        rviz
    )

    ## This setting causes Qt's "MOC" generation to happen automatically.
    set(CMAKE_AUTOMOC ON)

    if(rviz_QT_VERSION VERSION_LESS "5")
        message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
        find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
        ## pull in all required include dirs, define QT_LIBRARIES, etc.
        include(${QT_USE_FILE})
    else()
        message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
        find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
        ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
        set(QT_LIBRARIES Qt5::Widgets)
    endif()

    # Avoid conflicts between Qt and boost signals
    add_definitions(-DQT_NO_KEYWORDS)

    set(SRC_FILES mission_command_panel.cpp)
    set(HEADER_FILES  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/ui/mission_command_panel.h)
    add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
    target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

endif() # if(USING_CATKIN)

if(USING_AMENT)
    message(STATUS "Mission command panel only supported in ROS1, skipping...")
endif() # if(USING_AMENT)
