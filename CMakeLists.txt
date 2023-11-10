cmake_minimum_required(VERSION 3.8)
project(obj_tracker)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(phnx_msgs REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

# Add source for node executable (link non-ros dependencies here)
add_executable(obj_tracker src/ObjTrackerNode.cpp src/ObjTrackerNode_node.cpp src/Hungarian.cpp src/tracker.cpp src/mot.cpp)
target_include_directories(obj_tracker PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(obj_tracker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Make ros deps a variable so they get linked to tests as well
set(dependencies
        rclcpp
        geometry_msgs
        nav_msgs
        phnx_msgs
        OpenCV
)

# Link ros dependencies
ament_target_dependencies(
        obj_tracker
        ${dependencies}
)

install(TARGETS obj_tracker
        DESTINATION lib/${PROJECT_NAME})

# Uncomment below to make launch files available if created
#install(
#        DIRECTORY launch config
#        DESTINATION share/${PROJECT_NAME}/
#)

if (BUILD_TESTING)
    # Manually invoke clang format so it actually uses our file
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    find_package(ament_cmake_gtest REQUIRED)

    # Add unit tests
    ament_add_gtest(${PROJECT_NAME}-test
            tests/unit.cpp
            # Remember to add node source files
            src/ObjTrackerNode_node.cpp
            src/tracker.cpp
            src/Hungarian.cpp
            src/mot.cpp
    )
    ament_target_dependencies(${PROJECT_NAME}-test ${dependencies})
    target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
endif ()

ament_package()
