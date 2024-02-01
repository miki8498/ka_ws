cmake_minimum_required(VERSION 3.0.2)
project(collision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
  gazebo_msgs
  message_runtime
  tf tf2 tf2_bullet tf2_ros tf2_geometry_msgs tf2_kdl tf2_msgs 
  actionlib_msgs
  message_generation
)

add_service_files(
  FILES
  CollisionServer.srv
  CollisionDistance.srv
)

add_message_files(
  FILES
  InfoData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    gazebo_msgs
    message_generation
    actionlib_msgs
    trajectory_msgs
    geometry_msgs
    message_runtime 
    std_srvs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(INTERACTIVITY_LIB_NAME interactivity_utils)

add_library(${INTERACTIVITY_LIB_NAME}
  src/interactive_robot.cpp
)

target_link_libraries(
  ${INTERACTIVITY_LIB_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    ${INTERACTIVITY_LIB_NAME}
  LIBRARY
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
  DESTINATION
    ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(collision src/CollisionChecker.cpp
                     src/ComputeCollision.cpp   include/ComputeCollision.h
                     src/MoveGroup.cpp          include/MoveGroup.h)

target_link_libraries(collision
    ${interactive_markers_LIBRARIES}
    interactivity_utils
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    fmt)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})