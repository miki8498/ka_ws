cmake_minimum_required(VERSION 3.0.2)
project(uc1_c_spline)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  message_generation
  actionlib
  actionlib_msgs
  tf
  kdl_parser
)
  
find_package(orocos_kdl REQUIRED)
find_package( Eigen3 REQUIRED )

  
# add_message_files(
#   FILES
#   JointControl.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  interpolate.srv
  joint_interpolate.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   UrPositionControl.action
# )

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
  LIBRARIES uc1_c_spline
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs std_srvs
  DEPENDS 
)

include_directories(
  include
  include/uc1_c_spline
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/FIRFilter.cpp
  src/${PROJECT_NAME}/Spline.cpp
)



target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(uc1_c_spline_main src/uc1_c_spline.cpp)
target_link_libraries(uc1_c_spline_main ${catkin_LIBRARIES} uc1_c_spline)
add_dependencies(uc1_c_spline_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

