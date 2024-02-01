cmake_minimum_required(VERSION 2.8.3)
project(uc1_msgs)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED
  COMPONENTS
   actionlib_msgs
   message_generation
   geometry_msgs
   sensor_msgs
)

# add_message_files( FILES
#   hole.msg
#   holeArray.msg
# )



# add_service_files( FILES
#   newObjectName.srv
#   startNetwork.srv
#   getClusterTransformation.srv
#   getHoleDetection.srv
# )

# add_action_files(FILES
#   componentDetection.action 
#   addNewComponent.action
#   checkCollision.action
#   findComponent.action
#   objectDetector.action
#   PinToPin.action
#   Wait.action
#   RailScanning.action

#   graspDLO.action
# )



generate_messages(DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
   sensor_msgs
)


catkin_package(
	CATKIN_DEPENDS
		geometry_msgs
		message_generation
		message_runtime
)

