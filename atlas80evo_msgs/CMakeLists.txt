cmake_minimum_required(VERSION 2.8.3)
project(atlas80evo_msgs)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  visualization_msgs
)



## Generate messages in the 'msg' folder
add_message_files(
   FILES
   FSMState.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SetFSMState.srv
   SetFileLocation.srv
   SetLifter.srv
   SetFindTable.srv
   SetFindCharger.srv
   SetLeaveTable.srv
   SetFileName.srv
   SetId.srv
   SetONOFF.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
   visualization_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES atlas80evo_msgs
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  sensor_msgs
  std_msgs
  visualization_msgs
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

