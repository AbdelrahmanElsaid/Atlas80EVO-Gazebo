cmake_minimum_required(VERSION 2.8.3)
project(gyro_odometry)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gyro_odometry
#  CATKIN_DEPENDS geometry_msgs message_filters nav_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xsens_gyro_odometry src/xsens_gyro_odometry.cpp)
target_link_libraries(xsens_gyro_odometry ${catkin_LIBRARIES})
add_dependencies(xsens_gyro_odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

