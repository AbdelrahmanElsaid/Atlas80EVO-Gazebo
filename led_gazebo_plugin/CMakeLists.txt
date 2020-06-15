cmake_minimum_required(VERSION 3.0.2)
project(led_gazebo_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED COMPONENTS)
find_package(ignition-transport4 REQUIRED COMPONENTS)
find_package(ignition-msgs1 REQUIRED COMPONENTS)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime gazebo_ros roscpp#std_srvs
#  LIBRARIES led_gazebo_plugin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Create the libRosFlashLightPlugin.so library.
set(ros_flashlight_plugin_name RosFlashLightPlugin)
add_library(${ros_flashlight_plugin_name} src/RosFlashLightPlugin.cc)
target_link_libraries(${ros_flashlight_plugin_name}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  LedPlugin
)
install(TARGETS ${ros_flashlight_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libLightVisualPlugin.so library.
set(light_visual_plugin_name LightVisualPlugin)
add_library(${light_visual_plugin_name} src/LightVisualPlugin.cc)
target_link_libraries(${light_visual_plugin_name}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${light_visual_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

