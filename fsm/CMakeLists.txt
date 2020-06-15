cmake_minimum_required(VERSION 2.8.3)
project(fsm)


find_package(catkin REQUIRED COMPONENTS
  atlas80evo_msgs
  rospy
  std_msgs
)


# catkin_python_setup()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

