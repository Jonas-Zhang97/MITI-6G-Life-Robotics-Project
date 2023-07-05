cmake_minimum_required(VERSION 3.0.2)
project(project_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Threats.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pkg
   CATKIN_DEPENDS 
    geometry_msgs 
    roscpp std_msgs 
    message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)