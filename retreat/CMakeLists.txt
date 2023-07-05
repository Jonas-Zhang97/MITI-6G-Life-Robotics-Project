cmake_minimum_required(VERSION 3.1.3)
project(retreat)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  # for msg
  geometry_msgs
  message_generation

  project_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    tf2_geometry_msgs
    # for msg
    message_runtime
    geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/application/${PROJECT_NAME}_node.cpp)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} project_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})