cmake_minimum_required(VERSION 3.0.2)
project(abot_piper_diagnostics)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  pluginlib
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    roscpp
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(piper_fk_workspace_mapper src/piper_fk_workspace_mapper.cpp)
add_dependencies(piper_fk_workspace_mapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(piper_fk_workspace_mapper ${catkin_LIBRARIES})

install(TARGETS piper_fk_workspace_mapper
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
