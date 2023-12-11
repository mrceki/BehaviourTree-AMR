cmake_minimum_required(VERSION 3.0.2)
project(bt_amr)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  autodock_core
  behaviortree_cpp_v3
  move_base_msgs
  roscpp
  dolly_pose_estimation
  std_msgs
)

add_action_files(
  FILES
  Tree.action
)

generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs  # Add actionlib_msgs as a dependency
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES bt_amr
   CATKIN_DEPENDS std_msgs actionlib actionlib_msgs behaviortree_cpp_v3 roscpp dolly_pose_estimation autodock_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/bt_amr.cpp
  src/docking.cpp
  src/dolly_client.cpp
  src/battery_check.cpp
  src/is_battery_charged.cpp
  src/move_base_client.cpp
  src/stop_dolly_client.cpp
  src/print_dolly.cpp
)

add_executable(bt_action_server src/bt_action_server.cpp)
add_dependencies(bt_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(bt_action_server
  ${catkin_LIBRARIES}
  bt_amr
)

install(TARGETS ${PROJECT_NAME} bt_action_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)