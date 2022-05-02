cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
)

add_message_files (
  FILES
  wheels_rpm_msg.msg
)

add_service_files(
  FILES
  reset.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg 
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(computevelocity src/computevelocity.cpp)
add_dependencies(computevelocity ${PROJECT_NAME}_gencfg)
add_dependencies(computevelocity ${catkin_EXPORTED_TARGETS})
target_link_libraries(computevelocity ${catkin_LIBRARIES})