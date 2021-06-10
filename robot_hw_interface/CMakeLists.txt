cmake_minimum_required(VERSION 3.0.2)
project(robot_hw_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  controller_manager
  hardware_interface
  robot_info
  robot_sh_memory
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_hw_interface
  CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy 
  std_msgs 
  controller_manager 
  hardware_interface 
  robot_info
  robot_sh_memory
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/robot_hw_interface.cpp
)
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)

add_executable(robot_hw_control src/robot_hw_control.cpp)
add_dependencies(robot_hw_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(robot_hw_control
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
