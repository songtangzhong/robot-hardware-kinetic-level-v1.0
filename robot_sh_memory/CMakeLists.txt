cmake_minimum_required(VERSION 3.0.2)
project(robot_sh_memory)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  robot_info
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_sh_memory
  CATKIN_DEPENDS 
  message_runtime
  roscpp 
  rospy 
  std_msgs
  robot_info
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sem_common.cpp
  src/shm_common.cpp
)
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
