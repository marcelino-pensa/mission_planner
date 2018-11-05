cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  mavros_msgs
  mg_msgs
  px4_control
  mav_trajectory_generation_ros
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(inspector_node 
               src/inspection_class.cpp
               src/mission_node.cpp
               src/mission_class.cpp
               src/helper.cpp
               src/threads.cpp
               src/callbacks.cpp)
target_link_libraries(inspector_node ${catkin_LIBRARIES})

#This makes sure that necessary messages and services are compiled before this package
add_dependencies(inspector_node mavros_msgs_generate_messages_cpp mg_msgs_generate_messages_cpp
                                mav_trajectory_generation_ros_gencpp ${catkin_EXPORTED_TARGETS})
# add_dependencies(inspector_node mg_msgs_generate_messages_cpp)
# add_dependencies(inspector_node mav_trajectory_generation_ros_gencpp)
# add_dependencies(inspector_node ${catkin_EXPORTED_TARGETS})
# add_dependencies(inspector_node px4_control_generate_messages_cpp)

