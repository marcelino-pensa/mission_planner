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
  mg_msgs
  px4_control
  cmake_modules
  p4_ros
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  DEPENDS Eigen
)
catkin_package(
  DEPENDS mg_msgs p4_ros
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

# Simulation inspection node
add_executable(inspector_node 
               src/nodes/mission_node.cpp
               src/inspection_class.cpp
               src/mission_class.cpp
               src/drone_threads.cpp
               src/helper.cpp
               src/callbacks.cpp
               src/visualization_functions.cpp)
target_link_libraries(inspector_node ${catkin_LIBRARIES})
add_dependencies(inspector_node mg_msgs_gencpp p4_ros_gencpp
                 ${catkin_EXPORTED_TARGETS} ${mg_msgs_EXPORTED_TARGETS}
                 ${p4_ros_EXPORTED_TARGETS})