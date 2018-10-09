cmake_minimum_required(VERSION 2.8.3)
project(inspector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  mg_msgs
  px4_control
  roscpp
  std_msgs
) 

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES inspector
#  CATKIN_DEPENDS mavros_msgs mg_msgs rocpp std_msgs
#  DEPENDS system_lib
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
               src/inspector_node.cpp
               src/inspector_class.cpp
               src/helper.cpp)
target_link_libraries(inspector_node ${catkin_LIBRARIES})

#This makes sure that messages and services are compiled before the rest
add_dependencies(inspector_node mg_msgs_generate_messages_cpp)
add_dependencies(inspector_node px4_control_generate_messages_cpp)
add_dependencies(inspector_node ${catkin_EXPORTED_TARGETS})
add_dependencies(inspector_node mavros_msgs_generate_messages_cpp)

