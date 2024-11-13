cmake_minimum_required(VERSION 2.8.3)
project(lazo_cerrado)

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  lazo_abierto
  robmovil_msgs
  cmake_modules
  angles
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robmovil_ekf
   CATKIN_DEPENDS roscpp tf2_ros lazo_abierto
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(closed_trajectory_follower src/kinematic_position_controller_node.cpp src/KinematicPositionController.cpp)
add_executable(logger src/logger_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(closed_trajectory_follower ${catkin_LIBRARIES})
target_link_libraries(logger ${catkin_LIBRARIES})
