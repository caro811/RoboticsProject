cmake_minimum_required(VERSION 2.8.3)
project(lazo_abierto)

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  robmovil_msgs
  cmake_modules
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
  LIBRARIES trajectory_controller
   CATKIN_DEPENDS roscpp
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
add_library(trajectory_controller src/TrajectoryFollower.cpp)

add_executable(trajectory_follower src/trajectory_follower_node.cpp src/FeedForwardController.cpp)
add_executable(trajectory_generator src/trajectory_generator_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_follower trajectory_controller ${catkin_LIBRARIES})
target_link_libraries(trajectory_generator ${catkin_LIBRARIES})
