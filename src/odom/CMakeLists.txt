cmake_minimum_required(VERSION 2.8.3)
project(odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robmovil_msgs
  geometry_msgs
  cmake_modules
  tf
  lazo_cerrado
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
#  INCLUDE_DIRS include
#  LIBRARIES robmovil_ekf
   CATKIN_DEPENDS roscpp lazo_cerrado
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(robot_odometry src/robot_odometry_node.cpp src/robot_odometry.cpp)
add_executable(loggerOdom src/logger_node_odom.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_odometry
  ${catkin_LIBRARIES}
)
target_link_libraries(loggerOdom ${catkin_LIBRARIES})

