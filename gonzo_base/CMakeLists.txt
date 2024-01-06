cmake_minimum_required(VERSION 3.0.2)
project(gonzo_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  hardware_interface
  std_msgs
  gonzo_msgs
  controller_manager
  rosparam_shortcuts
  urdf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
# find_package(Eigen REQUIRED)


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
 INCLUDE_DIRS include
 LIBRARIES gonzo_hw_interface
 CATKIN_DEPENDS hardware_interface roscpp rospy std_msgs gonzo_msgs controller_manager urdf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(gonzo_hw_interface
  src/gonzo_hw_interface.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(gonzo_hw_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(gonzo_base src/gonzo_base.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(gonzo_base
#   ${catkin_LIBRARIES} 
#   gonzo_hw_interface
# )

target_link_libraries(
  gonzo_hw_interface
  ${catkin_LIBRARIES} 
)
target_link_libraries(gonzo_base
  gonzo_hw_interface
  ${catkin_LIBRARIES} 
)
