cmake_minimum_required(VERSION 2.8.3)
project(skiros_lib_conditions)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS 
  rospy
  roscpp
  pluginlib
  skiros_common
  skiros_skill
  skiros_config
)

## System dependencies are found with CMake's conventions (skill-AAU)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES skiros_lib_conditions
  #CATKIN_DEPENDS  roscpp pluginlib skiros_msgs skiros_common skiros_skill
)

###########
## Build ##
###########

include_directories(include
${catkin_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(skiros_lib_conditions
  src/condition_types.cpp
)
target_link_libraries(skiros_lib_conditions ${catkin_LIBRARIES} ${Boost_LIBRARIES})
