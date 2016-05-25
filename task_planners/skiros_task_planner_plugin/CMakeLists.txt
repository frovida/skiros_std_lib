cmake_minimum_required(VERSION 2.8.3)
project(skiros_task_planner_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  pluginlib
  skiros_task
)

## System dependencies are found with CMake's conventions (skill-AAU)
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
    LIBRARIES task_planner
    #CATKIN_DEPENDS actionlib roscpp pluginlib skiros_msgs skiros_common  skiros_device
    #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
${catkin_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS})

add_library(task_planner
    src/task_planner.cpp
)
target_link_libraries(task_planner ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})


