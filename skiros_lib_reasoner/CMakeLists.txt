cmake_minimum_required(VERSION 2.8.3)
project(skiros_lib_reasoner)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  pluginlib
  skiros_msgs
  skiros_common
  skiros_config
  skiros_world_model
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  visualization_msgs
)
## System dependencies are found with CMake's conventions (skill-AAU)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS aau_spatial_reasoner/include
  LIBRARIES aau_spatial_reasoner
  #CATKIN_DEPENDS actionlib roscpp pluginlib skiros_msgs skiros_common  skiros_device
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
${catkin_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS}
${EIGEN_INCLUDE_DIRS})

link_directories(${EIGEN_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(aau_spatial_reasoner)

