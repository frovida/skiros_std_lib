include_directories(include
${CMAKE_CURRENT_BINARY_DIR})

add_library(aau_spatial_reasoner
src/aau_spatial_reasoner.cpp
)
target_link_libraries(aau_spatial_reasoner ${catkin_LIBRARIES})

add_dependencies(aau_spatial_reasoner skiros_msgs_generate_messages_cpp)
