cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stream_pcd)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(catkin REQUIRED COMPONENTS
  PCL
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (stream_node src/StreamNode.cpp)
target_link_libraries (stream_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(stream_node ${PROJECT_NAME}_gencpp)