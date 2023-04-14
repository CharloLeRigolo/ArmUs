cmake_minimum_required(VERSION 3.0.2)
project(arm_us_graph)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
  arm_us_msg
)

catkin_package(
 INCLUDE_DIRS
 LIBRARIES arm_us_graph
 CATKIN_DEPENDS roscpp rospy visualization_msgs arm_us_msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(graph src/graph.cpp)
add_dependencies(graph ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph ${catkin_LIBRARIES})
