cmake_minimum_required(VERSION 3.0.2)
project(arm_us)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  arm_us_msg
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs arm_us_msg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(libraries/ArmUs)
add_subdirectory(libraries/ArmUs_Info)
add_subdirectory(libraries/ArmUs_Controller)

add_executable(arm_us_node src/master_node.cpp)
add_dependencies(arm_us_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_us_node ${catkin_LIBRARIES} ArmUs ArmUs_Info ArmUs_Controller)

add_executable(motor_translator src/motor_translator.cpp)
add_dependencies(motor_translator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_translator ${catkin_LIBRARIES})
