cmake_minimum_required(VERSION 3.0.2)
project(arm_us_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GuiInfo.msg
  GraphInfo.msg
  JointLimits.msg
)

add_service_files(
  FILES
  InverseKinematicCalc.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs
)
