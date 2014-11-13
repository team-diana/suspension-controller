cmake_minimum_required(VERSION 2.8.3)
project(suspension_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Status.msg
)

add_service_files(
  FILES
  SetHeight.srv
  SetMode.srv
  StopAll.srv
  Freeze.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
