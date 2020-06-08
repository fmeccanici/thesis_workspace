cmake_minimum_required(VERSION 2.8.3)
project(teach_pendant)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  std_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
  catkin_package(CATKIN_DEPENDS std_msgs message_runtime
)