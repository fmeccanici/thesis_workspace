cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_msgs)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  DynamixelAX.msg
  DynamixelCommand.msg
  DynamixelEX.msg
  DynamixelMX.msg
  DynamixelMX64.msg
  DynamixelMX106.msg
  DynamixelPro.msg
  DynamixelProL42.msg
  DynamixelRX.msg
  DynamixelXL.msg
  DynamixelXM.msg
  MotorState.msg
  MotorStateList.msg
  WorkbenchParam.msg
)

add_service_files(
  FILES
  GetWorkbenchParam.srv
  SetDirection.srv
  SetPosition.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
   CATKIN_DEPENDS std_msgs message_generation
)
