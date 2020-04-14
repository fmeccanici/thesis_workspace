cmake_minimum_required(VERSION 2.8.3)
project(master_control)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation)

add_message_files(FILES ControlComm.msg ControlState.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs)
