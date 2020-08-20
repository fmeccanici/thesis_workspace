cmake_minimum_required(VERSION 2.8.3)
project(contact_sensor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED message_generation std_msgs geometry_msgs)
find_package(gazebo REQUIRED COMPONENTS std_msgs geometry_msgs)


add_message_files(
FILES
GripperCollision.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs message_generation
)



