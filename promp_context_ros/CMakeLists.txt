  cmake_minimum_required(VERSION 2.8.3)
  project(promp_context_ros)
  
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
  )

add_message_files(
   FILES
   prompData.msg
   prompPoint.msg
   prompTraj.msg
   prompContext.msg
 )

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
  catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)