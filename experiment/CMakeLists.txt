  cmake_minimum_required(VERSION 2.8.3)
  project(experiment)
  
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
  )

  #add_service_files(
  #  FILES
  #)

  #add_message_files(
  #  FILES
  #)

  catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)