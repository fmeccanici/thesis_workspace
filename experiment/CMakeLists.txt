  cmake_minimum_required(VERSION 2.8.3)
  project(experiment)
  
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
  )

  add_message_files(
    FILES
    FormGUIinteraction.msg
  )

  add_service_files(
    FILES
    SetText.srv
  )

  catkin_python_setup()

  #add_message_files(
  #  FILES
  #)
  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
  )
  catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)