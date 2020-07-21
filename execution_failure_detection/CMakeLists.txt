  cmake_minimum_required(VERSION 2.8.3)
  project(execution_failure_detection)
  
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
  )

  add_service_files(
    FILES
    GetExecutionFailure.srv
    SetExpectedObjectPosition.srv
  )

add_message_files(
   FILES
   ExecutionFailure.msg
 )

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
  catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs
)