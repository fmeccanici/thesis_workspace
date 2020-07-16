  cmake_minimum_required(VERSION 2.8.3)
  project(data_logger)
  
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
    promp_context_ros
  )

  add_service_files(
    FILES
    CreateParticipant.srv
    AddRefinement.srv
    SetObjectMissed.srv
    SetObstaclesHit.srv
    ToCsv.srv
    SetPrediction.srv
    IncrementNumberOfRefinements.srv
    SetParameters.srv
  )
  add_message_files(
    FILES
    TrajectoryData.msg
    OperatorGUIinteraction.msg
  )

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  promp_context_ros
)
  catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)