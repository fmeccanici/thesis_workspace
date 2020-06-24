  cmake_minimum_required(VERSION 2.8.3)
  project(learning_from_demonstration)
  
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    message_generation
    promp_context_ros
  )

  add_service_files(
    FILES
    AddDemonstration.srv
    MakePrediction.srv
    SetObject.srv
    GetContext.srv
    GoToPose.srv
    ExecuteTrajectory.srv
    GetObjectPosition.srv
    WelfordUpdate.srv
    SetTeachingMode.srv
    BuildInitialModel.srv
    GetEEPose.srv
    SetPath.srv
    GetDemonstration.srv
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