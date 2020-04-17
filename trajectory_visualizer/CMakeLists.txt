  cmake_minimum_required(VERSION 2.8.3)
  project(trajectory_visualizer)
  find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs message_generation learning_from_demonstration)


  add_message_files(
    FILES
    TrajectoryVisualization.msg
  )


  add_service_files(
    FILES
    ClearTrajectories.srv
    VisualizeTrajectory.srv
  )

  catkin_python_setup()

  generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
    learning_from_demonstration
  )

  catkin_package()