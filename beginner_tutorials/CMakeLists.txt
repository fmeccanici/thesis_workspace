  cmake_minimum_required(VERSION 2.8.3)
  project(beginner_tutorials)
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
  )

  add_service_files(
  FILES
  AddTwoInts.srv
  )

  generate_messages(
  DEPENDENCIES
  std_msgs
)

  catkin_package()