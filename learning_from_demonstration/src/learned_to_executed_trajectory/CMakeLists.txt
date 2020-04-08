  cmake_minimum_required(VERSION 2.8.3)
  project(learned_to_executed_trajectory)
  find_package(catkin REQUIRED COMPONENTS
    rospy
  )
  catkin_python_setup()
  catkin_package()