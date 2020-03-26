  cmake_minimum_required(VERSION 2.8.3)
  project(trajectory_parser)
  find_package(catkin REQUIRED COMPONENTS
    rospy
  )
  catkin_python_setup()
  catkin_package()