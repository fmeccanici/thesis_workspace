  cmake_minimum_required(VERSION 2.8.3)
  project(python2to3_function_caller)
  find_package(catkin REQUIRED COMPONENTS
    rospy
  )
  catkin_python_setup()
  catkin_package()