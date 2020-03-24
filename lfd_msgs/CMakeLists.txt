  cmake_minimum_required(VERSION 2.8.3)
  project(lfd_msgs)

  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
      set(CMAKE_CXX_FLAGS "-std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
      set(CMAKE_CXX_FLAGS "-std=c++0x")
  else()
      message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()

  find_package(catkin REQUIRED COMPONENTS message_generation rospy geometry_msgs)



  add_message_files(
      FILES
      TrajectoryVisualization.msg
  )

  generate_messages(
      DEPENDENCIES
      geometry_msgs
  )

  catkin_package(
    CATKIN_DEPENDS message_runtime
  )