  cmake_minimum_required(VERSION 2.8.3)
  project(rls)
  
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
  )


catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)
  catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs
)