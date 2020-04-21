cmake_minimum_required(VERSION 2.8.3)
project(marco_control)


if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  eigen_conversions
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(plan_arm_cartesian src/plan_arm_cartesian.cpp)
add_executable(plan_arm_joint src/plan_arm_joint.cpp)





add_dependencies(plan_arm_cartesian ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plan_arm_joint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(plan_arm_cartesian ${catkin_LIBRARIES})
target_link_libraries(plan_arm_joint ${catkin_LIBRARIES})


install(TARGETS plan_arm_cartesian
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install(TARGETS plan_arm_joint
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})