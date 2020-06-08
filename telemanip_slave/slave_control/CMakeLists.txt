cmake_minimum_required(VERSION 2.8.3)
project(slave_control)

# check c++11 / c++0x
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

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_filters geometry_msgs message_generation dynamic_reconfigure)

add_message_files(FILES HapticState.msg ControlState.msg ControlComm.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

generate_dynamic_reconfigure_options(cfg/parameters.cfg)

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_filters geometry_msgs message_generation dynamic_reconfigure)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(slave_control src/slave_control.cpp)
add_executable(slave_parameters src/slave_parameters.cpp)

target_link_libraries(slave_control ${catkin_LIBRARIES})
target_link_libraries(slave_parameters ${catkin_LIBRARIES})

add_dependencies(slave_control ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(slave_parameters ${PROJECT_NAME}_gencfg)


install(TARGETS slave_control slave_parameters
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

