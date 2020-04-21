cmake_minimum_required(VERSION 2.8.3)
project(marco_bringup)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
