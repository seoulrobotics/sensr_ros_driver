cmake_minimum_required(VERSION 2.8.3)
project(argos_driver)

set(${PROJECT_NAME}_CATKIN_DEPS 
    nodelet
    roscpp)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# This driver uses Boost threads
find_package(argos_sdk REQUIRED)

# objects needed by other ROS packages that depend on this one
catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
               INCLUDE_DIRS include)

add_subdirectory(src/driver)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_argos.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)