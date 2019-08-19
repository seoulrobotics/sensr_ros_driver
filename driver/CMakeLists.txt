cmake_minimum_required(VERSION 3.8)
project(sensr_driver)

set(${PROJECT_NAME}_CATKIN_DEPS 
    nodelet
    roscpp
    sensr_msgs)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

find_package(Protobuf REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(sensr_sdk REQUIRED)

# objects needed by other ROS packages that depend on this one
catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
               INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM ${catkin_INCLUDE_DIRS} sensr_sdk::INCLUDE_DIRECTORIES ${})

add_subdirectory(src/driver)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_sensr.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch) 