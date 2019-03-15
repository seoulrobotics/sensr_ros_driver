# build the driver node
add_executable(argos_node argos_node.cc driver.cc)
target_link_libraries(argos_node
  ${catkin_LIBRARIES} argos_sdk ${CMAKE_DL_LIBS} uuid
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
target_link_libraries(driver_nodelet
  ${catkin_LIBRARIES} argos_sdk ${CMAKE_DL_LIBS} uuid
)

target_include_directories(driver_nodelet PRIVATE argos_sdk::INCLUDE_DIRECTORIES)
target_include_directories(argos_node PRIVATE argos_sdk::INCLUDE_DIRECTORIES)


# install runtime files
install(TARGETS argos_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
