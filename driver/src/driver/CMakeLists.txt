# build the driver node
add_executable(sensr_node sensr_node.cc driver.cc)
target_link_libraries(sensr_node
  ${catkin_LIBRARIES} sensr_sdk ${CMAKE_DL_LIBS} uuid
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
target_link_libraries(driver_nodelet
  ${catkin_LIBRARIES} sensr_sdk ${CMAKE_DL_LIBS} uuid
)

target_include_directories(driver_nodelet PRIVATE sensr_sdk::INCLUDE_DIRECTORIES)
target_include_directories(sensr_node PRIVATE sensr_sdk::INCLUDE_DIRECTORIES)


# install runtime files
install(TARGETS sensr_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
