add_library(velodyne_hdl_driver hdl_grabber_driver.cpp quaternion_fnc.cpp)
target_link_libraries(velodyne_hdl_driver
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(velodyne_hdl_driver ${catkin_EXPORTED_TARGETS})
endif()

##install(TARGETS velodyne_hdl_driver
##        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##) 
