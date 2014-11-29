include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

include_directories(${ROSCPP_INCLUDE_DIRS})
link_directories(${ROSCPP_LIBRARY_DIRS})

include_directories(${HRPSYS_INCLUDE_DIRS})
link_directories(${HRPSYS_LIBRARY_DIRS})

add_library(ROSCamera SHARED roscamera.cc)
target_link_libraries(ROSCamera ${ROSCPP_LIBRARIES} ${OPENRTM_LIBRARIES} pthread)
set_target_properties(ROSCamera PROPERTIES PREFIX "")
