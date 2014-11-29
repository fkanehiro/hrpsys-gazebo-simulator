include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

include_directories(${ROSCPP_INCLUDE_DIRS})
link_directories(${ROSCPP_LIBRARY_DIRS})

include_directories(${HRPSYS_INCLUDE_DIRS})
link_directories(${HRPSYS_LIBRARY_DIRS})

add_library(ROSBody SHARED rosbody.cc)
target_link_libraries(ROSBody ${ROSCPP_LIBRARIES} ${OPENRTM_LIBRARIES} pthread)
set_target_properties(ROSBody PROPERTIES PREFIX "")
