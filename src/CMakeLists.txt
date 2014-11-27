include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

include_directories(${ROSCPP_INCLUDE_DIRS})
link_directories(${ROSCPP_LIBRARY_DIRS})

include_directories(${HRPSYS_INCLUDE_DIRS})
link_directories(${HRPSYS_LIBRARY_DIRS})

add_executable(hrpsys-gazebo-simulation hrpsys-gazebo-simulation.cc SimulatorGazebo.cpp)
target_link_libraries(hrpsys-gazebo-simulation ${GAZEBO_LIBRARIES} ${HRPSYS_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} hrpIo hrpsysUtil pthread)

add_library(ROSBody SHARED rosbody.cc)
set_target_properties(ROSBody PROPERTIES PREFIX "")

add_library(ROSCamera SHARED roscamera.cc)
set_target_properties(ROSCamera PROPERTIES PREFIX "")
