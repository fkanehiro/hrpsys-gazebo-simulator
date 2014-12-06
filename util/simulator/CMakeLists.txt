add_executable(hrpsys-gazebo-simulation hrpsys-gazebo-simulation.cc SimulatorGazebo.cpp GZbodyRTC.cpp)
target_link_libraries(hrpsys-gazebo-simulation ${GAZEBO_LIBRARIES} ${HRPSYS_LIBRARIES} ${OPENRTM_LIBRARIES} ${ROSCPP_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} hrpIo hrpsysUtil pthread)
