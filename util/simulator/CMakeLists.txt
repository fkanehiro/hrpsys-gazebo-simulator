add_executable(hrpsys-gazebo-simulation hrpsys-gazebo-simulation.cc SimulatorGazebo.cpp GZbodyRTC.cpp Project.cpp ProjectUtil.cpp OpenRTMUtil.cpp)
target_link_libraries(hrpsys-gazebo-simulation ${GAZEBO_LIBRARIES} ${OPENRTM_LIBRARIES} ${ROSCPP_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${LIBXML2_LIBRARIES} pthread)
