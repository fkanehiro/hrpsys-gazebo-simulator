add_executable(hrpsys-gazebo-simulator hrpsys-gazebo-simulator.cpp SimulatorGazebo.cpp GZbodyRTC.cpp Project.cpp ProjectUtil.cpp OpenRTMUtil.cpp)
target_link_libraries(hrpsys-gazebo-simulator ${GAZEBO_LIBRARIES} ${OPENRTM_LIBRARIES} ${ROSCPP_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${LIBXML2_LIBRARIES} hrpsysBaseStub image_transport pthread)
