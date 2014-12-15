set(target hrpsys-gazebo-simulator)

add_executable(${target} hrpsys-gazebo-simulator.cpp SimulatorGazebo.cpp GZbodyRTC.cpp Project.cpp ProjectUtil.cpp OpenRTMUtil.cpp)
target_link_libraries(${target} ${GAZEBO_LIBRARIES} ${OPENRTM_LIBRARIES} ${ROSCPP_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${LIBXML2_LIBRARIES} hrpsysBaseStub image_transport pthread)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )
