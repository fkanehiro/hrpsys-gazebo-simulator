set(comp_sources SimulatorGazebo.cpp RTCBody.cpp)
add_library(SimulatorGazebo SHARED ${comp_sources})
set(libraries
  hrpsysUtil 
  hrpsysBaseStub
  )
target_link_libraries(SimulatorGazebo ${libraries})
set_target_properties(SimulatorGazebo PROPERTIES PREFIX "")

add_executable(SimulatorGazeboComp SimulatorGazeboComp.cpp ${comp_sources})
target_link_libraries(SimulatorGazeboComp ${libraries})

set(target SimulatorGazebo SimulatorGazeboComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
