cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OPENRTM openrtm-aist)
  pkg_check_modules(HRPSYS hrpsys-base)
  pkg_check_modules(ROSCPP roscpp)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

add_subdirectory(src)
add_subdirectory(rtc)
