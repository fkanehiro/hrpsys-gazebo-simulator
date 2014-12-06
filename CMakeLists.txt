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
find_package(LibXml2 REQUIRED)

include_directories(${LIBXML2_INCLUDE_DIR})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

include_directories(${ROSCPP_INCLUDE_DIRS})
link_directories(${ROSCPP_LIBRARY_DIRS})

include_directories(${HRPSYS_INCLUDE_DIRS})
link_directories(${HRPSYS_LIBRARY_DIRS})

add_subdirectory(util)
add_subdirectory(rtc)
