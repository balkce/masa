cmake_minimum_required(VERSION 3.10)
project(rosjack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jack_msgs REQUIRED)

INCLUDE(FindPkgConfig)
pkg_check_modules (JACK jack)
#message( "Found jack: " ${JACK_FOUND} )
pkg_check_modules (SNDFILE sndfile)
#message( "Found sndfile: " ${SNDFILE_FOUND} )
pkg_check_modules (SAMPLERATE samplerate)
#message( "Found samplerate: " ${SAMPLERATE_FOUND} )

include_directories(
  include/rosjack
  ${JACK_INCLUDE_DIRS}
  ${SNDFILE_INCLUDE_DIRS}
  ${SAMPLERATE_INCLUDE_DIRS}
)

add_library(rosjack src/rosjack.cpp include/rosjack/rosjack.hpp) 

target_link_libraries(rosjack
  ${JACK_LIBRARIES}
  ${SNDFILE_LIBRARIES}
  ${SAMPLERATE_LIBRARIES}
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${jack_msgs_TARGETS}
)

ament_export_targets(rosjack HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/rosjack
  DESTINATION include
)

install(
  TARGETS rosjack
  EXPORT rosjack
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
