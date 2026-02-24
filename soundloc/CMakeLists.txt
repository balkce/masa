cmake_minimum_required(VERSION 3.5)
project(soundloc)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-but-set-variable)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

INCLUDE(FindPkgConfig)
pkg_check_modules (GLIB2 glib-2.0)
pkg_check_modules (JACK jack)
pkg_check_modules (FFTW fftw3)
pkg_check_modules (EIGEN eigen3)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DOA.msg"
  "msg/Frequencies.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${GLIB2_INCLUDE_DIRS}
  ${JACK_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_library(multisoundloc src/multisoundloc.cpp)
target_link_libraries(multisoundloc
  ${GLIB2_LIBRARIES}
  ${JACK_LIBRARIES}
  ${FFTW_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

ament_export_targets(multisoundloc HAS_LIBRARY_TARGET)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS multisoundloc
  EXPORT multisoundloc
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(soundloc_multi src/soundloc_multi.cpp)
ament_target_dependencies(soundloc_multi rclcpp std_srvs std_msgs)
target_link_libraries(soundloc_multi
  multisoundloc
  ${GLIB2_LIBRARIES}
  ${cpp_typesupport_target}
)

install(TARGETS soundloc_multi
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
