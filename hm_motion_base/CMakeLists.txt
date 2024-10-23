cmake_minimum_required(VERSION 3.8)
project(hm_motion_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(HM_MOTION_BASE hm_motion_base)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  tf2_kdl
  orocos_kdl
  cartesian_controller_msgs
  Eigen3
  rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Library for motion base
include_directories(include/hm_motion_base)
set(HEADER_FILES include/hm_motion_base/hm_motion_base.hpp)
add_library(${HM_MOTION_BASE} SHARED
  src/hm_motion_base.cpp
  ${HEADER_FILES})
ament_target_dependencies(${HM_MOTION_BASE} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${HM_MOTION_BASE} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_${HM_MOTION_BASE} HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/
  DESTINATION include
)

INSTALL(
  TARGETS ${HM_MOTION_BASE}
  EXPORT export_${HM_MOTION_BASE}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${HM_MOTION_BASE})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
