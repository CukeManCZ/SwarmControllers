cmake_minimum_required(VERSION 3.8)
project(swarm_tools)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

add_library(shape_lib
  src/shape_builder.cpp
)

target_include_directories(shape_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(shape_lib
  Eigen3::Eigen
  octomap
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS shape_lib
  EXPORT shape_lib_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(shape_lib_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(shape_lib)
ament_export_dependencies(Eigen3 octomap)

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
