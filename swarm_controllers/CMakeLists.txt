cmake_minimum_required(VERSION 3.28)
project(swarm_controllers)

# --------------------------------------------------------------
# |                      build settings                        |
# --------------------------------------------------------------

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------
# |                       dependencies                         |
# --------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(mrs_msgs REQUIRED)
find_package(mrs_lib REQUIRED)

find_package(myc_messages REQUIRED)

find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(swarm_tools REQUIRED)

# --------------------------------------------------------------
# |                    Boid Controller                         |
# --------------------------------------------------------------

add_library(boid_controller SHARED
  src/boid_controller.cpp
)

target_include_directories(boid_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(boid_controller
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${std_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${mrs_msgs_TARGETS}
    ${myc_messages_TARGETS}
)

rclcpp_components_register_nodes(boid_controller
  "boid_controller::BoidController"
)

install(
  TARGETS boid_controller
  EXPORT export_boid_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_boid_controller HAS_LIBRARY_TARGET)

# --------------------------------------------------------------
# |                   Shape Controller                         |
# --------------------------------------------------------------

add_library(shape_controller SHARED
  src/shape_controller.cpp
)

target_include_directories(shape_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OCTOMAP_INCLUDE_DIRS}
)

ament_target_dependencies(shape_controller
  rclcpp
  rclcpp_components
  mrs_lib
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  std_srvs
  mrs_msgs
  myc_messages
  octomap
  octomap_msgs
  swarm_tools
)

rclcpp_components_register_nodes(shape_controller
  "shape_controller::ShapeController"
)

install(
  TARGETS shape_controller
  EXPORT export_shape_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_shape_controller HAS_LIBRARY_TARGET)


# --------------------------------------------------------------
# |                    Data Gatherer                           |
# --------------------------------------------------------------
add_library(data_gatherer SHARED
  src/data_gatherer.cpp
)

target_include_directories(data_gatherer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(data_gatherer
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${OCTOMAP_LIBRARIES}
    ${std_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${mrs_msgs_TARGETS}
    ${myc_messages_TARGETS}
    ${octomap_msgs_TARGETS}
)

rclcpp_components_register_nodes(data_gatherer
  "data_gatherer::DataGatherer"
)

install(
  TARGETS data_gatherer
  EXPORT export_data_gatherer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_data_gatherer HAS_LIBRARY_TARGET)

# --------------------------------------------------------------
# |                        install                             |
# --------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

# --------------------------------------------------------------
# |                         testing                            |
# --------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
