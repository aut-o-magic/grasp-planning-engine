cmake_minimum_required(VERSION 3.5)
project(grasp_planning_engine)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include additional directories
include_directories(include
  /opt/ros/foxy/include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

# Octomap imports
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_grasping REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS}
${octomap_grasping_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  octomap_msgs
  octomap_grasping
)

add_library(gp_engine src/gp_engine.cpp)
target_include_directories(gp_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(gp_engine PRIVATE "GP_ENGINE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS gp_engine
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Grasp Planning Node executable

add_executable(gp_node src/gp_node.cpp)

ament_target_dependencies(gp_node
  ${dependencies}
)

target_include_directories(gp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(gp_node
${OCTOMAP_LIBRARIES}
gp_engine
)

install(TARGETS gp_node
  DESTINATION lib/${PROJECT_NAME})


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

ament_export_include_directories(
  include
)
ament_export_libraries(
  gp_engine
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
