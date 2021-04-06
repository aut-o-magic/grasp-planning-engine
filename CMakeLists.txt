cmake_minimum_required(VERSION 3.5)
project(grasp-planning-engine)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-copy)
endif()

# include additional directories
include_directories(
  /opt/ros/foxy/include
)

# find dependencies

# Octomap imports
find_package(octomap REQUIRED)
find_package(octomap_grasping REQUIRED)
include_directories(
${OCTOMAP_INCLUDE_DIRS}
${octomap_grasping_INCLUDE_DIRS}
)

add_library(gp_engine src/gp_engine.cpp)
target_include_directories(gp_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(gp_engine
${OCTOMAP_LIBRARIES}
${octomap_grasping_LIBRARIES}
pthread # for std::thread and std::ref
)

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

target_include_directories(gp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(gp_node
${OCTOMAP_LIBRARIES}
${octomap_grasping_LIBRARIES}
gp_engine
)

install(TARGETS gp_node
  DESTINATION lib/${PROJECT_NAME})
