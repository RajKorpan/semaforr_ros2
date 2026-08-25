# File overview: This file exercises build behavior for automated verification and regression testing. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `test/integration/downstream/CMakeLists.txt`.
cmake_minimum_required(VERSION 3.16)
project(semaforr_downstream_smoke LANGUAGES CXX)

find_package(semaforr REQUIRED)

add_executable(semaforr_downstream_smoke main.cpp)
target_link_libraries(semaforr_downstream_smoke PRIVATE semaforr::domain)

add_executable(semaforr_planning_smoke main.cpp)
target_link_libraries(semaforr_planning_smoke PRIVATE semaforr::planning)

add_executable(semaforr_advisors_smoke main.cpp)
target_link_libraries(semaforr_advisors_smoke PRIVATE semaforr::advisors)

add_executable(semaforr_spatial_smoke spatial_main.cpp)
target_link_libraries(semaforr_spatial_smoke PRIVATE semaforr::spatial)

add_executable(semaforr_navigation_smoke main.cpp)
target_link_libraries(semaforr_navigation_smoke PRIVATE semaforr::navigation)

add_executable(semaforr_ros_adapters_smoke main.cpp)
target_link_libraries(
  semaforr_ros_adapters_smoke PRIVATE semaforr::ros_adapters)
