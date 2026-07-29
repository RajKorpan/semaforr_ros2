cmake_minimum_required(VERSION 3.16)
project(semaforr_downstream_smoke LANGUAGES CXX)

find_package(semaforr REQUIRED)

add_executable(semaforr_downstream_smoke main.cpp)
target_link_libraries(semaforr_downstream_smoke PRIVATE semaforr::domain)

add_executable(semaforr_core_compat_smoke main.cpp)
target_link_libraries(semaforr_core_compat_smoke PRIVATE semaforr::core)

add_executable(semaforr_planning_smoke main.cpp)
target_link_libraries(semaforr_planning_smoke PRIVATE semaforr::planning)

add_executable(semaforr_advisors_smoke main.cpp)
target_link_libraries(semaforr_advisors_smoke PRIVATE semaforr::advisors)

add_executable(semaforr_spatial_smoke spatial_main.cpp)
target_link_libraries(semaforr_spatial_smoke PRIVATE semaforr::spatial)

add_executable(semaforr_ros_smoke main.cpp)
target_link_libraries(semaforr_ros_smoke PRIVATE semaforr::ros)
