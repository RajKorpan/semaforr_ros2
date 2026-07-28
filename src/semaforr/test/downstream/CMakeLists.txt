cmake_minimum_required(VERSION 3.16)
project(semaforr_downstream_smoke LANGUAGES CXX)

find_package(semaforr REQUIRED)

add_executable(semaforr_downstream_smoke main.cpp)
target_link_libraries(semaforr_downstream_smoke PRIVATE semaforr::domain)

add_executable(semaforr_core_compat_smoke main.cpp)
target_link_libraries(semaforr_core_compat_smoke PRIVATE semaforr::core)
