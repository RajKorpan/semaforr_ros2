cmake_minimum_required(VERSION 3.16)
project(why VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(semaforr_msgs REQUIRED)

add_library(why_core src/why_system.cpp)
target_compile_features(why_core PUBLIC cxx_std_20)
target_compile_options(why_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(
  why_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(why_core geometry_msgs semaforr_msgs)

add_executable(why src/main.cpp)
target_compile_features(why PRIVATE cxx_std_20)
target_compile_options(why PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(why why_core)
ament_target_dependencies(why rclcpp semaforr_msgs)

install(
  TARGETS why_core why
  EXPORT why_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(why_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs semaforr_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(why_system_test test/why_system_test.cpp)
  if(TARGET why_system_test)
    target_link_libraries(why_system_test why_core)
    ament_target_dependencies(why_system_test geometry_msgs semaforr_msgs)
  endif()
  ament_add_pytest_test(
    why_ros2_contract_test
    test/test_ros2_contract.py
    ENV "WHY_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
endif()

ament_package()
