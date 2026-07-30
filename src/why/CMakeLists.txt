cmake_minimum_required(VERSION 3.16)
project(why VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(semaforr_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(why src/main.cpp)
target_compile_features(why PRIVATE cxx_std_20)
target_compile_options(why PRIVATE -Wall -Wextra -Wpedantic -Werror)
ament_target_dependencies(
  why
  rclcpp
  semaforr_msgs
  std_msgs
)

install(TARGETS why DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    why_ros2_contract_test
    test/test_ros2_contract.py
    ENV "WHY_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
endif()

ament_package()
