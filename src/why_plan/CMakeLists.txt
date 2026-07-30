cmake_minimum_required(VERSION 3.16)
project(why_plan VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(semaforr_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(why_plan src/main.cpp)
target_compile_features(why_plan PRIVATE cxx_std_20)
target_compile_options(why_plan PRIVATE -Wall -Wextra -Wpedantic -Werror)
ament_target_dependencies(
  why_plan
  rclcpp
  semaforr_msgs
  std_msgs
)

install(TARGETS why_plan DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    why_plan_ros2_contract_test
    test/test_ros2_contract.py
    ENV "WHY_PLAN_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
endif()

ament_package()
