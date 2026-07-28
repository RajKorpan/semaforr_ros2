cmake_minimum_required(VERSION 3.8)
project(semaforr)

option(SEMAFORR_ENABLE_SANITIZERS
  "Build SemaFORR with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(SEMAFORR_ENABLE_COVERAGE
  "Build SemaFORR with GCC/Clang coverage instrumentation" OFF)

if(SEMAFORR_ENABLE_SANITIZERS AND SEMAFORR_ENABLE_COVERAGE)
  message(FATAL_ERROR
    "SEMAFORR_ENABLE_SANITIZERS and SEMAFORR_ENABLE_COVERAGE cannot be enabled together")
endif()

function(semaforr_apply_instrumentation target)
  if(SEMAFORR_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${target} PRIVATE
        -fno-omit-frame-pointer
        -fsanitize=address,undefined)
      target_link_options(${target} PRIVATE
        -fno-omit-frame-pointer
        -fsanitize=address,undefined)
    else()
      message(FATAL_ERROR
        "SemaFORR sanitizer profile currently supports GCC and Clang only")
    endif()
  endif()

  if(SEMAFORR_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${target} PRIVATE -O0 -g --coverage)
      target_link_options(${target} PRIVATE --coverage)
    else()
      message(FATAL_ERROR
        "SemaFORR coverage profile currently supports GCC and Clang only")
    endif()
  endif()
endfunction()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add Python3 development package
find_package(Python3 REQUIRED COMPONENTS Development)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CrowdModel.msg"
  DEPENDENCIES std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include/semaforr/
  ${Python3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c/
)

## Declare a C++ executable
file(
  GLOB_RECURSE
  source_files
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/include/semaforr/*.h
)

add_executable(semaforr_node ${source_files})
semaforr_apply_instrumentation(semaforr_node)

# Link against Python3 libraries
target_link_libraries(semaforr_node ${Python3_LIBRARIES})

ament_target_dependencies(semaforr_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "visualization_msgs"
)

# Install targets
install(TARGETS semaforr_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/record_baseline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semaforr_record_baseline
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  add_executable(semaforr_characterization_test
    test/characterization_smoke.cpp
    src/FORRAction.cpp
    src/Position.cpp
  )
  target_include_directories(semaforr_characterization_test PRIVATE
    ${PROJECT_SOURCE_DIR}/include/semaforr
  )
  semaforr_apply_instrumentation(semaforr_characterization_test)
  add_test(NAME semaforr_characterization_test
    COMMAND semaforr_characterization_test)

  ament_add_pytest_test(semaforr_source_contract_test
    test/test_source_contract.py
    ENV
      "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
endif()

ament_package()
