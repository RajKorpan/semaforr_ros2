cmake_minimum_required(VERSION 3.16)
project(semaforr VERSION 0.1.0 LANGUAGES C CXX)

option(SEMAFORR_ENABLE_SANITIZERS
  "Build SemaFORR with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(SEMAFORR_ENABLE_COVERAGE
  "Build SemaFORR with GCC/Clang coverage instrumentation" OFF)

if(SEMAFORR_ENABLE_SANITIZERS AND SEMAFORR_ENABLE_COVERAGE)
  message(FATAL_ERROR
    "SEMAFORR_ENABLE_SANITIZERS and SEMAFORR_ENABLE_COVERAGE cannot be enabled together")
endif()

function(semaforr_apply_project_options target)
  target_compile_features(${target} PUBLIC cxx_std_14)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  elseif(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  endif()

  if(SEMAFORR_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${target} PRIVATE
        -fno-omit-frame-pointer
        -fno-sanitize-recover=all
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

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CrowdModel.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Preserve the legacy translation-unit order while known undefined-behavior
# findings are addressed. Phase 2 changes paths, not link ordering.
set(SEMAFORR_DOMAIN_SOURCES
  src/config/Configuration.cpp
  src/decision/AgentState.cpp
  src/navigation/astar.cpp
  src/decision/Controller.cpp
  src/core/FORRAction.cpp
  src/spatial/FORRBarriers.cpp
  src/spatial/FORRConveyors.cpp
  src/core/FORRGeometry.cpp
  src/spatial/FORRHallways.cpp
  src/spatial/FORRTrails.cpp
  src/navigation/Graph.cpp
  src/navigation/Map.cpp
  src/navigation/PathPlanner.cpp
  src/core/Position.cpp
  src/decision/Tier1Advisor.cpp
  src/decision/Tier3Advisor.cpp
  src/vendor/tinyxml/tinystr.cpp
  src/vendor/tinyxml/tinyxml.cpp
  src/vendor/tinyxml/tinyxmlerror.cpp
  src/vendor/tinyxml/tinyxmlparser.cpp
)

add_library(semaforr_domain SHARED ${SEMAFORR_DOMAIN_SOURCES})
set_target_properties(semaforr_domain PROPERTIES EXPORT_NAME domain)
add_library(semaforr::domain ALIAS semaforr_domain)
target_include_directories(semaforr_domain
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
semaforr_apply_project_options(semaforr_domain)

# Preserve the Phase 1 target name for downstream consumers while directing
# new code to the explicit ROS-independent domain target.
add_library(semaforr_core INTERFACE)
set_target_properties(semaforr_core PROPERTIES EXPORT_NAME core)
add_library(semaforr::core ALIAS semaforr_core)
target_link_libraries(semaforr_core INTERFACE semaforr_domain)

add_library(semaforr_ros_adapters SHARED src/ros/MessageAdapters.cpp)
set_target_properties(
  semaforr_ros_adapters PROPERTIES EXPORT_NAME ros_adapters)
add_library(semaforr::ros_adapters ALIAS semaforr_ros_adapters)
target_include_directories(semaforr_ros_adapters
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  semaforr_ros_adapters semaforr_domain "${cpp_typesupport_target}")
ament_target_dependencies(semaforr_ros_adapters
  geometry_msgs
  sensor_msgs
  std_msgs
)
semaforr_apply_project_options(semaforr_ros_adapters)

add_executable(semaforr_node src/ros/semaforr_node.cpp)
target_link_libraries(
  semaforr_node
  semaforr_domain
  semaforr_ros_adapters
  "${cpp_typesupport_target}"
)
ament_target_dependencies(semaforr_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  visualization_msgs
)
semaforr_apply_project_options(semaforr_node)

install(
  TARGETS semaforr_domain semaforr_ros_adapters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  TARGETS semaforr_core
  EXPORT export_${PROJECT_NAME}
)
install(
  TARGETS semaforr_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/record_baseline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semaforr_record_baseline
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  add_executable(semaforr_characterization_test
    test/characterization_smoke.cpp
    src/core/FORRAction.cpp
    src/core/Position.cpp
  )
  target_include_directories(semaforr_characterization_test PRIVATE
    ${PROJECT_SOURCE_DIR}/include
  )
  semaforr_apply_project_options(semaforr_characterization_test)
  add_test(
    NAME semaforr_characterization_test
    COMMAND semaforr_characterization_test
  )

  add_executable(
    semaforr_domain_model_test
    test/domain_model_smoke.cpp
  )
  target_link_libraries(semaforr_domain_model_test semaforr_domain)
  semaforr_apply_project_options(semaforr_domain_model_test)
  add_test(
    NAME semaforr_domain_model_test
    COMMAND semaforr_domain_model_test
  )

  add_executable(
    semaforr_message_adapters_test
    test/message_adapters_smoke.cpp
  )
  target_link_libraries(
    semaforr_message_adapters_test
    semaforr_ros_adapters
    "${cpp_typesupport_target}"
  )
  semaforr_apply_project_options(semaforr_message_adapters_test)
  add_test(
    NAME semaforr_message_adapters_test
    COMMAND semaforr_message_adapters_test
  )

  add_executable(
    semaforr_ownership_model_test
    test/ownership_model_smoke.cpp
  )
  target_link_libraries(semaforr_ownership_model_test semaforr_domain)
  target_compile_definitions(semaforr_ownership_model_test PRIVATE
    "SEMAFORR_TEST_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\""
  )
  semaforr_apply_project_options(semaforr_ownership_model_test)
  add_test(
    NAME semaforr_ownership_model_test
    COMMAND semaforr_ownership_model_test
  )

  add_executable(
    semaforr_configuration_test
    test/configuration_smoke.cpp
  )
  target_link_libraries(semaforr_configuration_test semaforr_domain)
  target_compile_definitions(semaforr_configuration_test PRIVATE
    "SEMAFORR_TEST_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\""
  )
  semaforr_apply_project_options(semaforr_configuration_test)
  add_test(
    NAME semaforr_configuration_test
    COMMAND semaforr_configuration_test
  )

  ament_add_pytest_test(semaforr_source_contract_test
    test/test_source_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_build_contract_test
    test/test_build_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_baseline_recorder_contract_test
    test/test_baseline_recorder_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_domain_boundary_test
    test/test_domain_boundary.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_ownership_contract_test
    test/test_ownership_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_configuration_contract_test
    test/test_configuration_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  rosidl_default_runtime
  sensor_msgs
  std_msgs
  tf2
  visualization_msgs
)

ament_package()
