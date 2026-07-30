cmake_minimum_required(VERSION 3.16)
project(semaforr VERSION 0.1.0 LANGUAGES C CXX)

option(SEMAFORR_ENABLE_SANITIZERS
  "Build SemaFORR with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(SEMAFORR_ENABLE_COVERAGE
  "Build SemaFORR with GCC/Clang coverage instrumentation" OFF)
option(SEMAFORR_STRICT_MODERN_CODE
  "Promote warnings to errors in refactored component code" ON)

if(SEMAFORR_ENABLE_SANITIZERS AND SEMAFORR_ENABLE_COVERAGE)
  message(FATAL_ERROR
    "SEMAFORR_ENABLE_SANITIZERS and SEMAFORR_ENABLE_COVERAGE cannot be enabled together")
endif()

function(semaforr_apply_project_options target)
  target_compile_features(${target} PUBLIC cxx_std_20)

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

function(semaforr_apply_strict_warnings target)
  if(SEMAFORR_STRICT_MODERN_CODE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${target} PRIVATE -Werror)
    elseif(MSVC)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  endif()
endfunction()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(semaforr_msgs REQUIRED)
find_package(social_context_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(SEMAFORR_DOMAIN_SOURCES
  src/config/Configuration.cpp
  src/domain/CrowdModel.cpp
  src/domain/MotionModel.cpp
  src/decision/DecisionResult.cpp
  src/social/CrowdFieldLearner.cpp
)

set(SEMAFORR_NAVIGATION_SOURCES
  src/decision/Enforcer.cpp
  src/decision/MissionManager.cpp
  src/decision/NavigationEngine.cpp
  src/decision/NavigationPhase.cpp
)

add_library(semaforr_planning SHARED
  src/navigation/DomainAStar.cpp
  src/navigation/DomainGraph.cpp
  src/navigation/DomainPlanner.cpp
  src/navigation/HighwayExplorer.cpp
  src/navigation/HierarchicalPlan.cpp
  src/navigation/MapParser.cpp
  src/navigation/PlanningCoordinator.cpp
)
set_target_properties(semaforr_planning PROPERTIES EXPORT_NAME planning)
add_library(semaforr::planning ALIAS semaforr_planning)
target_include_directories(semaforr_planning
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
semaforr_apply_project_options(semaforr_planning)
semaforr_apply_strict_warnings(semaforr_planning)

add_library(semaforr_advisors SHARED
  src/decision/DecisionCoordinator.cpp
  src/decision/HardSafetyFilter.cpp
  src/decision/LearnedCrowdAdvisor.cpp
  src/decision/NavigationAdvisor.cpp
  src/decision/ObstacleVetoRule.cpp
  src/decision/SocialAdvisorRegistry.cpp
  src/decision/SocialNavigationAdvisor.cpp
)
set_target_properties(semaforr_advisors PROPERTIES EXPORT_NAME advisors)
add_library(semaforr::advisors ALIAS semaforr_advisors)
target_include_directories(semaforr_advisors
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
semaforr_apply_project_options(semaforr_advisors)
semaforr_apply_strict_warnings(semaforr_advisors)

add_library(semaforr_spatial SHARED
  src/spatial/BarrierLearner.cpp
  src/spatial/ConveyorLearner.cpp
  src/spatial/DoorExitLearner.cpp
  src/spatial/HallwayLearner.cpp
  src/spatial/GridLearners.cpp
  src/spatial/HighwayLearner.cpp
  src/spatial/PassageSkeletonLearner.cpp
  src/spatial/RegionLearner.cpp
  src/spatial/SpatialLearner.cpp
  src/spatial/SpatialLearningCoordinator.cpp
  src/spatial/TrailLearner.cpp
)
set_target_properties(semaforr_spatial PROPERTIES EXPORT_NAME spatial)
add_library(semaforr::spatial ALIAS semaforr_spatial)
target_include_directories(semaforr_spatial
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
semaforr_apply_project_options(semaforr_spatial)
semaforr_apply_strict_warnings(semaforr_spatial)

add_library(semaforr_domain SHARED ${SEMAFORR_DOMAIN_SOURCES})
set_target_properties(semaforr_domain PROPERTIES EXPORT_NAME domain)
add_library(semaforr::domain ALIAS semaforr_domain)
target_include_directories(semaforr_domain
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
semaforr_apply_project_options(semaforr_domain)
semaforr_apply_strict_warnings(semaforr_domain)

target_link_libraries(semaforr_planning PUBLIC semaforr_domain)
target_link_libraries(semaforr_advisors PUBLIC semaforr_domain)
target_link_libraries(semaforr_spatial PUBLIC semaforr_domain)

add_library(semaforr_navigation SHARED ${SEMAFORR_NAVIGATION_SOURCES})
set_target_properties(semaforr_navigation PROPERTIES EXPORT_NAME navigation)
add_library(semaforr::navigation ALIAS semaforr_navigation)
target_include_directories(semaforr_navigation
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  semaforr_navigation
  PUBLIC
    semaforr_domain
    semaforr_planning
    semaforr_advisors
    semaforr_spatial
)
semaforr_apply_project_options(semaforr_navigation)
semaforr_apply_strict_warnings(semaforr_navigation)

add_library(semaforr_ros_adapters SHARED
  src/ros/CommandExecutor.cpp
  src/ros/MessageAdapters.cpp
  src/ros/NavigationEngineAdapter.cpp
  src/ros/ParameterConfiguration.cpp
  src/ros/SemaFORRNode.cpp
  src/ros/SensorSynchronizer.cpp
  src/ros/SocialObservationBuffer.cpp
  src/ros/VisualizationPublisher.cpp
)
set_target_properties(
  semaforr_ros_adapters PROPERTIES EXPORT_NAME ros_adapters)
add_library(semaforr::ros_adapters ALIAS semaforr_ros_adapters)
target_include_directories(semaforr_ros_adapters
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(semaforr_ros_adapters semaforr_navigation)
ament_target_dependencies(semaforr_ros_adapters
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  semaforr_msgs
  social_context_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
semaforr_apply_project_options(semaforr_ros_adapters)
semaforr_apply_strict_warnings(semaforr_ros_adapters)

add_executable(semaforr_node src/ros/semaforr_node.cpp)
target_link_libraries(
  semaforr_node
  semaforr_navigation
  semaforr_ros_adapters
)
ament_target_dependencies(semaforr_node
  rclcpp
)
semaforr_apply_project_options(semaforr_node)
semaforr_apply_strict_warnings(semaforr_node)

install(
  TARGETS
    semaforr_domain
    semaforr_planning
    semaforr_advisors
    semaforr_spatial
    semaforr_navigation
    semaforr_ros_adapters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
  DIRECTORY config docs launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/check_coverage.py
    scripts/check_source_quality.py
    scripts/compare_decision_traces.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/record_baseline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semaforr_record_baseline
)
install(
  PROGRAMS scripts/convert_legacy_config.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semaforr_convert_legacy_config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  add_executable(
    semaforr_configuration_test
    test/unit/configuration_smoke.cpp
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

  ament_add_gtest(
    semaforr_domain_types_test
    test/unit/domain_types_test.cpp
  )
  target_link_libraries(semaforr_domain_types_test semaforr_domain)
  semaforr_apply_project_options(semaforr_domain_types_test)

  ament_add_gtest(
    semaforr_decision_coordinator_test
    test/unit/decision_coordinator_test.cpp
  )
  target_link_libraries(semaforr_decision_coordinator_test semaforr_advisors)
  semaforr_apply_project_options(semaforr_decision_coordinator_test)

  ament_add_gtest(
    semaforr_decision_record_test
    test/unit/decision_record_test.cpp
  )
  target_link_libraries(semaforr_decision_record_test semaforr_domain)
  semaforr_apply_project_options(semaforr_decision_record_test)

  ament_add_gtest(
    semaforr_domain_planning_test
    test/unit/domain_planning_test.cpp
  )
  target_link_libraries(semaforr_domain_planning_test semaforr_planning)
  semaforr_apply_project_options(semaforr_domain_planning_test)

  ament_add_gtest(
    semaforr_map_parser_test
    test/unit/map_parser_test.cpp
  )
  target_link_libraries(semaforr_map_parser_test semaforr_planning)
  semaforr_apply_project_options(semaforr_map_parser_test)

  ament_add_gtest(
    semaforr_spatial_learning_test
    test/unit/spatial_learning_test.cpp
  )
  target_link_libraries(
    semaforr_spatial_learning_test semaforr_navigation)
  semaforr_apply_project_options(semaforr_spatial_learning_test)

  ament_add_gtest(
    semaforr_ros_execution_test
    test/unit/ros_execution_test.cpp
  )
  target_link_libraries(
    semaforr_ros_execution_test semaforr_ros_adapters)
  semaforr_apply_project_options(semaforr_ros_execution_test)

  ament_add_gtest(
    semaforr_social_navigation_test
    test/unit/social_navigation_test.cpp
  )
  target_link_libraries(
    semaforr_social_navigation_test semaforr_ros_adapters)
  semaforr_apply_project_options(semaforr_social_navigation_test)

  ament_add_gtest(
    semaforr_crowd_model_test
    test/unit/crowd_model_test.cpp
  )
  target_link_libraries(
    semaforr_crowd_model_test semaforr_advisors semaforr_planning)
  semaforr_apply_project_options(semaforr_crowd_model_test)

  ament_add_gtest(
    semaforr_navigation_behavior_test
    test/unit/navigation_behavior_test.cpp
  )
  target_link_libraries(
    semaforr_navigation_behavior_test semaforr_advisors)
  semaforr_apply_project_options(semaforr_navigation_behavior_test)

  ament_add_gtest(
    semaforr_component_strategy_test
    test/unit/component_strategy_test.cpp
  )
  target_link_libraries(
    semaforr_component_strategy_test semaforr_navigation)
  semaforr_apply_project_options(semaforr_component_strategy_test)

  ament_add_gtest(
    semaforr_navigation_scenario_test
    test/integration/navigation_scenario_test.cpp
  )
  target_link_libraries(
    semaforr_navigation_scenario_test semaforr_navigation)
  semaforr_apply_project_options(semaforr_navigation_scenario_test)

  ament_add_gtest(
    semaforr_phase34_navigation_test
    test/unit/phase34_navigation_test.cpp
  )
  target_link_libraries(
    semaforr_phase34_navigation_test semaforr_navigation)
  semaforr_apply_project_options(semaforr_phase34_navigation_test)

  ament_add_pytest_test(semaforr_source_contract_test
    test/contracts/test_source_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_build_contract_test
    test/contracts/test_build_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_baseline_recorder_contract_test
    test/contracts/test_baseline_recorder_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_domain_boundary_test
    test/contracts/test_domain_boundary.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_ownership_contract_test
    test/contracts/test_ownership_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_configuration_contract_test
    test/contracts/test_configuration_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_controller_decomposition_test
    test/contracts/test_controller_decomposition.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_decision_tier_interface_test
    test/contracts/test_decision_tier_interfaces.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_arbitration_contract_test
    test/contracts/test_arbitration_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_planning_geometry_contract_test
    test/contracts/test_planning_geometry_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_spatial_learning_contract_test
    test/contracts/test_spatial_learning_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_ros_node_contract_test
    test/contracts/test_ros_node_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_social_navigation_contract_test
    test/contracts/test_social_navigation_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_observability_contract_test
    test/contracts/test_observability_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_phase14_strategy_test
    test/contracts/test_phase14_strategy.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_phase15_deployment_test
    test/contracts/test_phase15_deployment.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  semaforr_msgs
  social_context_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

ament_package()
