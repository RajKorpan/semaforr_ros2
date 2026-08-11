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
find_package(ament_index_cpp REQUIRED)
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
  src/config/navigation_configuration.cpp
  src/domain/circumstance.cpp
  src/domain/action_execution.cpp
  src/domain/model_revision.cpp
  src/domain/crowd_model.cpp
  src/domain/grid_geometry.cpp
  src/domain/grid_layers.cpp
  src/domain/motion_model.cpp
  src/decision/decision_result.cpp
  src/social/crowd_field_learner.cpp
  src/spatial/coverage.cpp
)

set(SEMAFORR_NAVIGATION_SOURCES
  src/decision/mission_manager.cpp
  src/decision/navigation_engine.cpp
  src/navigation/navigation_phase.cpp
)

add_library(semaforr_planning SHARED
  src/planning/domain_astar.cpp
  src/planning/domain_graph.cpp
  src/planning/domain_planner.cpp
  src/planning/hierarchical_plan.cpp
  src/planning/map_parser.cpp
  src/planning/static_map_loader.cpp
  src/planning/traversability.cpp
  src/planning/plan_types.cpp
  src/planning/planning_coordinator.cpp
  src/planning/planner_registry.cpp
  src/planning/reactive_planner.cpp
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
  src/decision/advisors/catalog_registry.cpp
  src/decision/decision_coordinator.cpp
  src/decision/advisors/heuristic_advisor.cpp
  src/decision/plan_enforcer.cpp
  src/decision/hard_safety_filter.cpp
  src/decision/advisors/social/learned_crowd_advisor.cpp
  src/decision/advisors/navigation_advisor.cpp
  src/decision/obstacle_veto_rule.cpp
  src/decision/tier_registry.cpp
  src/decision/advisors/social/registry.cpp
  src/decision/advisors/social/social_navigation_advisor.cpp
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
  src/spatial/barrier_learner.cpp
  src/spatial/conveyor_learner.cpp
  src/spatial/circumstance_learner.cpp
  src/spatial/door_exit_learner.cpp
  src/spatial/hallway_learner.cpp
  src/spatial/grid_learners.cpp
  src/spatial/highway_learner.cpp
  src/spatial/passage_skeleton_learner.cpp
  src/spatial/region_learner.cpp
  src/spatial/learner.cpp
  src/spatial/spatial_learning_coordinator.cpp
  src/spatial/trail_learner.cpp
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
target_link_libraries(semaforr_advisors PUBLIC semaforr_domain semaforr_planning)
target_link_libraries(semaforr_spatial PUBLIC semaforr_domain)

add_library(semaforr_validation SHARED
  src/validation/allocation_probe.cpp
  src/validation/experiment_metrics.cpp
)
set_target_properties(semaforr_validation PROPERTIES EXPORT_NAME validation)
add_library(semaforr::validation ALIAS semaforr_validation)
target_include_directories(semaforr_validation
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(semaforr_validation PUBLIC semaforr_domain)
semaforr_apply_project_options(semaforr_validation)
semaforr_apply_strict_warnings(semaforr_validation)

add_library(semaforr_exploration SHARED
  src/exploration/exploration_coordinator.cpp
  src/exploration/high_level_explorer.cpp
  src/exploration/highway_explorer.cpp
)
set_target_properties(semaforr_exploration PROPERTIES EXPORT_NAME exploration)
add_library(semaforr::exploration ALIAS semaforr_exploration)
target_include_directories(semaforr_exploration
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  semaforr_exploration PUBLIC semaforr_domain semaforr_planning semaforr_spatial)
semaforr_apply_project_options(semaforr_exploration)
semaforr_apply_strict_warnings(semaforr_exploration)

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
    semaforr_exploration
    semaforr_advisors
    semaforr_spatial
)
semaforr_apply_project_options(semaforr_navigation)
semaforr_apply_strict_warnings(semaforr_navigation)

add_library(semaforr_ros_adapters SHARED
  src/ros/command_executor.cpp
  src/ros/message_adapters.cpp
  src/ros/navigation_engine_adapter.cpp
  src/ros/parameter_configuration.cpp
  src/ros/semaforr_node_component.cpp
  src/ros/sensor_synchronizer.cpp
  src/ros/social_observation_buffer.cpp
  src/ros/visualization_publisher.cpp
)
set_target_properties(
  semaforr_ros_adapters PROPERTIES EXPORT_NAME ros_adapters)
add_library(semaforr::ros_adapters ALIAS semaforr_ros_adapters)
target_include_directories(semaforr_ros_adapters
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  semaforr_ros_adapters semaforr_navigation semaforr_validation)
ament_target_dependencies(semaforr_ros_adapters
  ament_index_cpp
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

add_executable(semaforr_node src/ros/main.cpp)
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
    semaforr_exploration
    semaforr_advisors
    semaforr_spatial
    semaforr_navigation
    semaforr_ros_adapters
    semaforr_validation
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
    scripts/run_experiment_matrix.py
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
    semaforr_experiment_metrics_test
    test/unit/experiment_metrics_test.cpp
  )
  target_link_libraries(
    semaforr_experiment_metrics_test semaforr_validation)
  semaforr_apply_project_options(semaforr_experiment_metrics_test)

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
  target_compile_definitions(semaforr_map_parser_test PRIVATE
    "SEMAFORR_TEST_WORKSPACE_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}/../..\""
  )
  semaforr_apply_project_options(semaforr_map_parser_test)

  ament_add_gtest(
    semaforr_map_runtime_test
    test/integration/map_runtime_test.cpp
  )
  target_link_libraries(semaforr_map_runtime_test semaforr_ros_adapters)
  target_compile_definitions(semaforr_map_runtime_test PRIVATE
    "SEMAFORR_TEST_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\""
  )
  semaforr_apply_project_options(semaforr_map_runtime_test)

  ament_add_gtest(
    semaforr_spatial_learning_test
    test/unit/spatial_learning_test.cpp
  )
  target_link_libraries(
    semaforr_spatial_learning_test semaforr_navigation)
  semaforr_apply_project_options(semaforr_spatial_learning_test)

  ament_add_gtest(
    semaforr_grid_layers_test
    test/unit/grid_layers_test.cpp
  )
  target_link_libraries(
    semaforr_grid_layers_test
    semaforr_exploration semaforr_spatial semaforr_planning)
  semaforr_apply_project_options(semaforr_grid_layers_test)

  ament_add_gtest(
    semaforr_grid_geometry_test
    test/unit/grid_geometry_test.cpp
  )
  target_link_libraries(semaforr_grid_geometry_test semaforr_domain)
  semaforr_apply_project_options(semaforr_grid_geometry_test)

  ament_add_gtest(
    semaforr_ros_execution_test
    test/unit/ros_execution_test.cpp
  )
  target_link_libraries(
    semaforr_ros_execution_test semaforr_ros_adapters)
  semaforr_apply_project_options(semaforr_ros_execution_test)

  ament_add_gtest(
    semaforr_action_execution_lifecycle_test
    test/unit/action_execution_lifecycle_test.cpp
  )
  target_link_libraries(
    semaforr_action_execution_lifecycle_test semaforr_navigation)
  semaforr_apply_project_options(semaforr_action_execution_lifecycle_test)

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
    semaforr_exploration_planning_test
    test/unit/exploration_planning_test.cpp
  )
  target_link_libraries(
    semaforr_exploration_planning_test semaforr_navigation)
  semaforr_apply_project_options(semaforr_exploration_planning_test)

  ament_add_gtest(
    semaforr_revision_dependency_test
    test/unit/revision_dependency_test.cpp
  )
  target_link_libraries(
    semaforr_revision_dependency_test semaforr_navigation)
  semaforr_apply_project_options(semaforr_revision_dependency_test)

  ament_add_gtest(
    semaforr_snapshot_projection_test
    test/unit/snapshot_projection_test.cpp
  )
  target_link_libraries(
    semaforr_snapshot_projection_test
    semaforr_navigation semaforr_validation)
  semaforr_apply_project_options(semaforr_snapshot_projection_test)

  ament_add_gtest(
    semaforr_tier_advisor_test
    test/unit/tier_advisor_test.cpp
  )
  target_link_libraries(
    semaforr_tier_advisor_test
    semaforr_navigation semaforr_advisors)
  semaforr_apply_project_options(semaforr_tier_advisor_test)

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
  ament_add_pytest_test(semaforr_navigation_strategy_test
    test/contracts/test_navigation_strategy.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_deployment_test
    test/contracts/test_deployment.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_compatibility_contract_test
    test/contracts/test_compatibility_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_map_modes_contract_test
    test/contracts/test_map_modes_contract.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(semaforr_simulator_map_test
    test/unit/test_simulator_map.py
    ENV "SEMAFORR_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )

  get_property(semaforr_registered_tests DIRECTORY PROPERTY TESTS)
  foreach(semaforr_test_name IN LISTS semaforr_registered_tests)
    if(semaforr_test_name STREQUAL "semaforr_compatibility_contract_test")
      set_property(TEST ${semaforr_test_name} APPEND PROPERTY
        LABELS "behavior_mode:compatibility-contract")
    else()
      set_property(TEST ${semaforr_test_name} APPEND PROPERTY
        LABELS "behavior_mode:modernized")
    endif()
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
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
