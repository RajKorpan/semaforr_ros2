import os
from pathlib import Path


SOURCE_ROOT = Path(os.environ['WHY_PLAN_SOURCE_DIR'])


def test_plan_explanation_consumes_the_typed_decision_record():
    source = (SOURCE_ROOT / 'src' / 'main.cpp').read_text(encoding='utf-8')

    assert '#include <semaforr_msgs/msg/decision_record.hpp>' in source
    assert 'create_subscription<DecisionRecord>' in source
    assert 'record.has_planner' in source
    assert 'record.selected_planner' in source
    assert 'plannerObjective' in source
    assert 'outcomeText' in source
    assert 'DecisionRecord::OUTCOME_SENSOR_LOST' in source
    assert 'rclcpp::spin(std::make_shared<PlanExplanationNode>())' in source
    assert 'create_subscription<std_msgs::msg::String>' not in source
    assert 'OccupancyGrid' not in source
    assert 'spin_some' not in source
    assert 'decision_log' not in source
    assert 'cout' not in source


def test_package_requires_the_stable_message_api_and_strict_warnings():
    cmake = (SOURCE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    manifest = (SOURCE_ROOT / 'package.xml').read_text(encoding='utf-8')

    assert 'find_package(semaforr_msgs REQUIRED)' in cmake
    assert '-Wall -Wextra -Wpedantic -Werror' in cmake
    assert '<depend>semaforr_msgs</depend>' in manifest
    assert 'install(DIRECTORY config' not in cmake
