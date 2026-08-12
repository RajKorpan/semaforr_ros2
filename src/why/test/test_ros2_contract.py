import os
from pathlib import Path


SOURCE_ROOT = Path(os.environ['WHY_SOURCE_DIR'])


def test_unified_explanation_is_request_driven_and_retains_typed_records():
    source = (SOURCE_ROOT / 'src' / 'main.cpp').read_text(encoding='utf-8')

    assert '#include <semaforr_msgs/msg/decision_record.hpp>' in source
    assert '#include <semaforr_msgs/msg/explanation_question.hpp>' in source
    assert '#include <semaforr_msgs/msg/explanation_response.hpp>' in source
    assert 'why_.record(*record)' in source
    assert 'why_.answer(*question)' in source
    assert 'rclcpp::spin(std::make_shared<WhyNode>())' in source
    assert 'std_msgs::msg::String' not in source
    assert 'spin_some' not in source
    assert 'cout' not in source


def test_package_requires_the_stable_message_api_and_strict_warnings():
    cmake = (SOURCE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    manifest = (SOURCE_ROOT / 'package.xml').read_text(encoding='utf-8')

    assert 'find_package(semaforr_msgs REQUIRED)' in cmake
    assert '-Wall -Wextra -Wpedantic -Werror' in cmake
    assert '<depend>semaforr_msgs</depend>' in manifest
    assert 'add_library(why_core' in cmake
    assert 'ament_add_gtest(why_system_test' in cmake
    assert 'install(DIRECTORY config' not in cmake
