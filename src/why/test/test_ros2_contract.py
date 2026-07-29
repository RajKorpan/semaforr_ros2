import os
from pathlib import Path


SOURCE_ROOT = Path(os.environ['WHY_SOURCE_DIR'])


def test_explanation_node_uses_ros2_and_installed_configuration():
    source = (SOURCE_ROOT / 'src' / 'main.cpp').read_text(encoding='utf-8')

    assert '#include <rclcpp/rclcpp.hpp>' in source
    assert 'create_subscription<std_msgs::msg::String>' in source
    assert 'ament_index_cpp::get_package_share_directory("why")' in source
    assert 'declare_parameter<string>("decision_log_topic"' in source
    assert 'fields.size() < 17' in source
    assert 'validateConfigGroup(vstrings, 2' in source
    assert 'ros::' not in source
    assert '#include <ros/' not in source


def test_runtime_text_configuration_is_installed():
    cmake = (SOURCE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert 'install(DIRECTORY config DESTINATION share/${PROJECT_NAME})' in cmake
    assert (SOURCE_ROOT / 'config' / 'text.conf').is_file()
