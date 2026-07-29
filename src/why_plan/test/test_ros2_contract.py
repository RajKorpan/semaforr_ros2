import os
from pathlib import Path


SOURCE_ROOT = Path(os.environ['WHY_PLAN_SOURCE_DIR'])


def test_plan_explanation_node_uses_ros2_and_validates_diagnostics():
    source = (SOURCE_ROOT / 'src' / 'main.cpp').read_text(encoding='utf-8')

    assert '#include <rclcpp/rclcpp.hpp>' in source
    assert 'create_subscription<nav_msgs::msg::OccupancyGrid>' in source
    assert 'ament_index_cpp::get_package_share_directory("why_plan")' in source
    assert 'declare_parameter<string>("crowd_density_topic"' in source
    assert 'fields.size() <= 25' in source
    assert 'validPlanField(fields[16])' in source
    assert 'alternative_planners.empty()' in source
    assert 'grid.info.origin.position.x' in source
    assert 'grid.info.resolution' in source
    assert 'ros::' not in source
    assert '#include <ros/' not in source


def test_runtime_text_configuration_is_installed():
    cmake = (SOURCE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert 'install(DIRECTORY config DESTINATION share/${PROJECT_NAME})' in cmake
    assert (SOURCE_ROOT / 'config' / 'text.conf').is_file()
