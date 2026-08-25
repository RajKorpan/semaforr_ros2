"""SemaFORR module overview.

Summary:
    This file exercises test observability contract behavior for automated verification and regression testing. It centers on `read`, `test_one_decision_result_contains_complete_explanation`, `test_ros_publisher_serializes_structured_decision_record`, `test_node_uses_ros_logging_levels`. Its package-relative location is `test/contracts/test_observability_contract.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""









import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def read(relative):
    """Summary:
        Reads package content for this subsystem.

    Args:
        relative (Any): Supplies relative input to the operation.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    return (SOURCE_DIR / relative).read_text(encoding="utf-8")


def test_one_decision_result_contains_complete_explanation():
    """Summary:
        Performs the test one decision result contains complete explanation operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    header = read("include/semaforr/decision/decision_result.hpp")
    for field in (
        "sequence",
        "robot_pose",
        "task",
        "candidates",
        "vetoes",
        "contributions",
        "source",
        "planner",
        "action",
        "decision_latency_s",
        "action_outcome",
    ):
        assert field in header
    assert "DecisionTier" in header


def test_ros_publisher_serializes_structured_decision_record():
    """Summary:
        Performs the test ros publisher serializes structured decision record operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    source = read("src/ros/visualization_publisher.cpp")
    assert "semaforr_msgs::msg::DecisionRecord" in source
    assert "raw_score" in source
    assert "weighted_score" in source
    assert "decision_latency_s" in source


def test_node_uses_ros_logging_levels():
    """Summary:
        Performs the test node uses ros logging levels operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    source = (
        read("src/ros/semaforr_node_component.cpp")
        + read("src/ros/visualization_publisher.cpp")
    )
    for level in ("RCLCPP_DEBUG", "RCLCPP_INFO", "RCLCPP_WARN", "RCLCPP_ERROR"):
        assert level in source
