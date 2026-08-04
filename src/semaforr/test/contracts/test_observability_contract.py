import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def read(relative):
    return (SOURCE_DIR / relative).read_text(encoding="utf-8")


def test_one_decision_result_contains_complete_explanation():
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
    source = read("src/ros/visualization_publisher.cpp")
    assert "semaforr_msgs::msg::DecisionRecord" in source
    assert "raw_score" in source
    assert "weighted_score" in source
    assert "decision_latency_s" in source


def test_node_uses_ros_logging_levels():
    source = (
        read("src/ros/semaforr_node_component.cpp")
        + read("src/ros/visualization_publisher.cpp")
    )
    for level in ("RCLCPP_DEBUG", "RCLCPP_INFO", "RCLCPP_WARN", "RCLCPP_ERROR"):
        assert level in source
