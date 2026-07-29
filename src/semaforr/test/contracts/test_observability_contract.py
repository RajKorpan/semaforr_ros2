import os
import re
from pathlib import Path


ROOT = Path(os.environ["SEMAFORR_SOURCE_DIR"])


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_decision_record_is_structured_and_ros_independent():
    header = read("include/semaforr/decision/decision_result.hpp")
    assert "enum class DecisionTier" in header
    assert "enum class ActionOutcome" in header
    for field in (
        "sequence",
        "robot_pose",
        "task",
        "candidates",
        "vetoes",
        "contributions",
        "planner",
        "decision_latency_s",
        "action_outcome",
    ):
        assert field in header
    assert "legacy_tier" not in header
    assert "std_msgs" not in header


def test_ros_boundary_uses_typed_messages_not_encoded_strings():
    node = read("src/ros/SemaFORRNode.cpp")
    visualizer = read("include/semaforr/ros/Visualizer.hpp")
    publisher = read("src/ros/VisualizationPublisher.cpp")
    assert "semaforr_msgs::msg::NavigationState" in node
    assert "semaforr_msgs::msg::DecisionRecord" in publisher
    assert "std_msgs::msg::String" not in node
    assert "message.data +=" not in node
    assert "decision_log" not in visualizer
    assert "publish_log" not in visualizer


def test_decision_pipeline_has_no_numeric_tier_side_channel():
    controller = (
        read("src/decision/ControllerMission.cpp")
        + read("src/decision/ControllerDecision.cpp")
    )
    tier_one = read("include/semaforr/decision/DecisionTier.hpp")
    stats = read("include/semaforr/core/FORRActionStats.hpp")
    assert "decisionTier" not in controller
    assert "decisionTier" not in stats
    assert "decision_tier" not in tier_one
    assert "DecisionTier::" in controller
    assert "DecisionTier::SafeStop" in controller
    assert '"no_advisor_score"' in controller


def test_runtime_diagnostics_use_ros_log_levels():
    node = read("src/ros/SemaFORRNode.cpp")
    publisher = read("src/ros/VisualizationPublisher.cpp")
    main = read("src/ros/semaforr_node.cpp")
    assert "RCLCPP_DEBUG" in publisher
    assert "RCLCPP_INFO" in publisher
    assert "RCLCPP_WARN" in node
    assert "RCLCPP_ERROR" in main


def test_runtime_sources_do_not_write_directly_to_console():
    direct_console = re.compile(
        r"^\s*(?:std::)?cout\b|^\s*std::cerr\b", re.MULTILINE
    )
    for directory in ("src", "include/semaforr"):
        for suffix in ("*.cpp", "*.hpp"):
            for source in (ROOT / directory).rglob(suffix):
                assert not direct_console.search(
                    source.read_text(encoding="utf-8")
                ), f"direct console output remains in {source}"
