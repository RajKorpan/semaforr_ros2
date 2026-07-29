import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def read(relative):
    return (SOURCE_DIR / relative).read_text(encoding="utf-8")


def test_ros_node_has_event_driven_state_machine():
    implementation = read("src/ros/SemaFORRNode.cpp")
    main = read("src/ros/semaforr_node.cpp")
    header = read("include/semaforr/ros/semaforr_node.hpp")

    for state in (
        "WaitingForSensors",
        "ReadyToDecide",
        "ExecutingAction",
        "Stopped",
    ):
        assert state in header
    assert "create_subscription" in implementation
    assert "rclcpp::create_timer" in implementation
    assert "add_pre_shutdown_callback" in main
    assert "spin_some" not in implementation
    assert "spin_some" not in main
    assert "while (rclcpp::ok())" not in implementation


def test_ros_responsibilities_are_separate_public_components():
    components = {
        "sensor_synchronizer.hpp": "class SensorSynchronizer",
        "command_executor.hpp": "class CommandExecutor",
        "navigation_engine_adapter.hpp": "class NavigationEngineAdapter",
        "visualization_publisher.hpp": "class VisualizationPublisher",
        "semaforr_node.hpp": "class SemaFORRNode",
    }
    for filename, declaration in components.items():
        header = read(f"include/semaforr/ros/{filename}")
        assert declaration in header

    cmake = read("CMakeLists.txt")
    for source in (
        "CommandExecutor.cpp",
        "NavigationEngineAdapter.cpp",
        "SensorSynchronizer.cpp",
        "VisualizationPublisher.cpp",
        "SemaFORRNode.cpp",
    ):
        assert f"src/ros/{source}" in cmake


def test_safety_time_frames_topics_and_qos_are_explicit():
    implementation = read("src/ros/SemaFORRNode.cpp")
    synchronizer = read("src/ros/SensorSynchronizer.cpp")
    executor = read("src/ros/CommandExecutor.cpp")
    yaml = read("config/semaforr.yaml")

    assert "publishZero(true)" in implementation
    assert "sensor_" in implementation
    assert "PoseStale" in synchronizer
    assert "ScanStale" in synchronizer
    assert "ClockReset" in synchronizer
    assert "transform_buffer_.transform" in implementation
    assert "odometry_reset" in yaml
    assert "Angle::normalize" in executor
    for section in ("topics:", "qos:", "frames:", "timing:", "command:"):
        assert section in yaml


def test_action_execution_has_typed_failure_states():
    header = read("include/semaforr/ros/command_executor.hpp")
    for status in (
        "Completed",
        "TimedOut",
        "OdometryReset",
        "ClockReset",
        "Cancelled",
    ):
        assert status in header
    assert "target_distance_m" in header
    assert "target_angle_rad" in header
    assert "linear_velocity_mps" in header
    assert "angular_velocity_radps" in header
