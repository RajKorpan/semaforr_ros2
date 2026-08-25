"""SemaFORR module overview.

Summary:
    This file exercises test domain boundary behavior for automated verification and regression testing. It centers on `test_non_ros_components_have_no_ros_dependencies`, `test_ros_messages_are_converted_only_in_ros_adapter_layer`. Its package-relative location is `test/contracts/test_domain_boundary.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""









import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)

ROS_PATTERN = re.compile(
    r"(#\s*include\s*[<\"](?:rclcpp|geometry_msgs|sensor_msgs|nav_msgs|"
    r"visualization_msgs|std_msgs|tf2|semaforr_msgs|social_context_msgs)|"
    r"\b(?:rclcpp|geometry_msgs|sensor_msgs|nav_msgs|visualization_msgs|"
    r"std_msgs|tf2|semaforr_msgs|social_context_msgs)::)"
)


def test_non_ros_components_have_no_ros_dependencies():
    """Summary:
        Performs the test non ros components have no ros dependencies operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    violations = []
    roots = [
        SOURCE_DIR / "include" / "semaforr" / area
        for area in ("config", "domain", "decision", "planning", "social", "spatial")
    ] + [
        SOURCE_DIR / "src" / area
        for area in ("config", "domain", "decision", "navigation", "social", "spatial")
    ]
    for root in roots:
        for path in root.rglob("*"):
            if path.suffix not in {".hpp", ".cpp"}:
                continue
            if ROS_PATTERN.search(path.read_text(encoding="utf-8")):
                violations.append(path.relative_to(SOURCE_DIR).as_posix())
    assert not violations


def test_ros_messages_are_converted_only_in_ros_adapter_layer():
    """Summary:
        Performs the test ros messages are converted only in ros adapter layer operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    non_ros = "\n".join(
        path.read_text(encoding="utf-8")
        for root in (SOURCE_DIR / "include", SOURCE_DIR / "src")
        for path in root.rglob("*")
        if path.suffix in {".hpp", ".cpp"} and "/ros/" not in path.as_posix()
    )
    assert "social_context_msgs::msg::SocialObservation" not in non_ros
    assert "sensor_msgs::msg::LaserScan" not in non_ros
