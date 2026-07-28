import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[1])
)

DOMAIN_HEADER_AREAS = (
    "config",
    "core",
    "domain",
    "decision",
    "exploration",
    "navigation",
    "spatial",
    "vendor",
)
DOMAIN_SOURCE_AREAS = (
    "config",
    "core",
    "decision",
    "navigation",
    "spatial",
    "vendor",
)
ROS_TOKENS = (
    "rclcpp::",
    "geometry_msgs::",
    "sensor_msgs::",
    "nav_msgs::",
    "visualization_msgs::",
    "std_msgs::",
    "tf2::",
    "semaforr::msg::",
)
ROS_INCLUDE_PATTERN = re.compile(
    r"#\s*include\s*[<\"]"
    r"(?:rclcpp|geometry_msgs|sensor_msgs|nav_msgs|visualization_msgs|"
    r"std_msgs|tf2|semaforr/msg)"
)


def without_comments(source):
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return "\n".join(line.split("//", 1)[0] for line in source.splitlines())


def domain_files():
    include_root = SOURCE_DIR / "include" / "semaforr"
    source_root = SOURCE_DIR / "src"
    for area in DOMAIN_HEADER_AREAS:
        yield from (include_root / area).rglob("*.h")
    for area in DOMAIN_SOURCE_AREAS:
        yield from (source_root / area).rglob("*.cpp")


def test_domain_code_has_no_ros_dependencies():
    violations = []
    for path in domain_files():
        source = without_comments(path.read_text(encoding="utf-8"))
        if ROS_INCLUDE_PATTERN.search(source):
            violations.append(f"{path}: ROS include")
        for token in ROS_TOKENS:
            if token in source:
                violations.append(f"{path}: {token}")

    assert not violations, "\n".join(violations)


def test_domain_target_has_no_ros_linkage():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    target = re.search(
        r"add_library\(semaforr_domain SHARED.*?"
        r"add_library\(semaforr_core INTERFACE\)",
        cmake,
        re.DOTALL,
    )

    assert target is not None
    target_definition = target.group(0)
    assert "ament_target_dependencies(semaforr_domain" not in target_definition
    assert "cpp_typesupport_target" not in target_definition
    for dependency in (
        "geometry_msgs",
        "nav_msgs",
        "rclcpp",
        "sensor_msgs",
        "std_msgs",
        "tf2",
        "visualization_msgs",
    ):
        assert dependency not in target_definition
