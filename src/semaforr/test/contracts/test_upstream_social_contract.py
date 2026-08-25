"""SemaFORR module overview.

Summary:
    This file exercises test upstream social contract behavior for automated verification and regression testing. It centers on `test_consumed_upstream_interfaces_are_exactly_versioned`, `test_upstream_topics_match_collaborator_nodes`, `test_semaforr_does_not_duplicate_upstream_interfaces`, `test_crowd_field_is_internal_not_an_upstream_ros_interface`. Its package-relative location is `test/contracts/test_upstream_social_contract.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""









import hashlib
import json
import os
from pathlib import Path


PACKAGE_ROOT = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
CONTRACT = json.loads(
    (PACKAGE_ROOT / "docs" / "upstream-social-contract.json").read_text(
        encoding="utf-8"
    )
)


def test_consumed_upstream_interfaces_are_exactly_versioned():
    """Summary:
        Performs the test consumed upstream interfaces are exactly versioned operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    for interface in CONTRACT["interfaces"]:
        path = WORKSPACE_ROOT / interface["path"]
        assert path.is_file(), f"missing upstream interface: {path}"
        assert hashlib.sha256(path.read_bytes()).hexdigest() == interface["sha256"]


def test_upstream_topics_match_collaborator_nodes():
    """Summary:
        Performs the test upstream topics match collaborator nodes operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    tracked = (WORKSPACE_ROOT / "src/social_context/social_context/social_context_tracked.py").read_text(encoding="utf-8")
    hunav = (WORKSPACE_ROOT / "src/social_context/social_context/social_context_hunav.py").read_text(encoding="utf-8")
    formation = (WORKSPACE_ROOT / "src/social_context/social_context/pose_estimation/src/formation/formation_node.py").read_text(encoding="utf-8")
    topics = CONTRACT["topics"]
    assert topics["tracked_current"] in tracked
    assert topics["tracked_predictions"] in tracked
    assert topics["hunav_current"] in hunav
    assert topics["hunav_predictions"] in hunav
    assert topics["formations"] in formation


def test_semaforr_does_not_duplicate_upstream_interfaces():
    """Summary:
        Performs the test semaforr does not duplicate upstream interfaces operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    forbidden = {Path(value["path"]).name for value in CONTRACT["interfaces"]}
    duplicates = [path for path in PACKAGE_ROOT.rglob("*.msg") if path.name in forbidden]
    assert not duplicates, f"SemaFORR duplicates upstream messages: {duplicates}"


def test_crowd_field_is_internal_not_an_upstream_ros_interface():
    """Summary:
        Performs the test crowd field is internal not an upstream ros interface operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    message_root = WORKSPACE_ROOT / "src/social_context_msgs/msg"
    assert not (message_root / "CrowdField.msg").exists()
    assert not (message_root / "CrowdFieldCell.msg").exists()
    source = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for path in PACKAGE_ROOT.rglob("*")
        if path.is_file() and path.suffix in {".cpp", ".hpp"}
    )
    assert "social_context_msgs/msg/crowd_field.hpp" not in source
    assert "social_context_msgs::msg::CrowdField" not in source
