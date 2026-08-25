"""SemaFORR module overview.

Summary:
    This file exercises test controller decomposition behavior for automated verification and regression testing. It centers on `read`, `test_navigation_engine_composes_focused_collaborators`, `test_ros_adapter_owns_domain_composition_not_legacy_controller`, `test_removed_controller_files_do_not_exist`. Its package-relative location is `test/contracts/test_controller_decomposition.py`.

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


def test_navigation_engine_composes_focused_collaborators():
    """Summary:
        Performs the test navigation engine composes focused collaborators operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    header = read("include/semaforr/decision/navigation_engine.hpp")
    for collaborator in (
        "DecisionCoordinator&",
        "MissionManager&",
        "planning::PlanningCoordinator&",
        "spatial::SpatialLearningCoordinator&",
    ):
        assert collaborator in header
    assert "class Controller" not in header
    assert "Controller&" not in header


def test_ros_adapter_owns_domain_composition_not_legacy_controller():
    """Summary:
        Performs the test ros adapter owns domain composition not legacy controller operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    source = read("src/ros/navigation_engine_adapter.cpp")
    for component in (
        "domain::WorldModel",
        "decision::DecisionCoordinator",
        "decision::MissionManager",
        "planning::PlanningCoordinator",
        "spatial::SpatialLearningCoordinator",
        "decision::NavigationEngine",
    ):
        assert component in source
    assert "class Controller" not in source
    assert "Controller controller_" not in source


def test_removed_controller_files_do_not_exist():
    """Summary:
        Performs the test removed controller files do not exist operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    assert not list((SOURCE_DIR / "src" / "decision").glob("Controller*.cpp"))
    assert not (SOURCE_DIR / "include/semaforr/decision/Controller.hpp").exists()
    assert not (SOURCE_DIR / "include/semaforr/decision/AgentState.hpp").exists()
