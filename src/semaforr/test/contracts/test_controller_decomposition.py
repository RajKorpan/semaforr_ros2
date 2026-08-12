import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def read(relative):
    return (SOURCE_DIR / relative).read_text(encoding="utf-8")


def test_navigation_engine_composes_focused_collaborators():
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
    assert not list((SOURCE_DIR / "src" / "decision").glob("Controller*.cpp"))
    assert not (SOURCE_DIR / "include/semaforr/decision/Controller.hpp").exists()
    assert not (SOURCE_DIR / "include/semaforr/decision/AgentState.hpp").exists()
