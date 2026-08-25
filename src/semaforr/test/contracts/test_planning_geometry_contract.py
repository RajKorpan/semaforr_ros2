"""SemaFORR module overview.

Summary:
    This file exercises test planning geometry contract behavior for automated verification and regression testing. It centers on `read`, `test_geometry_uses_typed_metric_values_and_one_angle_normalizer`, `test_graph_storage_is_separate_from_astar_search_state`, `test_typed_planner_uses_domain_crowd_and_spatial_models`, `test_legacy_geometry_and_planner_are_removed`. Its package-relative location is `test/contracts/test_planning_geometry_contract.py`.

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


def test_geometry_uses_typed_metric_values_and_one_angle_normalizer():
    """Summary:
        Performs the test geometry uses typed metric values and one angle normalizer operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    geometry = read("include/semaforr/domain/geometry.hpp")
    assert "class Distance" in geometry
    assert "class Angle" in geometry
    assert "struct Point2D" in geometry
    assert "struct Segment2D" in geometry
    assert "struct Circle" in geometry
    assert "class Polygon" in geometry
    assert "static double normalize" in geometry
    assert "* 100" not in geometry


def test_graph_storage_is_separate_from_astar_search_state():
    """Summary:
        Performs the test graph storage is separate from astar search state operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    graph = read("include/semaforr/planning/graph.hpp")
    astar = read("src/planning/domain_astar.cpp")
    assert "struct GraphEdge" in graph
    assert "mutable" not in graph
    assert "std::priority_queue" in astar
    assert "PathResult" in astar


def test_typed_planner_uses_domain_crowd_and_spatial_models():
    """Summary:
        Performs the test typed planner uses domain crowd and spatial models operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    planner = read("include/semaforr/planning/planner.hpp")
    implementation = read("src/planning/domain_planner.cpp")
    assert "const domain::SpatialModel*" in planner
    assert "const domain::CrowdModel*" in planner
    assert "PlannerObjective" in read("include/semaforr/planning/domain_planner.hpp")
    assert "socialPenalty" in implementation
    assert "PathPlanner" not in implementation


def test_legacy_geometry_and_planner_are_removed():
    """Summary:
        Performs the test legacy geometry and planner are removed operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    assert not (SOURCE_DIR / "include/semaforr/core/FORRGeometry.hpp").exists()
    assert not (SOURCE_DIR / "include/semaforr/navigation/PathPlanner.hpp").exists()
