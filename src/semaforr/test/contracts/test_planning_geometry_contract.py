import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def read(relative):
    return (SOURCE_DIR / relative).read_text(encoding="utf-8")


def test_geometry_uses_typed_metric_values_and_one_angle_normalizer():
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
    graph = read("include/semaforr/planning/graph.hpp")
    astar = read("src/navigation/DomainAStar.cpp")
    assert "struct GraphEdge" in graph
    assert "mutable" not in graph
    assert "std::priority_queue" in astar
    assert "PathResult" in astar


def test_typed_planner_uses_domain_crowd_and_spatial_models():
    planner = read("include/semaforr/planning/planner.hpp")
    implementation = read("src/navigation/DomainPlanner.cpp")
    assert "const domain::SpatialModel*" in planner
    assert "const domain::CrowdModel*" in planner
    assert "PlannerObjective" in read("include/semaforr/planning/domain_planner.hpp")
    assert "socialPenalty" in implementation
    assert "PathPlanner" not in implementation


def test_legacy_geometry_and_planner_are_removed():
    assert not (SOURCE_DIR / "include/semaforr/core/FORRGeometry.hpp").exists()
    assert not (SOURCE_DIR / "include/semaforr/navigation/PathPlanner.hpp").exists()
