import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
NAVIGATION_DIR = SOURCE_DIR / "src" / "navigation"


def test_path_planner_implementation_is_grouped_by_responsibility():
    expected = {
        "PathPlanner.cpp": (
            "calcPath",
            "calcOrigPath",
            "updateNavGraph",
            "computeNewEdgeCost",
        ),
        "PathPlannerCosts.cpp": (
            "cellCost",
            "calcPathCost",
            "estimateCost",
        ),
        "PathPlannerQueries.cpp": (
            "getRemainingPathLength",
            "getClosestNode",
            "getClosestNodes",
        ),
        "PathPlannerSmoothing.cpp": (
            "smoothPath",
            "allWaypointsValid",
        ),
    }
    sources = {
        filename: (NAVIGATION_DIR / filename).read_text(encoding="utf-8")
        for filename in expected
    }
    for filename, methods in expected.items():
        for method in methods:
            definition = re.compile(rf"\bPathPlanner::{method}\s*\(")
            assert definition.search(sources[filename])
            for other_filename, source in sources.items():
                if other_filename != filename:
                    assert not definition.search(source)

    assert all(len(source.splitlines()) < 1100 for source in sources.values())


def test_geometry_uses_value_semantics_and_no_error_macro():
    header = (
        SOURCE_DIR / "include" / "semaforr" / "core" / "FORRGeometry.hpp"
    ).read_text(encoding="utf-8")

    assert "constexpr double kGeometryTolerance" in header
    assert "#define ERROR" not in header
    assert "operator=(const CartesianPoint& other) = default" in header
    assert "operator=(const Line& other) = default" in header
    assert "bool is_degenerate() const" in header


def test_planner_state_and_empty_trails_have_safe_defaults():
    header = (
        SOURCE_DIR / "include" / "semaforr" / "navigation" / "PathPlanner.hpp"
    ).read_text(encoding="utf-8")

    for initialization in (
        "double pathCost = 0.0",
        "double origPathCost = 0.0",
        "FORRConveyors* conveyors = nullptr",
        "bool objectiveSet = false",
        "bool pathCalculated = false",
    ):
        assert initialization in header

    assert "if (trl[i].empty())" in header
