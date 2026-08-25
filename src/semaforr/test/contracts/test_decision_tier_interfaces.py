"""SemaFORR module overview.

Summary:
    This file exercises test decision tier interfaces behavior for automated verification and regression testing. It centers on `read`, `test_tier_interfaces_are_typed_and_value_returning`, `test_registries_reject_unknown_component_names`. Its package-relative location is `test/contracts/test_decision_tier_interfaces.py`.

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


def test_tier_interfaces_are_typed_and_value_returning():
    """Summary:
        Performs the test tier interfaces are typed and value returning operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    rules = read("include/semaforr/decision/rules.hpp")
    advisor = read("include/semaforr/decision/advisor.hpp")
    planner = read("include/semaforr/planning/planner.hpp")
    assert "std::optional<Decision>" in rules
    assert "std::vector<Veto>" in rules
    assert "AdvisorEvaluation evaluate" in advisor
    assert "std::span<const domain::Action>" in advisor
    assert "PlanResult plan" in planner
    assert "PlanStatus" in planner
    assert "FORRAction" not in rules + advisor + planner


def test_registries_reject_unknown_component_names():
    """Summary:
        Performs the test registries reject unknown component names operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    registry = read("include/semaforr/decision/registry.hpp")
    planning = read("src/ros/parameter_configuration.cpp")
    assert "throw std::invalid_argument" in registry
    assert "unknown planner name" in planning.lower()
