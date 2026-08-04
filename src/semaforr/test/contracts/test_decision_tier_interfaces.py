import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def read(relative):
    return (SOURCE_DIR / relative).read_text(encoding="utf-8")


def test_tier_interfaces_are_typed_and_value_returning():
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
    registry = read("include/semaforr/decision/registry.hpp")
    planning = read("src/ros/parameter_configuration.cpp")
    assert "throw std::invalid_argument" in registry
    assert "unknown planner name" in planning.lower()
