import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
INCLUDE_DIR = SOURCE_DIR / "include" / "semaforr" / "decision"
DECISION_DIR = SOURCE_DIR / "src" / "decision"


def test_each_decision_tier_has_a_value_returning_interface():
    header = (INCLUDE_DIR / "DecisionTier.hpp").read_text(encoding="utf-8")

    expected_interfaces = {
        "TierOneDecision": "virtual TierOneResult decide() = 0",
        "TierTwoDecision": (
            "virtual TierTwoResult plan(Position current, "
            "bool select_next_task) = 0"
        ),
        "TierThreeDecision": "virtual TierThreeResult decide() = 0",
    }
    for interface, operation in expected_interfaces.items():
        assert f"class {interface}" in header
        assert f"virtual ~{interface}() = default" in header
        assert operation in header

    assert "FORRAction*" not in header
    assert "FORRAction *" not in header
    assert "Dependencies" not in header
    assert "makeTier" not in header


def test_controller_depends_on_interfaces_not_tier_implementations():
    header = (INCLUDE_DIR / "Controller.hpp").read_text(encoding="utf-8")
    decision_source = (
        DECISION_DIR / "ControllerDecision.cpp"
    ).read_text(encoding="utf-8")
    planning_source = (
        DECISION_DIR / "ControllerPlanning.cpp"
    ).read_text(encoding="utf-8")

    for interface in (
        "TierOneDecision",
        "TierTwoDecision",
        "TierThreeDecision",
    ):
        assert (
            f"std::unique_ptr<semaforr::decision::{interface}>" in header
        )

    for legacy_method in (
        "bool tierOneDecision(",
        "void tierTwoDecision(",
        "void tierThreeDecision(",
    ):
        assert legacy_method not in header

    assert "advisorVictory" not in decision_source
    assert "allAdvice" not in decision_source
    assert "getPlansWaypoints" not in planning_source


def test_default_tier_implementations_are_isolated_and_constructed_by_factory():
    expected = {
        "TierOneDecision.cpp": (
            "DefaultTierOneDecision",
            "makeTierOneDecision",
        ),
        "TierTwoDecision.cpp": (
            "DefaultTierTwoDecision",
            "makeTierTwoDecision",
        ),
        "TierThreeDecision.cpp": (
            "DefaultTierThreeDecision",
            "makeTierThreeDecision",
        ),
    }

    for filename, symbols in expected.items():
        source = (DECISION_DIR / filename).read_text(encoding="utf-8")
        for symbol in symbols:
            assert symbol in source
        assert "Controller::" not in source
