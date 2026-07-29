import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[1])
)
DECISION_DIR = SOURCE_DIR / "src" / "decision"


def test_decision_tiers_do_not_use_process_global_randomness():
    tier_sources = "\n".join(
        (DECISION_DIR / filename).read_text(encoding="utf-8")
        for filename in ("TierTwoDecision.cpp", "TierThreeDecision.cpp")
    )

    for token in ("srand(", "rand(", "time(nullptr)", "time(NULL)"):
        assert token not in tier_sources


def test_tier_three_uses_safe_deterministic_selection():
    source = (
        DECISION_DIR / "TierThreeDecision.cpp"
    ).read_text(encoding="utf-8")

    assert "selectHighestScoringAction(all_comments)" in source
    assert "if (!selection.selected)" in source
    assert "FORRAction(PAUSE, 0)" in source
    assert "% static_cast<int>" not in source


def test_tier_two_rejects_empty_and_invalid_plans():
    source = (
        DECISION_DIR / "TierTwoDecision.cpp"
    ).read_text(encoding="utf-8")

    assert "if (!candidate.empty())" in source
    assert "selectLowestCostPlan(total_costs)" in source
    assert "std::isfinite" in source
    assert "minimum_cost = 100000" not in source
