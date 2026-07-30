import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def test_arbitration_has_explicit_safe_fallbacks_and_seeded_rng():
    header = (
        SOURCE_DIR / "include/semaforr/decision/decision_coordinator.hpp"
    ).read_text(encoding="utf-8")
    source = (SOURCE_DIR / "src/decision/DecisionCoordinator.cpp").read_text(
        encoding="utf-8"
    )
    assert "random_seed" in header
    assert "tie_tolerance" in header
    assert "fallback" in header
    assert "unscored_policy" in header
    assert "std::mt19937" in header
    assert "std::isfinite" in source
    assert "no_safe_candidate" in source
    assert "no_advisor_score" in source
    assert "std::sort" in source


def test_vetoed_actions_cannot_reenter_aggregation():
    source = (SOURCE_DIR / "src/decision/DecisionCoordinator.cpp").read_text(
        encoding="utf-8"
    )
    assert "vetoed.contains(candidate)" in source
    assert "scored an unavailable or vetoed action" in source
