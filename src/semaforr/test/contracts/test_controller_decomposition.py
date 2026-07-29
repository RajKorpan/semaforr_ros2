import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
DECISION_DIR = SOURCE_DIR / "src" / "decision"

RESPONSIBILITIES = {
    "Controller.cpp": (
        "initialize_advisors",
        "initialize_planner",
        "initialize_tasks",
    ),
    "ControllerMission.cpp": (
        "updateState",
        "isMissionComplete",
        "decide",
    ),
    "ControllerLearning.cpp": (
        "learnSpatialModel",
        "updateSkeletonGraph",
    ),
    "ControllerDecision.cpp": (
        "FORRDecision",
    ),
    "ControllerPlanning.cpp": ("planForCurrentTask",),
}


def controller_sources():
    return {
        filename: (DECISION_DIR / filename).read_text(encoding="utf-8")
        for filename in RESPONSIBILITIES
    }


def test_controller_methods_are_grouped_by_responsibility():
    sources = controller_sources()
    all_source = "\n".join(sources.values())

    for expected_file, methods in RESPONSIBILITIES.items():
        for method in methods:
            definition = re.compile(rf"\bController::{method}\s*\(")
            assert definition.search(sources[expected_file])
            assert len(definition.findall(all_source)) == 1


def test_controller_translation_units_remain_focused():
    sources = controller_sources()

    assert len(sources["Controller.cpp"].splitlines()) < 250
    for filename, source in sources.items():
        assert len(source.splitlines()) < 350, filename

    assert "tierThreeAdvisorInfluence" not in "\n".join(sources.values())
    assert "isAdvisorActive" not in "\n".join(sources.values())
    assert "Controller::tierOneDecision" not in "\n".join(sources.values())
    assert "Controller::tierTwoDecision" not in "\n".join(sources.values())
    assert "Controller::tierThreeDecision" not in "\n".join(sources.values())


def test_controller_public_facade_remains_stable():
    header = (
        SOURCE_DIR / "include" / "semaforr" / "decision" / "Controller.hpp"
    ).read_text(encoding="utf-8")

    for signature in (
        "semaforr::decision::DecisionResult decide()",
        "void updateState(",
        "bool isMissionComplete()",
        "Beliefs *getBeliefs()",
        "PathPlanner *getPlanner()",
        "std::vector<PathPlanner*> getPlanners()",
    ):
        assert signature in header

    assert "tierThreeAdvisorInfluence" not in header
    assert "isAdvisorActive" not in header
