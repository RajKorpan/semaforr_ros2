import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def read(relative):
    return (SOURCE_DIR / relative).read_text(encoding="utf-8")


def test_upstream_social_messages_are_adapted_at_the_ros_boundary():
    adapters = read("src/ros/message_adapters.cpp")
    node = read("src/ros/semaforr_node_component.cpp")
    assert "TrackedPersonArray" in adapters + node
    assert "hunav_msgs::msg::Agents" in adapters + node
    assert "FormationGroupArray" in adapters + node
    assert "trackedPeopleToDomain" in adapters
    assert "hunavAgentsToDomain" in adapters
    assert "CrowdModel.msg" not in adapters + node
    assert "crowd_pose_all" not in adapters + node


def test_crowd_model_unifies_live_and_learned_social_state():
    model = read("include/semaforr/domain/crowd_model.hpp")
    for concept in (
        "CrowdState",
        "CrowdFieldSnapshot",
        "hasValidData",
        "learnedAt",
        "densityAt",
        "navigationRiskAt",
        "flowAlignmentAt",
    ):
        assert concept in model


def test_advisors_and_planners_consume_domain_crowd_model():
    advisor = read("src/decision/advisors/social/social_navigation_advisor.cpp")
    learned = read("src/decision/advisors/social/learned_crowd_advisor.cpp")
    planner = read("src/planning/domain_planner.cpp")
    assert "world.crowd" in advisor
    assert "world.crowd" in learned
    assert "request.crowd_model" in planner
    assert "social_context_msgs" not in advisor + learned + planner


def test_stale_social_data_has_explicit_fallback():
    advisor = read("src/decision/advisors/social/social_navigation_advisor.cpp")
    planner = read("src/planning/domain_planner.cpp")
    assert "current->usable" in advisor
    assert "advisor disabled" in advisor
    assert "sample->stale" in planner
