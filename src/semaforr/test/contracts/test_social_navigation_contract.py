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
    revisions = read("include/semaforr/domain/model_revision.hpp")
    for concept in (
        "CrowdState",
        "CrowdFieldSnapshot",
        "hasValidData",
        "learnedAt",
        "densityAt",
        "navigationRiskAt",
        "flowAlignmentAt",
        "LiveCrowdObservation",
        "inputSource",
        "predictionSource",
        "formationEvidenceParticipated",
    ):
        assert concept in model + revisions


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


def test_structured_social_modes_validate_only_selected_dependencies():
    node = read("src/ros/semaforr_node_component.cpp")
    parameter_config = read("src/ros/parameter_configuration.cpp")
    config = read("config/semaforr.yaml")
    for mode in ("none", "tracked", "hunav"):
        assert mode in node
    for field in (
        "social.input.tracked_people_topic",
        "social.input.tracked_predictions_topic",
        "social.input.hunav_agents_topic",
        "social.input.hunav_predictions_topic",
        "social.input.formations_topic",
        "social.input.current_maximum_age_s",
        "social.input.prediction_maximum_age_s",
        "social.input.fallback_prediction",
    ):
        assert field in node
    assert "social.input.mode" in node
    assert "social.observations.enabled" not in node + config
    assert "!configuration.experiment.social.observations" in parameter_config
    assert "crowd_learning.enabled = false" in parameter_config
    assert "social:\n      enabled: true" in config
    assert "mode: tracked" in config


def test_crowd_visualization_is_direct_and_ros_transport_is_absent():
    visualization = read("src/ros/visualization_publisher.cpp")
    assert "world_.crowd" in visualization
    assert "nav_msgs::msg::OccupancyGrid" in visualization
    assert "visualization_msgs::msg::MarkerArray" in visualization
    for topic in (
        "topics.crowd_density",
        "topics.crowd_risk",
        "topics.crowd_flow",
        "topics.crowd_people",
        "topics.crowd_predictions",
        "topics.crowd_formations",
    ):
        assert topic in visualization
    assert "social_context_msgs" not in visualization
    assert "social_context_msgs::msg::CrowdField" not in visualization


def test_decisions_plans_replay_and_why_share_social_diagnostics():
    decision = read("include/semaforr/decision/decision_result.hpp")
    navigation = read("src/decision/navigation_engine.cpp")
    replay = read("src/validation/replay.cpp")
    decision_message = (SOURCE_DIR.parent / "semaforr_msgs/msg/DecisionRecord.msg").read_text(
        encoding="utf-8"
    )
    for field in (
        "live_social_revision",
        "crowd_density_revision",
        "crowd_risk_revision",
        "crowd_flow_revision",
        "social_input_source",
        "social_prediction_source",
        "social_input_status",
        "formation_evidence_participated",
    ):
        assert field in decision
        assert field in navigation
        assert field in decision_message
    assert "SOCIAL_DECISION" in replay
    why_package = (SOURCE_DIR.parent / "why/package.xml").read_text(encoding="utf-8")
    assert "semaforr_msgs" in why_package
    assert "social_context_msgs" not in why_package
