"""SemaFORR module overview.

Summary:
    This file exercises test social navigation contract behavior for automated verification and regression testing. It centers on `read`, `test_upstream_social_messages_are_adapted_at_the_ros_boundary`, `test_crowd_model_unifies_live_and_learned_social_state`, `test_advisors_and_planners_consume_domain_crowd_model`, `test_stale_social_data_has_explicit_fallback`, `test_structured_social_modes_validate_only_selected_dependencies`, `test_crowd_visualization_is_direct_and_ros_transport_is_absent`, `test_decisions_plans_replay_and_why_share_social_diagnostics`. Its package-relative location is `test/contracts/test_social_navigation_contract.py`.

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


def test_upstream_social_messages_are_adapted_at_the_ros_boundary():
    """Summary:
        Performs the test upstream social messages are adapted at the ros boundary operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
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
    """Summary:
        Performs the test crowd model unifies live and learned social state operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
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
    """Summary:
        Performs the test advisors and planners consume domain crowd model operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    advisor = read("src/decision/advisors/social/social_navigation_advisor.cpp")
    learned = read("src/decision/advisors/social/learned_crowd_advisor.cpp")
    planner = read("src/planning/domain_planner.cpp")
    assert "world.crowd" in advisor
    assert "world.crowd" in learned
    assert "request.crowd_model" in planner
    assert "social_context_msgs" not in advisor + learned + planner


def test_stale_social_data_has_explicit_fallback():
    """Summary:
        Performs the test stale social data has explicit fallback operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    advisor = read("src/decision/advisors/social/social_navigation_advisor.cpp")
    planner = read("src/planning/domain_planner.cpp")
    assert "current->usable" in advisor
    assert "advisor disabled" in advisor
    assert "sample->stale" in planner


def test_structured_social_modes_validate_only_selected_dependencies():
    """Summary:
        Performs the test structured social modes validate only selected dependencies operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
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
    """Summary:
        Performs the test crowd visualization is direct and ros transport is absent operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
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
    """Summary:
        Performs the test decisions plans replay and why share social diagnostics operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
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
