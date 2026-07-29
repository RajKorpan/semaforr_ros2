import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
WORKSPACE_SRC = SOURCE_DIR.parent


def read(relative_path):
    return (WORKSPACE_SRC / relative_path).read_text(encoding="utf-8")


def test_one_canonical_social_observation_message():
    messages = WORKSPACE_SRC / "social_context_msgs" / "msg"
    canonical = read("social_context_msgs/msg/SocialObservation.msg")
    pedestrian = read("social_context_msgs/msg/PedestrianObservation.msg")

    assert "CrowdModel.msg" not in {
        path.name for path in (SOURCE_DIR / "msg").glob("*.msg")
    }
    assert "std_msgs/Header header" in canonical
    assert "PedestrianObservation[] pedestrians" in canonical
    for field in (
        "string id",
        "geometry_msgs/Point position",
        "geometry_msgs/Vector3 velocity",
        "geometry_msgs/Point[] predicted_positions",
        "builtin_interfaces/Time[] prediction_stamps",
        "float64 confidence",
        "float64[4] position_covariance",
    ):
        assert field in pedestrian
    assert (messages / "SocialObservation.msg").is_file()


def test_navigation_has_one_social_ros_input():
    node = read("semaforr/src/ros/SemaFORRNode.cpp")

    assert node.count(
        "create_subscription<\n        "
        "social_context_msgs::msg::SocialObservation>"
    ) == 1
    assert "crowd_pose" not in node
    assert "crowd_pose_all" not in node
    assert "CrowdModel" not in node
    assert "SocialObservationBuffer" in node
    assert "rotatePositionCovariance" in node


def test_social_domain_boundary_is_ros_independent():
    agent_state = read("semaforr/include/semaforr/decision/AgentState.hpp")
    planner = read("semaforr/include/semaforr/navigation/PathPlanner.hpp")
    social = read("semaforr/include/semaforr/domain/social.hpp")

    for source in (agent_state, planner, social):
        assert "geometry_msgs" not in source
        assert "social_context_msgs" not in source
        assert "rclcpp" not in source
    assert "CrowdState crowdState" in agent_state
    assert "CrowdState crowdState" in planner
    assert "data_age" in social


def test_producers_publish_the_canonical_api():
    tracker_bridge = read(
        "semaforr_bridge/semaforr_bridge/"
        "tracked_people_to_social_observation.py"
    )
    hunav = read("social_context/social_context/social_context_hunav.py")
    odometry_bridge = read(
        "semaforr_bridge/semaforr_bridge/odom_to_pose_bridge.py"
    )

    for producer in (tracker_bridge, hunav):
        assert "SocialObservation" in producer
        assert "PedestrianObservation" in producer
        assert "/social_observations" in producer
        assert "PoseStamped" not in producer
    assert "+ 100" not in odometry_bridge
    assert "+100" not in odometry_bridge


def test_social_consumers_and_stale_fallback_are_configured():
    configuration = read("semaforr/config/semaforr.yaml")
    advisor = read("semaforr/src/decision/SocialNavigationAdvisor.cpp")
    planner = read("semaforr/src/navigation/PathPlannerCosts.cpp")
    buffer = read("semaforr/src/ros/SocialObservationBuffer.cpp")

    for name in (
        "Interpersonal",
        "InterpersonalRotation",
        "CrowdAvoid",
        "CrowdAvoidRotation",
    ):
        assert f"- {name}" in configuration
    assert "maximum_age_s" in configuration
    assert "minimum_confidence" in configuration
    assert "social data absent, invalid, or stale; advisor disabled" in advisor
    assert advisor.index("return evaluation;") < advisor.index(
        "evaluation.participated = true"
    )
    assert "crowdState.current()" in planner
    assert "SocialObservationStatus::Stale" in buffer
