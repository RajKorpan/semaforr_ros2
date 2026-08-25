"""SemaFORR module overview.

Summary:
    This file exercises test documentation contract behavior for automated verification and regression testing. It centers on `read`, `test_every_document_is_indexed_and_every_local_link_resolves`, `test_registered_advisors_planners_tier_one_and_learners_are_cataloged`, `test_documented_topics_match_constructed_publishers_and_subscriptions`, `test_documented_launch_examples_use_real_arguments_and_installed_config`, `test_architecture_and_compatibility_claims_are_qualified`. Its package-relative location is `test/contracts/test_documentation_contract.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""









import importlib.util
import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
DOCS_DIR = SOURCE_DIR / "docs"


def test_every_package_file_and_source_api_has_structured_documentation():
    """Summary:
        Verifies file overviews and structured API documentation across the
        complete SemaFORR package source tree.

    Args:
        None.

    Returns:
        None; assertions report documentation coverage failures.

    Raises:
        ImportError: If the documentation audit module cannot be loaded.
        AssertionError: If a package file or detected API is undocumented.
    """
    script = SOURCE_DIR / "scripts" / "document_source_tree.py"
    specification = importlib.util.spec_from_file_location(
        "semaforr_document_source_tree", script
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)

    missing = module.process(SOURCE_DIR, audit=True)
    assert not missing, "missing structured documentation: " + ", ".join(
        str(path.relative_to(SOURCE_DIR)) for path in missing
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


def test_every_document_is_indexed_and_every_local_link_resolves():
    """Summary:
        Performs the test every document is indexed and every local link resolves operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    index = read("docs/README.md")
    for document in DOCS_DIR.glob("*.md"):
        if document.name != "README.md":
            assert f"({document.name})" in index, f"unindexed: {document.name}"

    for document in [SOURCE_DIR / "README.md", *DOCS_DIR.glob("*.md")]:
        text = document.read_text(encoding="utf-8")
        for raw_target in re.findall(r"\[[^]]+\]\(([^)]+)\)", text):
            target = raw_target.strip().split("#", 1)[0]
            if not target or "://" in target or target.startswith("mailto:"):
                continue
            resolved = (document.parent / target).resolve()
            assert resolved.exists(), f"broken link in {document}: {raw_target}"


def test_registered_advisors_planners_tier_one_and_learners_are_cataloged():
    """Summary:
        Performs the test registered advisors planners tier one and learners are cataloged operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    configuration = read("src/config/navigation_configuration.cpp")
    advisor_block = re.search(
        r"registered_advisors\{(.*?)\};", configuration, re.DOTALL
    )
    assert advisor_block
    advisors = set(re.findall(r'"([a-z0-9_]+)"', advisor_block.group(1)))
    assert len(advisors) == 34
    advisor_catalog = read("docs/advisor-catalog.md")
    for name in advisors:
        assert f"`{name}`" in advisor_catalog, f"advisor missing: {name}"

    planner_source = read("src/planning/planner_registry.cpp")
    planners = set(
        re.findall(r'(?:domain|learned)\(\s*"([a-z0-9_]+)"', planner_source)
    )
    planners.update(
        re.findall(r'registry\.add\(\s*"([a-z0-9_]+)"', planner_source)
    )
    assert planners == {
        "distance", "sensor_distance", "density", "risk", "flow",
        "region", "hallway", "trail", "conveyor", "skeleton", "highway",
    }
    planner_catalog = read("docs/planner-catalog.md")
    for name in planners:
        assert f"`{name}`" in planner_catalog, f"planner missing: {name}"

    tier_source = read("src/decision/tier_registry.cpp")
    tier_one = set(
        re.findall(
            r'tier_one\.register(?:Mandatory|Veto|Operationalizer|Reactive)'
            r'\(\s*"([a-z0-9_]+)"',
            tier_source,
        )
    )
    assert len(tier_one) == 10
    tier_catalog = read("docs/decision-tiers.md")
    for name in tier_one:
        assert f"`{name}`" in tier_catalog, f"Tier-1 component missing: {name}"

    coordinator = read("src/spatial/spatial_learning_coordinator.cpp")
    learners = set(re.findall(r"make_unique<(\w+Learner)>", coordinator))
    assert len(learners) == 12
    learner_catalog = read("docs/spatial-learning.md")
    for name in learners:
        assert f"`{name}`" in learner_catalog, f"learner missing: {name}"


def test_documented_topics_match_constructed_publishers_and_subscriptions():
    """Summary:
        Performs the test documented topics match constructed publishers and subscriptions operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    node = read("src/ros/semaforr_node_component.cpp")
    visualization = read("src/ros/visualization_publisher.cpp")
    topics = read("docs/topics-and-frames.md")

    topic_parameters = set(re.findall(
        r'declare_parameter(?:<[^>]+>)?\("('
        r'(?:topics\.[a-z_]+|social\.input\.[a-z_]+_topic))"',
        node,
    ))
    assert topic_parameters == {
        "topics.pose", "topics.scan", "topics.command",
        "topics.navigation_state", "topics.decision_records",
        "social.input.tracked_people_topic",
        "social.input.tracked_predictions_topic",
        "social.input.hunav_agents_topic",
        "social.input.hunav_predictions_topic",
        "social.input.formations_topic",
        "topics.crowd_density", "topics.crowd_risk", "topics.crowd_flow",
        "topics.crowd_people", "topics.crowd_predictions",
        "topics.crowd_formations",
    }
    for parameter in topic_parameters:
        assert f"`{parameter}`" in topics

    fixed_visualization_topics = set(
        re.findall(
            r'create_publisher<[^>]+>\(\s*"([a-z0-9_]+)"',
            visualization,
            re.DOTALL,
        )
    )
    assert fixed_visualization_topics == {
        "target_point", "waypoint", "plan", "static_map_geometry",
        "familiarity_grid", "sensed_occupancy_free",
        "sensed_occupancy_occupied", "static_map_occupancy", "decision_pose",
    }
    for name in fixed_visualization_topics:
        assert f"`{name}`" in topics


def test_documented_launch_examples_use_real_arguments_and_installed_config():
    """Summary:
        Performs the test documented launch examples use real arguments and installed config operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    launch = read("launch/example_simulation.launch.py")
    declared = set(re.findall(r'DeclareLaunchArgument\(\s*"([a-z0-9_]+)"', launch))
    examples = "\n".join(
        path.read_text(encoding="utf-8")
        for path in [SOURCE_DIR / "README.md", *DOCS_DIR.glob("*.md")]
    )
    used = set(re.findall(r"\b([a-z][a-z0-9_]*):=", examples))
    assert used <= declared, f"undocumented launch implementation: {used - declared}"
    assert 'str(config_dir / "semaforr.yaml")' in launch
    assert '"mission.tasks_path": str(example_dir / "mission.conf")' in launch
    assert (SOURCE_DIR / "config/semaforr.yaml").exists()
    assert (SOURCE_DIR / "config/example/mission.conf").exists()

    baseline_launch = read("launch/stage_tutorial_baseline.launch.py")
    baseline_arguments = re.findall(
        r'DeclareLaunchArgument\(\s*"([a-z0-9_]+)"', baseline_launch
    )
    assert len(baseline_arguments) == len(set(baseline_arguments))
    assert set(baseline_arguments) == {
        "profile", "random_seed", "output", "duration", "sensor_cutoff"
    }
    matrix = read("scripts/run_experiment_matrix.py")
    for argument in baseline_arguments:
        assert argument in matrix or argument == "sensor_cutoff"


def test_architecture_and_compatibility_claims_are_qualified():
    """Summary:
        Performs the test architecture and compatibility claims are qualified operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    architecture = " ".join(read("docs/architecture.md").split())
    for claim in (
        "SemaFORRNode::Impl` owns",
        "adapter owns the configuration",
        "shared world model",
        "`NavigationEngine` references adapter-owned objects",
        "simulator owns its environment geometry independently",
    ):
        assert claim in architecture

    compatibility = " ".join(read("docs/compatibility-matrix.md").split())
    assert "Status is an implementation target, not proof" in compatibility
    assert "No current whole-system run may be reported as an exact" in compatibility
    assert "compatibility` is reserved for exact experimental reproduction" in compatibility
    assert "currently rejects it" in compatibility
