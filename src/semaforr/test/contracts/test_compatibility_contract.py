import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def test_compatibility_matrix_classifies_the_full_system():
    matrix = (SOURCE_DIR / "docs/compatibility-matrix.md").read_text(
        encoding="utf-8"
    )
    for status in (
        "Dissertation-faithful",
        "Functionally adapted",
        "Engineering extension",
        "Temporary approximation",
        "Unsupported or incomplete",
    ):
        assert status in matrix
    for component in (
        "Trail / `TrailLearner`",
        "Conveyor / `ConveyorLearner`",
        "Region / `RegionLearner`",
        "Door / `DoorExitLearner`",
        "Hallway / `HallwayLearner`",
        "Skeleton / `PassageSkeletonLearner`",
        "HLE",
        "Circumstance / `CircumstanceLearner`",
        "Tier ordering",
        "Enforcer",
        "Weighted signed voting",
        "Why action explanations",
        "Why plan comparison",
    ):
        assert component in matrix
    for required_column in (
        "Published algorithm",
        "Current implementation",
        "Intentional?",
        "Behavioral consequence",
        "Exact reproduction?",
        "Planned resolution",
    ):
        assert required_column in matrix


def test_behavior_mode_is_fingerprinted_manifested_and_fail_closed():
    header = (
        SOURCE_DIR
        / "include/semaforr/config/navigation_configuration.hpp"
    ).read_text(encoding="utf-8")
    configuration = (
        SOURCE_DIR / "src/config/navigation_configuration.cpp"
    ).read_text(encoding="utf-8")
    adapter = (SOURCE_DIR / "src/ros/parameter_configuration.cpp").read_text(
        encoding="utf-8"
    )
    yaml = (SOURCE_DIR / "config/semaforr.yaml").read_text(encoding="utf-8")
    assert "enum class BehaviorMode" in header
    assert "BehaviorMode::Modernized" in header
    assert '"experiment.behavior_mode"' in adapter
    assert "behavior_mode: modernized" in yaml
    assert "configuration.experiment.behavior_mode" in configuration
    assert '"behavior_mode:"' in configuration
    assert "compatibility' is reserved" in configuration
    assert "not operational" in configuration


def test_test_suites_make_their_behavior_claim_explicit():
    manifest = (SOURCE_DIR / "test/compatibility_modes.yaml").read_text(
        encoding="utf-8"
    )
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "behavior_mode:modernized" in manifest
    assert "behavior_mode:compatibility-contract" in manifest
    assert "compatibility:\n    enabled: false" in manifest
    assert "semaforr_compatibility_contract_test" in cmake
    assert "behavior_mode:modernized" in cmake
    assert "behavior_mode:compatibility-contract" in cmake

