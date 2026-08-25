"""SemaFORR module overview.

Summary:
    This file exercises test configuration contract behavior for automated verification and regression testing. It centers on `test_runtime_configuration_is_parameter_only`, `test_structured_configuration_validates_all_runtime_invariants`, `test_legacy_converter_is_offline_only`, `test_replay_and_runtime_validation_fail_closed`. Its package-relative location is `test/contracts/test_configuration_contract.py`.

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


def test_runtime_configuration_is_parameter_only():
    """Summary:
        Performs the test runtime configuration is parameter only operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    adapter = (SOURCE_DIR / "src/ros/parameter_configuration.cpp").read_text(
        encoding="utf-8"
    )
    yaml = (SOURCE_DIR / "config/semaforr.yaml").read_text(encoding="utf-8")
    for obsolete in (
        "configuration.use_legacy_files",
        "configuration.legacy.",
        "semaforr_path",
        "target_set",
        "map_config",
        "map_dimensions",
    ):
        assert obsolete not in adapter
        assert obsolete not in yaml
    assert 'node.declare_parameter("map.mode"' in adapter
    assert "mapOperatingModeFromString" in adapter
    assert "mission.tasks_path: required path is empty" in adapter


def test_structured_configuration_validates_all_runtime_invariants():
    """Summary:
        Performs the test structured configuration validates all runtime invariants operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    source = (SOURCE_DIR / "src/config/navigation_configuration.cpp").read_text(
        encoding="utf-8"
    )
    for diagnostic in (
        "must not be empty",
        "must be finite",
        "must be positive",
        "unknown advisor",
        "planner",
        "map",
    ):
        assert diagnostic in source


def test_legacy_converter_is_offline_only():
    """Summary:
        Performs the test legacy converter is offline only operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "convert_legacy_config.py" in cmake
    assert "loadConfiguration({" not in (
        SOURCE_DIR / "src/ros/parameter_configuration.cpp"
    ).read_text(encoding="utf-8")


def test_replay_and_runtime_validation_fail_closed():
    """Summary:
        Performs the test replay and runtime validation fail closed operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    header = (SOURCE_DIR / "include/semaforr/validation/replay.hpp").read_text(
        encoding="utf-8"
    )
    configuration = (
        SOURCE_DIR / "src/config/navigation_configuration.cpp"
    ).read_text(encoding="utf-8")
    adapter = (SOURCE_DIR / "src/ros/navigation_engine_adapter.cpp").read_text(
        encoding="utf-8"
    )
    yaml = (SOURCE_DIR / "config/semaforr.yaml").read_text(encoding="utf-8")
    for field in (
        "tier_three_ties",
        "lle_fallback",
        "planner_ties",
        "clustering",
        "simulation_noise",
    ):
        assert field in header
        assert field in yaml
    assert "OfflineReplay" in header
    assert "loaded_highway_model is unsupported" in configuration
    assert "no highway-model loader is active" in configuration
    assert "retain_candidate_plans=true" in configuration
    assert "runtime_validation:passed" in adapter
